//! Joint constraint solver for the 2D physics backend.
//!
//! Joints are velocity-level constraints solved with accumulated, clamped
//! impulses (sequential impulses), interleaved with the contact solver:
//!
//! - rigid constraints (point, distance, angle, axis) use a Baumgarte bias
//!   `β·C/dt` to remove drift;
//! - springs (Spring, Wheel suspension, Mouse) and joint damping are *soft*
//!   constraints (Box2D's implicit spring formulation), so their response is
//!   stable for stiff springs and independent of the iteration count;
//! - limits are one-sided (speculative) constraints and motors are velocity
//!   constraints whose accumulated impulse is clamped to `max_force · dt`.
//!
//! Geometry (anchors, errors, effective masses) is computed once per sub-step
//! in [`PhysicsState2d::prepare_joints`]; positions don't change while
//! velocities are iterated.

use crate::arena::ArenaHandle;
use crate::joint::JointType;

use super::body_ah;
use super::state::PhysicsState2d;
use super::types::{EPSILON, Joint2d, RigidBody2d};

/// Fraction of a rigid joint's position error removed per step.
const JOINT_BAUMGARTE: f64 = 0.2;

// ---------------------------------------------------------------------------
// Small vector helpers
// ---------------------------------------------------------------------------

#[inline]
fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

#[inline]
fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

#[inline]
fn rotate(v: [f64; 2], rot: f64) -> [f64; 2] {
    let (s, c) = rot.sin_cos();
    [c * v[0] - s * v[1], s * v[0] + c * v[1]]
}

#[inline]
fn normalize(v: [f64; 2]) -> Option<[f64; 2]> {
    let len = v[0].hypot(v[1]);
    (len > EPSILON && len.is_finite()).then(|| [v[0] / len, v[1] / len])
}

/// Soft-constraint coefficients for a spring of stiffness `k` and damping `c`
/// (Box2D): returns `(gamma, bias_per_unit_error)`, or `None` if both are zero.
#[inline]
fn soft(k: f64, c: f64, dt: f64) -> Option<(f64, f64)> {
    let k = if k.is_finite() { k.max(0.0) } else { 0.0 };
    let c = if c.is_finite() { c.max(0.0) } else { 0.0 };
    let g = dt * (c + dt * k);
    (g > EPSILON).then(|| {
        let gamma = 1.0 / g;
        (gamma, dt * k * gamma)
    })
}

/// Clamp bound for a motor/mouse impulse: `|max_force|·dt`; NaN → 0.
#[inline]
fn impulse_cap(max_force: f64, dt: f64) -> f64 {
    let cap = (max_force * dt).abs();
    if cap.is_nan() { 0.0 } else { cap }
}

// ---------------------------------------------------------------------------
// Constraint rows
// ---------------------------------------------------------------------------

/// Body data the solver needs, copied once per sub-step.
#[derive(Clone, Copy)]
struct BodyRef {
    handle: Option<ArenaHandle>,
    inv_mass: f64,
    inv_inertia: f64,
}

impl BodyRef {
    fn of(handle: ArenaHandle, b: Option<&RigidBody2d>) -> Self {
        match b {
            Some(b) => {
                let (inv_mass, inv_inertia) = b.solver_inv_mass();
                Self {
                    handle: Some(handle),
                    inv_mass,
                    inv_inertia,
                }
            }
            None => Self {
                handle: None,
                inv_mass: 0.0,
                inv_inertia: 0.0,
            },
        }
    }
}

/// A scalar velocity constraint `Cdot = lin·(vB − vA) + ang_b·ωB − ang_a·ωA`.
#[derive(Clone, Copy)]
struct Row {
    lin: [f64; 2],
    ang_a: f64,
    ang_b: f64,
    mass: f64,
    bias: f64,
    gamma: f64,
    lo: f64,
    hi: f64,
    acc: f64,
}

impl Row {
    fn new(lin: [f64; 2], ang_a: f64, ang_b: f64, a: &BodyRef, b: &BodyRef, gamma: f64) -> Self {
        let k = (a.inv_mass + b.inv_mass) * dot(lin, lin)
            + a.inv_inertia * ang_a * ang_a
            + b.inv_inertia * ang_b * ang_b
            + gamma;
        Self {
            lin,
            ang_a,
            ang_b,
            mass: if k > EPSILON { 1.0 / k } else { 0.0 },
            bias: 0.0,
            gamma,
            lo: f64::NEG_INFINITY,
            hi: f64::INFINITY,
            acc: 0.0,
        }
    }

    /// Rigid bilateral constraint with Baumgarte bias.
    fn rigid(mut self, error: f64, dt: f64) -> Self {
        self.bias = JOINT_BAUMGARTE * error / dt;
        self
    }

    /// One-sided constraint `C ≥ 0` (impulse ≥ 0). While separated the bias
    /// lets it close the gap within this step; once violated it corrects.
    fn one_sided(mut self, c: f64, dt: f64) -> Self {
        self.bias = if c > 0.0 {
            c / dt
        } else {
            JOINT_BAUMGARTE * c / dt
        };
        self.lo = 0.0;
        self
    }

    fn solve(&mut self, va: &mut Vel, vb: &mut Vel, a: &BodyRef, b: &BodyRef) {
        if self.mass == 0.0 {
            return;
        }
        let cdot = dot(self.lin, [vb.v[0] - va.v[0], vb.v[1] - va.v[1]]) + self.ang_b * vb.w
            - self.ang_a * va.w;
        let lambda = -self.mass * (cdot + self.bias + self.gamma * self.acc);
        let mut new = (self.acc + lambda).clamp(self.lo, self.hi);
        if !new.is_finite() {
            new = 0.0;
        }
        let d = new - self.acc;
        self.acc = new;
        va.v[0] -= self.lin[0] * d * a.inv_mass;
        va.v[1] -= self.lin[1] * d * a.inv_mass;
        va.w -= self.ang_a * d * a.inv_inertia;
        vb.v[0] += self.lin[0] * d * b.inv_mass;
        vb.v[1] += self.lin[1] * d * b.inv_mass;
        vb.w += self.ang_b * d * b.inv_inertia;
    }
}

/// A 2D point-to-point constraint (anchors coincide), solved as a 2x2 block.
#[derive(Clone, Copy)]
struct PointBlock {
    ra: [f64; 2],
    rb: [f64; 2],
    /// Inverse of the effective-mass matrix `K + γI`.
    inv_k: [[f64; 2]; 2],
    bias: [f64; 2],
    gamma: f64,
    /// Maximum accumulated impulse magnitude (∞ = rigid).
    max_impulse: f64,
    acc: [f64; 2],
}

impl PointBlock {
    fn new(ra: [f64; 2], rb: [f64; 2], a: &BodyRef, b: &BodyRef, gamma: f64) -> Option<Self> {
        let (ma, ia, mb, ib) = (a.inv_mass, a.inv_inertia, b.inv_mass, b.inv_inertia);
        let k11 = ma + mb + ia * ra[1] * ra[1] + ib * rb[1] * rb[1] + gamma;
        let k12 = -ia * ra[0] * ra[1] - ib * rb[0] * rb[1];
        let k22 = ma + mb + ia * ra[0] * ra[0] + ib * rb[0] * rb[0] + gamma;
        let det = k11 * k22 - k12 * k12;
        if det.is_nan() || det.abs() <= EPSILON || !det.is_finite() {
            return None;
        }
        let inv = 1.0 / det;
        Some(Self {
            ra,
            rb,
            inv_k: [[k22 * inv, -k12 * inv], [-k12 * inv, k11 * inv]],
            bias: [0.0, 0.0],
            gamma,
            max_impulse: f64::INFINITY,
            acc: [0.0, 0.0],
        })
    }

    fn solve(&mut self, va: &mut Vel, vb: &mut Vel, a: &BodyRef, b: &BodyRef) {
        let cdot = {
            let pa = va.at(self.ra);
            let pb = vb.at(self.rb);
            [pb[0] - pa[0], pb[1] - pa[1]]
        };
        let rhs = [
            cdot[0] + self.bias[0] + self.gamma * self.acc[0],
            cdot[1] + self.bias[1] + self.gamma * self.acc[1],
        ];
        let lambda = [
            -(self.inv_k[0][0] * rhs[0] + self.inv_k[0][1] * rhs[1]),
            -(self.inv_k[1][0] * rhs[0] + self.inv_k[1][1] * rhs[1]),
        ];
        let mut new = [self.acc[0] + lambda[0], self.acc[1] + lambda[1]];
        let mag = new[0].hypot(new[1]);
        if mag > self.max_impulse {
            new = [
                new[0] * self.max_impulse / mag,
                new[1] * self.max_impulse / mag,
            ];
        }
        if !(new[0].is_finite() && new[1].is_finite()) {
            new = [0.0, 0.0];
        }
        let d = [new[0] - self.acc[0], new[1] - self.acc[1]];
        self.acc = new;
        va.v[0] -= d[0] * a.inv_mass;
        va.v[1] -= d[1] * a.inv_mass;
        va.w -= cross(self.ra, d) * a.inv_inertia;
        vb.v[0] += d[0] * b.inv_mass;
        vb.v[1] += d[1] * b.inv_mass;
        vb.w += cross(self.rb, d) * b.inv_inertia;
    }
}

#[derive(Clone, Copy)]
struct Vel {
    v: [f64; 2],
    w: f64,
}

impl Vel {
    #[inline]
    fn at(&self, r: [f64; 2]) -> [f64; 2] {
        [self.v[0] - self.w * r[1], self.v[1] + self.w * r[0]]
    }
}

/// One joint's constraints for the current sub-step.
pub(super) struct PreparedJoint {
    handle: ArenaHandle,
    a: BodyRef,
    b: BodyRef,
    point: Option<PointBlock>,
    rows: Vec<Row>,
    /// Index into `rows` of the row whose impulse counts toward breaking
    /// (when there is no point block).
    break_row: Option<usize>,
    break_force: Option<f64>,
}

impl PreparedJoint {
    /// Linear constraint force magnitude this step (for `break_force`).
    fn linear_force(&self, dt: f64) -> f64 {
        let impulse = match (&self.point, self.break_row) {
            (Some(p), _) => p.acc[0].hypot(p.acc[1]),
            (None, Some(i)) => self.rows.get(i).map_or(0.0, |r| r.acc.abs()),
            (None, None) => 0.0,
        };
        if dt > EPSILON { impulse / dt } else { 0.0 }
    }
}

impl PhysicsState2d {
    /// Build every joint's constraint rows for this sub-step.
    pub(super) fn prepare_joints(&self, dt: f64) -> Vec<PreparedJoint> {
        if dt <= EPSILON {
            return Vec::new();
        }
        self.joints
            .iter()
            .filter_map(|(handle, joint)| self.prepare_joint(handle, joint, dt))
            .collect()
    }

    fn prepare_joint(&self, handle: ArenaHandle, j: &Joint2d, dt: f64) -> Option<PreparedJoint> {
        let ah_a = body_ah(j.body_a);
        let ah_b = body_ah(j.body_b);
        let ba = self.bodies.get(ah_a);
        let bb = self.bodies.get(ah_b);
        let a = BodyRef::of(ah_a, ba);
        let b = BodyRef::of(ah_b, bb);

        // Mouse joint drags body A toward a world target; treat the target as
        // a static "body".
        if let JointType::Mouse {
            target,
            stiffness,
            damping,
            max_force,
        } = &j.joint_type
        {
            let ba = ba?;
            let anchor = rotate(j.local_anchor_a, ba.rotation);
            let ground = BodyRef {
                handle: None,
                inv_mass: 0.0,
                inv_inertia: 0.0,
            };
            let (gamma, beta) = soft(*stiffness, *damping, dt).unwrap_or((0.0, 0.0));
            // Ground is "A", the dragged body is "B": C = anchor − target.
            let mut p = PointBlock::new([0.0, 0.0], anchor, &ground, &a, gamma)?;
            let c = [
                ba.position[0] + anchor[0] - target[0],
                ba.position[1] + anchor[1] - target[1],
            ];
            p.bias = [c[0] * beta, c[1] * beta];
            p.max_impulse = impulse_cap(*max_force, dt);
            return Some(PreparedJoint {
                handle,
                a: ground,
                b: a,
                point: Some(p),
                rows: Vec::new(),
                break_row: None,
                break_force: j.break_force,
            });
        }

        let (pa, rot_a) = ba.map_or(([0.0, 0.0], 0.0), |b| (b.position, b.rotation));
        let (pb, rot_b) = bb.map_or(([0.0, 0.0], 0.0), |b| (b.position, b.rotation));
        if a.inv_mass == 0.0 && b.inv_mass == 0.0 && a.inv_inertia == 0.0 && b.inv_inertia == 0.0 {
            return None;
        }
        // Anchors relative to each body centre; a missing body anchors in world space.
        let ra = if ba.is_some() {
            rotate(j.local_anchor_a, rot_a)
        } else {
            [0.0, 0.0]
        };
        let rb = if bb.is_some() {
            rotate(j.local_anchor_b, rot_b)
        } else {
            [0.0, 0.0]
        };
        let anchor_a = if ba.is_some() {
            [pa[0] + ra[0], pa[1] + ra[1]]
        } else {
            j.local_anchor_a
        };
        let anchor_b = if bb.is_some() {
            [pb[0] + rb[0], pb[1] + rb[1]]
        } else {
            j.local_anchor_b
        };
        let d = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];
        let angle = rot_b - rot_a;

        let mut point = None;
        let mut rows: Vec<Row> = Vec::new();
        let mut break_row = None;

        let rigid_point = |gamma: f64| {
            PointBlock::new(ra, rb, &a, &b, gamma).map(|mut p| {
                p.bias = [JOINT_BAUMGARTE * d[0] / dt, JOINT_BAUMGARTE * d[1] / dt];
                p
            })
        };
        let angular_row = || Row::new([0.0, 0.0], 1.0, 1.0, &a, &b, 0.0);
        // Row along unit direction `u` between the anchors.
        let anchor_row =
            |u: [f64; 2], gamma: f64| Row::new(u, cross(ra, u), cross(rb, u), &a, &b, gamma);
        // Row along a direction fixed in body A (prismatic/wheel): A's
        // angular term uses the full A-centre → B-anchor arm.
        let axis_row = |u: [f64; 2]| {
            let arm_a = [d[0] + ra[0], d[1] + ra[1]];
            Row::new(u, cross(arm_a, u), cross(rb, u), &a, &b, 0.0)
        };

        match &j.joint_type {
            JointType::Fixed => {
                point = rigid_point(0.0);
                rows.push(angular_row().rigid(angle - j.reference_angle, dt));
            }
            JointType::Revolute { limits, .. } => {
                point = rigid_point(0.0);
                if let Some([lo, hi]) = limits {
                    rows.push(angular_row().one_sided(angle - lo, dt));
                    let mut upper = Row::new([0.0, 0.0], -1.0, -1.0, &a, &b, 0.0);
                    upper = upper.one_sided(hi - angle, dt);
                    rows.push(upper);
                }
                if let Some(m) = &j.motor {
                    let mut r = angular_row();
                    r.bias = -m.target_velocity;
                    let cap = impulse_cap(m.max_force, dt);
                    r.lo = -cap;
                    r.hi = cap;
                    rows.push(r);
                }
            }
            JointType::Prismatic { axis, limits } => {
                let ax = normalize(rotate(*axis, rot_a))?;
                let perp = [-ax[1], ax[0]];
                break_row = Some(rows.len());
                rows.push(axis_row(perp).rigid(dot(d, perp), dt));
                rows.push(angular_row().rigid(angle - j.reference_angle, dt));
                let translation = dot(d, ax);
                if let Some([lo, hi]) = limits {
                    rows.push(axis_row(ax).one_sided(translation - lo, dt));
                    let mut upper = axis_row(ax);
                    upper.lin = [-ax[0], -ax[1]];
                    upper.ang_a = -upper.ang_a;
                    upper.ang_b = -upper.ang_b;
                    rows.push(upper.one_sided(hi - translation, dt));
                }
                if let Some(m) = &j.motor {
                    let mut r = axis_row(ax);
                    r.bias = -m.target_velocity;
                    let cap = impulse_cap(m.max_force, dt);
                    r.lo = -cap;
                    r.hi = cap;
                    rows.push(r);
                }
            }
            JointType::Wheel {
                axis,
                stiffness,
                damping,
            } => {
                let ax = normalize(rotate(*axis, rot_a))?;
                let perp = [-ax[1], ax[0]];
                break_row = Some(rows.len());
                rows.push(axis_row(perp).rigid(dot(d, perp), dt));
                if let Some((gamma, beta)) = soft(*stiffness, *damping, dt) {
                    let mut r = axis_row(ax);
                    r = Row::new(r.lin, r.ang_a, r.ang_b, &a, &b, gamma);
                    r.bias = dot(d, ax) * beta;
                    rows.push(r);
                }
            }
            JointType::Spring {
                rest_length,
                stiffness,
                damping,
            } => {
                let u = normalize(d)?;
                let (gamma, beta) = soft(*stiffness, *damping, dt)?;
                let mut r = anchor_row(u, gamma);
                r.bias = (d[0].hypot(d[1]) - rest_length) * beta;
                break_row = Some(rows.len());
                rows.push(r);
            }
            JointType::Distance { length } => {
                let u = normalize(d)?;
                break_row = Some(rows.len());
                rows.push(anchor_row(u, 0.0).rigid(d[0].hypot(d[1]) - length, dt));
            }
            JointType::Rope { max_length } => {
                let u = normalize(d)?;
                // C = max_length − dist ≥ 0: impulse pulls the anchors together.
                let r = Row::new([-u[0], -u[1]], -cross(ra, u), -cross(rb, u), &a, &b, 0.0);
                break_row = Some(rows.len());
                rows.push(r.one_sided(max_length - d[0].hypot(d[1]), dt));
            }
            JointType::Mouse { .. } => return None,
        }

        // Generic joint damping (springs, wheels and mice have their own).
        if j.damping > 0.0 && j.damping.is_finite() {
            let gamma = 1.0 / (dt * j.damping);
            match &j.joint_type {
                JointType::Revolute { .. } | JointType::Wheel { .. } => {
                    rows.push(Row::new([0.0, 0.0], 1.0, 1.0, &a, &b, gamma));
                }
                JointType::Prismatic { axis, .. } => {
                    if let Some(ax) = normalize(rotate(*axis, rot_a)) {
                        let r = axis_row(ax);
                        rows.push(Row::new(r.lin, r.ang_a, r.ang_b, &a, &b, gamma));
                    }
                }
                JointType::Distance { .. } | JointType::Rope { .. } => {
                    if let Some(u) = normalize(d) {
                        rows.push(anchor_row(u, gamma));
                    }
                }
                _ => {}
            }
        }

        Some(PreparedJoint {
            handle,
            a,
            b,
            point,
            rows,
            break_row,
            break_force: j.break_force,
        })
    }

    /// One Gauss–Seidel pass over every prepared joint.
    pub(super) fn solve_joints_once(&mut self, joints: &mut [PreparedJoint]) {
        for pj in joints.iter_mut() {
            let read = |h: Option<ArenaHandle>| {
                h.and_then(|h| self.bodies.get(h)).map_or(
                    Vel {
                        v: [0.0, 0.0],
                        w: 0.0,
                    },
                    |b| Vel {
                        v: b.linear_velocity,
                        w: b.angular_velocity,
                    },
                )
            };
            let mut va = read(pj.a.handle);
            let mut vb = read(pj.b.handle);
            for row in &mut pj.rows {
                row.solve(&mut va, &mut vb, &pj.a, &pj.b);
            }
            if let Some(p) = &mut pj.point {
                p.solve(&mut va, &mut vb, &pj.a, &pj.b);
            }
            for (side, vel) in [(&pj.a, va), (&pj.b, vb)] {
                if (side.inv_mass > 0.0 || side.inv_inertia > 0.0)
                    && let Some(b) = side.handle.and_then(|h| self.bodies.get_mut(h))
                {
                    b.linear_velocity = vel.v;
                    b.angular_velocity = vel.w;
                }
            }
        }
    }

    /// Remove joints whose linear constraint force exceeded `break_force`.
    pub(super) fn break_joints(&mut self, joints: &[PreparedJoint], dt: f64) {
        for pj in joints {
            if let Some(limit) = pj.break_force
                && pj.linear_force(dt) > limit
            {
                self.joints.remove(pj.handle);
            }
        }
    }
}
