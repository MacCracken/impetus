//! Joint constraint solvers for the 3D physics backend.
//!
//! - **Fixed**, **Distance**, **Rope** and the **Wheel**'s perpendicular rows
//!   are velocity-level constraints solved with sequential impulses. Each row
//!   is mass-weighted with both linear and angular terms; position drift is
//!   removed with split-impulse pseudo-velocities (no energy is injected).
//!   A Fixed joint locks both the anchor points and the relative rotation
//!   the bodies had when the joint was created.
//! - **Spring**, **Wheel** suspension, **Mouse** and the per-joint `damping`
//!   are explicit forces, applied once per sub-step (not once per velocity
//!   iteration).
//! - **Revolute** and **Prismatic** joints and joint motors are **not
//!   implemented in 3D** (they need 3D axis definitions). They are silent
//!   no-ops apart from the per-joint `damping`.
//!
//! Joint axes (`Wheel::axis`) are world-space directions in the XY plane;
//! local anchors are the 2D `local_anchor_*` with `z = 0`.

use hisab::{DMat3, DQuat, DVec3};

use crate::arena::ArenaHandle;
use crate::body::BodyHandle;
use crate::joint::JointType;

use super::body_ah;
use super::solver::solver_mass;
use super::state::PhysicsState3d;
use super::types::{EPSILON, Joint3d, skew};

/// Velocity-level rows of a joint for one sub-step.
///
/// Built once per sub-step into a `Vec`; boxing the large Fixed variant would
/// only add an allocation per joint.
#[derive(Debug, Clone)]
#[allow(clippy::large_enum_variant)]
enum JointRows {
    /// Explicit-force-only joints: nothing to solve.
    None,
    /// Anchor coincidence (3 rows) and, if solvable, relative-rotation lock
    /// (3 rows), solved together as one coupled 6×6 block.
    Fixed {
        /// Inverse of the point block's effective mass: the Schur complement
        /// `K_pp − K_pl·K_ll⁻¹·K_lp` when the rotation is locked, else `K_pp`.
        point_mass: DMat3,
        /// Coupling blocks for the rotation lock, if `K_ll` is invertible.
        angular: Option<FixedAngular>,
        /// `anchor_b − anchor_a`.
        point_error: DVec3,
        /// Small-angle rotation error of B relative to its locked rotation.
        angular_error: DVec3,
        impulse: DVec3,
    },
    /// Distance (equality) or rope (upper limit only) along `axis` (A -> B).
    Axial {
        axis: DVec3,
        mass: f64,
        /// `distance − length`.
        error: f64,
        rope: bool,
        impulse: f64,
        position_impulse: f64,
    },
    /// Wheel: no relative motion perpendicular to the suspension axis.
    Perpendicular {
        axes: [DVec3; 2],
        masses: [f64; 2],
        errors: [f64; 2],
        impulse: [f64; 2],
    },
}

/// Effective-mass blocks coupling a Fixed joint's point and angular rows.
#[derive(Debug, Clone, Copy)]
struct FixedAngular {
    /// Anchor-velocity response to an angular impulse.
    k_pl: DMat3,
    /// Angular-velocity response to a point impulse.
    k_lp: DMat3,
    /// `(I_a⁻¹ + I_b⁻¹)⁻¹`.
    k_ll_inv: DMat3,
}

/// Per-joint solver state for one sub-step.
#[derive(Debug, Clone)]
pub(super) struct JointConstraint {
    handle: ArenaHandle,
    body_a: BodyHandle,
    body_b: BodyHandle,
    inv_mass_a: f64,
    inv_mass_b: f64,
    inv_i_a: DMat3,
    inv_i_b: DMat3,
    ra: DVec3,
    rb: DVec3,
    rows: JointRows,
    /// Magnitude of the explicit force applied this sub-step (N).
    explicit_force: f64,
    break_force: Option<f64>,
    broken: bool,
}

/// Scalar effective mass `1 / K` along `d` for lever arms `ra`, `rb`.
#[inline]
#[must_use]
#[allow(clippy::too_many_arguments)]
fn axial_mass(
    d: DVec3,
    ra: DVec3,
    rb: DVec3,
    inv_mass_a: f64,
    inv_mass_b: f64,
    inv_i_a: DMat3,
    inv_i_b: DMat3,
) -> f64 {
    let ka = ra.cross(d);
    let kb = rb.cross(d);
    let k = inv_mass_a + inv_mass_b + ka.dot(inv_i_a * ka) + kb.dot(inv_i_b * kb);
    if k > EPSILON { 1.0 / k } else { 0.0 }
}

/// Inverse of a symmetric 3×3 effective-mass matrix, if well-conditioned.
#[inline]
#[must_use]
fn invert_mass(k: DMat3) -> Option<DMat3> {
    let det = k.determinant();
    (det.is_finite() && det > EPSILON).then(|| k.inverse())
}

/// Small-angle error vector of rotation `q` (shortest arc).
#[inline]
#[must_use]
fn rotation_error(q: DQuat) -> DVec3 {
    let q = if q.w < 0.0 { -q } else { q };
    DVec3::new(q.x, q.y, q.z) * 2.0
}

impl PhysicsState3d {
    // -----------------------------------------------------------------------
    // Joint constraint solver
    // -----------------------------------------------------------------------

    /// Apply explicit joint forces once for this sub-step and build the
    /// velocity-level constraint rows.
    pub(super) fn prepare_joints(&mut self, dt: f64) -> Vec<JointConstraint> {
        let joints: Vec<(ArenaHandle, Joint3d)> =
            self.joints.iter().map(|(ah, j)| (ah, j.clone())).collect();
        let mut out = Vec::with_capacity(joints.len());
        for (handle, joint) in joints {
            let explicit_force = self.apply_joint_force(&joint, dt);
            if let Some(mut c) = self.joint_rows(&joint, handle) {
                c.explicit_force = explicit_force;
                out.push(c);
            } else if let Some(bf) = joint.break_force
                && explicit_force > bf
            {
                self.joints.remove(handle);
            }
        }
        out
    }

    /// Build the constraint rows for a joint from the current positions.
    #[must_use]
    fn joint_rows(&self, joint: &Joint3d, handle: ArenaHandle) -> Option<JointConstraint> {
        let ba = self.bodies.get(body_ah(joint.body_a))?;
        let bb = self.bodies.get(body_ah(joint.body_b))?;
        let (inv_mass_a, inv_i_a) = solver_mass(ba);
        let (inv_mass_b, inv_i_b) = solver_mass(bb);
        let ra = ba.rotation * joint.local_anchor_a;
        let rb = bb.rotation * joint.local_anchor_b;
        let diff = (bb.position + rb) - (ba.position + ra);
        let mass = |d: DVec3| axial_mass(d, ra, rb, inv_mass_a, inv_mass_b, inv_i_a, inv_i_b);

        let rows = if inv_mass_a == 0.0 && inv_mass_b == 0.0 {
            JointRows::None
        } else {
            match &joint.joint_type {
                JointType::Fixed => {
                    let (sa, sb) = (skew(ra), skew(rb));
                    let k_pp = DMat3::from_diagonal(DVec3::splat(inv_mass_a + inv_mass_b))
                        - sa * inv_i_a * sa
                        - sb * inv_i_b * sb;
                    let k_pl = -(sb * inv_i_b) - sa * inv_i_a;
                    let k_lp = inv_i_b * sb + inv_i_a * sa;
                    let block = invert_mass(inv_i_a + inv_i_b).and_then(|k_ll_inv| {
                        invert_mass(k_pp - k_pl * k_ll_inv * k_lp).map(|point_mass| {
                            (
                                point_mass,
                                Some(FixedAngular {
                                    k_pl,
                                    k_lp,
                                    k_ll_inv,
                                }),
                            )
                        })
                    });
                    match block.or_else(|| invert_mass(k_pp).map(|m| (m, None))) {
                        Some((point_mass, angular)) => JointRows::Fixed {
                            point_mass,
                            angular,
                            point_error: diff,
                            angular_error: rotation_error(
                                bb.rotation * (ba.rotation * joint.reference_rotation).inverse(),
                            ),
                            impulse: DVec3::ZERO,
                        },
                        None => JointRows::None,
                    }
                }
                JointType::Distance { length } | JointType::Rope { max_length: length } => {
                    let dist = diff.length();
                    if dist < EPSILON || !length.is_finite() {
                        JointRows::None
                    } else {
                        let axis = diff / dist;
                        JointRows::Axial {
                            axis,
                            mass: mass(axis),
                            error: dist - length,
                            rope: matches!(joint.joint_type, JointType::Rope { .. }),
                            impulse: 0.0,
                            position_impulse: 0.0,
                        }
                    }
                }
                JointType::Wheel { axis, .. } => {
                    let ax = DVec3::new(axis[0], axis[1], 0.0);
                    let len = ax.length();
                    if len < EPSILON || !len.is_finite() {
                        JointRows::None
                    } else {
                        let ax = ax / len;
                        let p1 = DVec3::new(-ax.y, ax.x, 0.0);
                        let p2 = ax.cross(p1);
                        JointRows::Perpendicular {
                            axes: [p1, p2],
                            masses: [mass(p1), mass(p2)],
                            errors: [diff.dot(p1), diff.dot(p2)],
                            impulse: [0.0, 0.0],
                        }
                    }
                }
                _ => JointRows::None,
            }
        };

        Some(JointConstraint {
            handle,
            body_a: joint.body_a,
            body_b: joint.body_b,
            inv_mass_a,
            inv_mass_b,
            inv_i_a,
            inv_i_b,
            ra,
            rb,
            rows,
            explicit_force: 0.0,
            break_force: joint.break_force,
            broken: false,
        })
    }

    /// Solve a Fixed joint's coupled point + angular block in one shot:
    /// `[K_pp K_pl; K_lp K_ll]·[P; L] = −[ċ_p + b_p; ċ_l + b_l]`, via the
    /// Schur complement. Returns the point impulse `P`.
    fn solve_fixed_block(
        &mut self,
        c: &JointConstraint,
        point_mass: DMat3,
        angular: Option<&FixedAngular>,
        bias: Option<(DVec3, DVec3)>,
        pseudo: bool,
    ) -> DVec3 {
        let (v_rel, w_rel) = self.joint_relative_velocity(c, pseudo);
        let (bias_p, bias_l) = bias.unwrap_or((DVec3::ZERO, DVec3::ZERO));
        let rhs_p = -(v_rel + bias_p);
        let (p, l) = match angular {
            Some(a) => {
                let rhs_l = -(w_rel + bias_l);
                let p = point_mass * (rhs_p - a.k_pl * (a.k_ll_inv * rhs_l));
                let l = a.k_ll_inv * (rhs_l - a.k_lp * p);
                (p, l)
            }
            None => (point_mass * rhs_p, DVec3::ZERO),
        };
        if p.is_finite() && l.is_finite() {
            self.apply_joint_impulse(c, p, l, pseudo);
            p
        } else {
            DVec3::ZERO
        }
    }

    /// Apply a linear impulse at the anchors (−p on A, +p on B) and an
    /// angular impulse (−l on A, +l on B), to velocities or pseudo-velocities.
    #[inline]
    fn apply_joint_impulse(&mut self, c: &JointConstraint, p: DVec3, l: DVec3, pseudo: bool) {
        if c.inv_mass_a > 0.0
            && let Some(ba) = self.bodies.get_mut(body_ah(c.body_a))
        {
            let dv = p * c.inv_mass_a;
            let dw = c.inv_i_a * (c.ra.cross(p) + l);
            if pseudo {
                ba.pseudo_velocity -= dv;
                ba.pseudo_angular_velocity -= dw;
            } else {
                ba.linear_velocity -= dv;
                ba.angular_velocity -= dw;
            }
        }
        if c.inv_mass_b > 0.0
            && let Some(bb) = self.bodies.get_mut(body_ah(c.body_b))
        {
            let dv = p * c.inv_mass_b;
            let dw = c.inv_i_b * (c.rb.cross(p) + l);
            if pseudo {
                bb.pseudo_velocity += dv;
                bb.pseudo_angular_velocity += dw;
            } else {
                bb.linear_velocity += dv;
                bb.angular_velocity += dw;
            }
        }
    }

    /// Relative (B − A) anchor velocity and angular velocity, real or pseudo.
    #[inline]
    #[must_use]
    fn joint_relative_velocity(&self, c: &JointConstraint, pseudo: bool) -> (DVec3, DVec3) {
        let (va, wa) = if pseudo {
            self.body_pseudo_velocity(c.body_a)
        } else {
            self.bodies
                .get(body_ah(c.body_a))
                .map_or((DVec3::ZERO, DVec3::ZERO), |b| {
                    (b.linear_velocity, b.angular_velocity)
                })
        };
        let (vb, wb) = if pseudo {
            self.body_pseudo_velocity(c.body_b)
        } else {
            self.bodies
                .get(body_ah(c.body_b))
                .map_or((DVec3::ZERO, DVec3::ZERO), |b| {
                    (b.linear_velocity, b.angular_velocity)
                })
        };
        (vb + wb.cross(c.rb) - va - wa.cross(c.ra), wb - wa)
    }

    /// One velocity iteration over the joint rows.
    pub(super) fn solve_joint_velocities(&mut self, constraints: &mut [JointConstraint], dt: f64) {
        for c in constraints.iter_mut() {
            if c.broken {
                continue;
            }
            let (v_rel, _) = self.joint_relative_velocity(c, false);
            let mut rows = std::mem::replace(&mut c.rows, JointRows::None);
            match &mut rows {
                JointRows::None => {}
                JointRows::Fixed {
                    point_mass,
                    angular,
                    impulse,
                    ..
                } => {
                    let p = self.solve_fixed_block(c, *point_mass, angular.as_ref(), None, false);
                    *impulse += p;
                }
                JointRows::Axial {
                    axis,
                    mass,
                    error,
                    rope,
                    impulse,
                    ..
                } => {
                    let cdot = v_rel.dot(*axis);
                    let lambda = if *rope {
                        // Slack rope: allow closing the gap this step, never push.
                        let bias = if *error < 0.0 && dt > 0.0 {
                            *error / dt
                        } else {
                            0.0
                        };
                        let new_impulse = (*impulse - (cdot + bias) * *mass).min(0.0);
                        let applied = new_impulse - *impulse;
                        *impulse = new_impulse;
                        applied
                    } else {
                        let l = -cdot * *mass;
                        *impulse += l;
                        l
                    };
                    if lambda != 0.0 {
                        self.apply_joint_impulse(c, *axis * lambda, DVec3::ZERO, false);
                    }
                }
                JointRows::Perpendicular {
                    axes,
                    masses,
                    impulse,
                    ..
                } => {
                    for i in 0..2 {
                        let (v_rel, _) = self.joint_relative_velocity(c, false);
                        let lambda = -v_rel.dot(axes[i]) * masses[i];
                        impulse[i] += lambda;
                        self.apply_joint_impulse(c, axes[i] * lambda, DVec3::ZERO, false);
                    }
                }
            }
            c.rows = rows;
        }
    }

    /// One split-impulse position iteration over the joint rows: drive the
    /// pseudo relative velocity to `−erp · error / dt`.
    pub(super) fn solve_joint_positions(
        &mut self,
        constraints: &mut [JointConstraint],
        erp: f64,
        dt: f64,
    ) {
        if dt <= 0.0 {
            return;
        }
        let rate = erp / dt;
        for c in constraints.iter_mut() {
            if c.broken {
                continue;
            }
            let mut rows = std::mem::replace(&mut c.rows, JointRows::None);
            match &mut rows {
                JointRows::None => {}
                JointRows::Fixed {
                    point_mass,
                    angular,
                    point_error,
                    angular_error,
                    ..
                } => {
                    let bias = (*point_error * rate, *angular_error * rate);
                    let _ =
                        self.solve_fixed_block(c, *point_mass, angular.as_ref(), Some(bias), true);
                }
                JointRows::Axial {
                    axis,
                    mass,
                    error,
                    rope,
                    position_impulse,
                    ..
                } => {
                    if !*rope || *error > 0.0 {
                        let (v_rel, _) = self.joint_relative_velocity(c, true);
                        let lambda = -(v_rel.dot(*axis) + *error * rate) * *mass;
                        let lambda = if *rope {
                            let new_impulse = (*position_impulse + lambda).min(0.0);
                            let applied = new_impulse - *position_impulse;
                            *position_impulse = new_impulse;
                            applied
                        } else {
                            lambda
                        };
                        if lambda != 0.0 {
                            self.apply_joint_impulse(c, *axis * lambda, DVec3::ZERO, true);
                        }
                    }
                }
                JointRows::Perpendicular {
                    axes,
                    masses,
                    errors,
                    ..
                } => {
                    for i in 0..2 {
                        let (v_rel, _) = self.joint_relative_velocity(c, true);
                        let lambda = -(v_rel.dot(axes[i]) + errors[i] * rate) * masses[i];
                        self.apply_joint_impulse(c, axes[i] * lambda, DVec3::ZERO, true);
                    }
                }
            }
            c.rows = rows;
        }
    }

    /// Remove joints whose reaction force this sub-step (constraint impulse
    /// / dt, or the explicit force) exceeds their `break_force`.
    pub(super) fn break_joints(&mut self, constraints: &mut [JointConstraint], dt: f64) {
        for c in constraints.iter_mut() {
            let Some(bf) = c.break_force else {
                continue;
            };
            let impulse = match &c.rows {
                JointRows::None => 0.0,
                JointRows::Fixed { impulse, .. } => impulse.length(),
                JointRows::Axial { impulse, .. } => impulse.abs(),
                JointRows::Perpendicular { impulse, .. } => impulse[0].hypot(impulse[1]),
            };
            let reaction = if dt > 0.0 { impulse / dt } else { 0.0 };
            if reaction.max(c.explicit_force) > bf {
                c.broken = true;
                self.joints.remove(c.handle);
            }
        }
    }

    /// Apply a joint's explicit forces (spring, wheel suspension, mouse, and
    /// per-joint damping) as impulses for one sub-step. Returns the force
    /// magnitude (for breaking).
    fn apply_joint_force(&mut self, joint: &Joint3d, dt: f64) -> f64 {
        let force = match &joint.joint_type {
            JointType::Spring {
                rest_length,
                stiffness,
                damping,
            } => self.apply_axial_spring(joint, None, *rest_length, *stiffness, *damping, dt),
            JointType::Wheel {
                axis,
                stiffness,
                damping,
            } => {
                let ax = DVec3::new(axis[0], axis[1], 0.0);
                let len = ax.length();
                if len < EPSILON || !len.is_finite() {
                    0.0
                } else {
                    self.apply_axial_spring(joint, Some(ax / len), 0.0, *stiffness, *damping, dt)
                }
            }
            JointType::Mouse {
                target,
                stiffness,
                damping,
                max_force,
            } => self.apply_mouse_force(joint, *target, *stiffness, *damping, *max_force, dt),
            // Revolute / Prismatic (and motors) need 3D axis definitions and
            // are not implemented in 3D; Fixed / Distance / Rope are rows.
            _ => 0.0,
        };
        // Per-joint damping (Spring has its own damping).
        if joint.damping > 0.0 && !matches!(joint.joint_type, JointType::Spring { .. }) {
            self.apply_joint_damping_3d(joint, dt);
        }
        force
    }

    /// World lever arms, inverse masses and inertias of a joint's bodies.
    #[inline]
    #[must_use]
    fn joint_bodies(&self, joint: &Joint3d) -> Option<JointBodies> {
        let ba = self.bodies.get(body_ah(joint.body_a))?;
        let bb = self.bodies.get(body_ah(joint.body_b))?;
        let (inv_mass_a, inv_i_a) = solver_mass(ba);
        let (inv_mass_b, inv_i_b) = solver_mass(bb);
        let ra = ba.rotation * joint.local_anchor_a;
        let rb = bb.rotation * joint.local_anchor_b;
        Some(JointBodies {
            diff: (bb.position + rb) - (ba.position + ra),
            c: JointConstraint {
                handle: ArenaHandle(0),
                body_a: joint.body_a,
                body_b: joint.body_b,
                inv_mass_a,
                inv_mass_b,
                inv_i_a,
                inv_i_b,
                ra,
                rb,
                rows: JointRows::None,
                explicit_force: 0.0,
                break_force: None,
                broken: false,
            },
        })
    }

    /// Spring-damper between the anchors, along the anchor separation (or a
    /// fixed `axis` with zero rest length for the wheel suspension).
    fn apply_axial_spring(
        &mut self,
        joint: &Joint3d,
        axis: Option<DVec3>,
        rest_length: f64,
        stiffness: f64,
        damping: f64,
        dt: f64,
    ) -> f64 {
        let Some(JointBodies { diff, c }) = self.joint_bodies(joint) else {
            return 0.0;
        };
        let (n, stretch) = match axis {
            Some(ax) => (ax, diff.dot(ax)),
            None => {
                let dist = diff.length();
                if dist < EPSILON {
                    return 0.0;
                }
                (diff / dist, dist - rest_length)
            }
        };
        let (v_rel, _) = self.joint_relative_velocity(&c, false);
        let total_force = stiffness * stretch + damping * v_rel.dot(n);
        if !total_force.is_finite() {
            return 0.0;
        }
        // Stretched (F > 0): pull B toward A and A toward B.
        self.apply_joint_impulse(&c, n * (-total_force * dt), DVec3::ZERO, false);
        total_force.abs()
    }

    /// Mouse joint — drags body_a's anchor toward a world-space target.
    fn apply_mouse_force(
        &mut self,
        joint: &Joint3d,
        target: [f64; 3],
        stiffness: f64,
        damping: f64,
        max_force: f64,
        dt: f64,
    ) -> f64 {
        let Some(ba) = self.bodies.get(body_ah(joint.body_a)) else {
            return 0.0;
        };
        let (inv_mass, inv_i) = solver_mass(ba);
        let ra = ba.rotation * joint.local_anchor_a;
        let anchor = ba.position + ra;
        let v_anchor = ba.linear_velocity + ba.angular_velocity.cross(ra);
        let mut force = (DVec3::from_array(target) - anchor) * stiffness - v_anchor * damping;
        let max_force = if max_force.is_nan() {
            0.0
        } else {
            max_force.max(0.0)
        };
        let mag = force.length();
        if !mag.is_finite() {
            return 0.0;
        }
        if mag > max_force {
            force *= if mag > 0.0 { max_force / mag } else { 0.0 };
        }
        let impulse = force * dt;
        if inv_mass > 0.0
            && let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
        {
            ba.linear_velocity += impulse * inv_mass;
            ba.angular_velocity += inv_i * ra.cross(impulse);
        }
        force.length()
    }

    /// Damping proportional to the relative anchor velocity, limited so it
    /// never reverses that velocity.
    fn apply_joint_damping_3d(&mut self, joint: &Joint3d, dt: f64) {
        let Some(JointBodies { c, .. }) = self.joint_bodies(joint) else {
            return;
        };
        let (v_rel, _) = self.joint_relative_velocity(&c, false);
        let speed = v_rel.length();
        if speed < EPSILON {
            return;
        }
        let dir = v_rel / speed;
        let mass = axial_mass(
            dir,
            c.ra,
            c.rb,
            c.inv_mass_a,
            c.inv_mass_b,
            c.inv_i_a,
            c.inv_i_b,
        );
        if mass == 0.0 {
            return;
        }
        let magnitude = (joint.damping * speed * dt).min(speed * mass);
        self.apply_joint_impulse(&c, dir * -magnitude, DVec3::ZERO, false);
    }
}

/// Anchor separation and solver data for a joint's two bodies.
struct JointBodies {
    diff: DVec3,
    c: JointConstraint,
}
