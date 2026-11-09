//! Internal types for the 2D physics backend.

use crate::body::{BodyHandle, BodyType};
use crate::collider::{ColliderDesc, ColliderHandle, ColliderShape};
use crate::material::PhysicsMaterial;

// ---------------------------------------------------------------------------
// Named constants
// ---------------------------------------------------------------------------

/// General-purpose geometry epsilon for zero-length checks (normals, axes, etc.).
pub(super) const EPSILON: f64 = 1e-10;
/// Squared epsilon for distance-squared checks (avoids sqrt for near-zero vectors).
pub(super) const EPSILON_SQ: f64 = 1e-20;
/// Bodies with both linear and angular speed below this are candidates for sleep.
pub(super) const SLEEP_VELOCITY_THRESHOLD: f64 = 0.01;
/// How many seconds of low motion before a body is put to sleep.
pub(super) const SLEEP_TIME_THRESHOLD: f64 = 0.5;
/// Minimum mass/inertia to avoid division by zero for dynamic bodies.
pub(super) const MIN_MASS: f64 = 1e-6;
/// Minimum inertia to avoid division by zero.
pub(super) const MIN_INERTIA: f64 = 1e-10;
/// Distance threshold for matching manifold points across frames (body-local coords).
pub(super) const MANIFOLD_MATCH_THRESHOLD: f64 = 0.02;
/// Maximum number of contact points per manifold in 2D.
/// Increased from 2 to 4 for one-shot manifold generation (edge clipping).
pub(super) const MAX_MANIFOLD_POINTS: usize = 4;
/// Warm starting scale factor — slightly less than 1.0 for stability.
pub(super) const WARM_START_FACTOR: f64 = 0.95;

// ---------------------------------------------------------------------------
// Internal body representation
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub(crate) struct RigidBody2d {
    pub handle: BodyHandle,
    pub body_type: BodyType,
    pub position: [f64; 2],
    pub rotation: f64,
    pub linear_velocity: [f64; 2],
    pub angular_velocity: f64,
    pub linear_damping: f64,
    pub angular_damping: f64,
    pub fixed_rotation: bool,
    pub gravity_scale: f64,
    pub force_accumulator: [f64; 2],
    pub torque_accumulator: f64,
    // Mass properties (accumulated from attached colliders)
    pub mass: f64,
    pub inv_mass: f64,
    pub inertia: f64,
    pub inv_inertia: f64,
    // Split impulse — pseudo-velocities for position correction only
    pub pseudo_velocity: [f64; 2],
    pub pseudo_angular_velocity: f64,
    // XPBD: previous-frame positions for velocity derivation
    pub prev_position: [f64; 2],
    pub prev_rotation: f64,
    // Sleep state
    pub is_sleeping: bool,
    pub sleep_timer: f64,
    // Island id assigned during island building (used for island-based sleep).
    pub island_id: u32,
}

impl RigidBody2d {
    pub(super) fn from_desc(handle: BodyHandle, desc: &crate::body::BodyDesc) -> Self {
        Self {
            handle,
            body_type: desc.body_type,
            position: [desc.position[0], desc.position[1]],
            rotation: desc.rotation,
            linear_velocity: [desc.linear_velocity[0], desc.linear_velocity[1]],
            angular_velocity: desc.angular_velocity,
            linear_damping: desc.linear_damping,
            angular_damping: desc.angular_damping,
            fixed_rotation: desc.fixed_rotation,
            gravity_scale: desc.gravity_scale.unwrap_or(1.0),
            force_accumulator: [0.0, 0.0],
            torque_accumulator: 0.0,
            mass: 0.0,
            inv_mass: 0.0,
            inertia: 0.0,
            inv_inertia: 0.0,
            pseudo_velocity: [0.0, 0.0],
            pseudo_angular_velocity: 0.0,
            prev_position: [desc.position[0], desc.position[1]],
            prev_rotation: desc.rotation,
            is_sleeping: false,
            sleep_timer: 0.0,
            island_id: 0,
        }
    }

    pub(super) fn is_dynamic(&self) -> bool {
        self.body_type == BodyType::Dynamic
    }

    pub(super) fn is_static(&self) -> bool {
        self.body_type == BodyType::Static
    }

    /// Derive `inv_mass` / `inv_inertia` from `mass` / `inertia`: only
    /// dynamic bodies with finite positive mass get non-zero inverses.
    pub(super) fn refresh_inverse_mass(&mut self) {
        if self.is_dynamic() && self.mass > 0.0 && self.mass.is_finite() {
            self.inv_mass = 1.0 / self.mass;
            self.inv_inertia = if self.fixed_rotation
                || self.inertia.is_nan()
                || self.inertia <= 0.0
                || !self.inertia.is_finite()
            {
                0.0
            } else {
                1.0 / self.inertia
            };
        } else {
            self.inv_mass = 0.0;
            self.inv_inertia = 0.0;
        }
    }

    /// Inverse mass and inverse inertia as seen by the constraint solver:
    /// zero (infinite mass) for static, kinematic and sleeping bodies.
    #[inline]
    #[must_use]
    pub(super) fn solver_inv_mass(&self) -> (f64, f64) {
        if self.is_dynamic() && !self.is_sleeping {
            (self.inv_mass, self.inv_inertia)
        } else {
            (0.0, 0.0)
        }
    }

    pub(super) fn integrate_velocities(&mut self, gravity: [f64; 2], dt: f64, max_velocity: f64) {
        if !self.is_dynamic() || self.inv_mass == 0.0 {
            return;
        }
        // Sleeping bodies skip integration
        if self.is_sleeping {
            return;
        }

        // Apply gravity
        self.linear_velocity[0] += gravity[0] * self.gravity_scale * dt;
        self.linear_velocity[1] += gravity[1] * self.gravity_scale * dt;

        // Apply accumulated forces: a = F * inv_mass
        self.linear_velocity[0] += self.force_accumulator[0] * self.inv_mass * dt;
        self.linear_velocity[1] += self.force_accumulator[1] * self.inv_mass * dt;

        // Apply accumulated torque
        if !self.fixed_rotation {
            self.angular_velocity += self.torque_accumulator * self.inv_inertia * dt;
        }

        // Apply damping
        self.linear_velocity[0] *= 1.0 / (1.0 + dt * self.linear_damping);
        self.linear_velocity[1] *= 1.0 / (1.0 + dt * self.linear_damping);
        self.angular_velocity *= 1.0 / (1.0 + dt * self.angular_damping);

        // CCD: clamp velocity magnitude to prevent tunneling
        let speed_sq = self.linear_velocity[0].powi(2) + self.linear_velocity[1].powi(2);
        if speed_sq > max_velocity * max_velocity {
            let scale = max_velocity / speed_sq.sqrt();
            self.linear_velocity[0] *= scale;
            self.linear_velocity[1] *= scale;
        }
    }

    pub(super) fn integrate_positions(&mut self, dt: f64) {
        // Static bodies never move
        if self.is_static() {
            return;
        }
        // Dynamic bodies need mass and must be awake; kinematic bodies move
        // from user-set velocity
        if self.is_dynamic() && (self.inv_mass == 0.0 || self.is_sleeping) {
            self.pseudo_velocity = [0.0, 0.0];
            self.pseudo_angular_velocity = 0.0;
            return;
        }

        // Real velocity integration
        self.position[0] += self.linear_velocity[0] * dt;
        self.position[1] += self.linear_velocity[1] * dt;

        // Split impulse: apply pseudo-velocities for position correction
        self.position[0] += self.pseudo_velocity[0] * dt;
        self.position[1] += self.pseudo_velocity[1] * dt;

        if !self.fixed_rotation {
            self.rotation += self.angular_velocity * dt;
            self.rotation += self.pseudo_angular_velocity * dt;
        }

        // Clear pseudo-velocities after integration
        self.pseudo_velocity = [0.0, 0.0];
        self.pseudo_angular_velocity = 0.0;
    }

    pub(super) fn clear_forces(&mut self) {
        self.force_accumulator = [0.0, 0.0];
        self.torque_accumulator = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Internal collider representation
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub(crate) struct Collider2d {
    pub handle: ColliderHandle,
    pub body: BodyHandle,
    pub shape: ColliderShape,
    pub offset: [f64; 2],
    pub material: PhysicsMaterial,
    pub is_sensor: bool,
    pub mass: Option<f64>,
    pub collision_layer: u32,
    pub collision_mask: u32,
}

impl Collider2d {
    pub(super) fn from_desc(handle: ColliderHandle, body: BodyHandle, desc: &ColliderDesc) -> Self {
        Self {
            handle,
            body,
            shape: desc.shape.clone(),
            offset: [desc.offset[0], desc.offset[1]],
            material: desc.material.clone(),
            is_sensor: desc.is_sensor,
            mass: desc.mass,
            collision_layer: desc.collision_layer,
            collision_mask: desc.collision_mask,
        }
    }

    /// Compute AABB in world space given body position and rotation.
    pub(crate) fn world_aabb(&self, body_pos: [f64; 2], body_rot: f64) -> Aabb2d {
        let (sin, cos) = body_rot.sin_cos();
        let wx = body_pos[0] + cos * self.offset[0] - sin * self.offset[1];
        let wy = body_pos[1] + sin * self.offset[0] + cos * self.offset[1];

        match &self.shape {
            ColliderShape::Ball { radius } => Aabb2d {
                min: [wx - radius, wy - radius],
                max: [wx + radius, wy + radius],
            },
            ColliderShape::Box { half_extents } => {
                let hx = half_extents[0];
                let hy = half_extents[1];
                let ex = (cos * hx).abs() + (sin * hy).abs();
                let ey = (sin * hx).abs() + (cos * hy).abs();
                Aabb2d {
                    min: [wx - ex, wy - ey],
                    max: [wx + ex, wy + ey],
                }
            }
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                let ex = (sin * half_height).abs() + radius;
                let ey = (cos * half_height).abs() + radius;
                Aabb2d {
                    min: [wx - ex, wy - ey],
                    max: [wx + ex, wy + ey],
                }
            }
            ColliderShape::Segment { a, b } => {
                let ax = cos * a[0] - sin * a[1] + wx;
                let ay = sin * a[0] + cos * a[1] + wy;
                let bx = cos * b[0] - sin * b[1] + wx;
                let by = sin * b[0] + cos * b[1] + wy;
                Aabb2d {
                    min: [ax.min(bx), ay.min(by)],
                    max: [ax.max(bx), ay.max(by)],
                }
            }
            ColliderShape::ConvexHull { points } => {
                let mut min_p = [f64::INFINITY, f64::INFINITY];
                let mut max_p = [f64::NEG_INFINITY, f64::NEG_INFINITY];
                for p in points {
                    let px = cos * p[0] - sin * p[1] + wx;
                    let py = sin * p[0] + cos * p[1] + wy;
                    min_p[0] = min_p[0].min(px);
                    min_p[1] = min_p[1].min(py);
                    max_p[0] = max_p[0].max(px);
                    max_p[1] = max_p[1].max(py);
                }
                Aabb2d {
                    min: min_p,
                    max: max_p,
                }
            }
            ColliderShape::Heightfield { heights, scale } => {
                let w = scale[0] * (heights.len().max(1) - 1) as f64;
                let h_min = heights.iter().copied().fold(f64::INFINITY, f64::min);
                let h_max = heights.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                Aabb2d {
                    min: [wx, wy + h_min * scale[1]],
                    max: [wx + w, wy + h_max * scale[1]],
                }
            }
            ColliderShape::TriMesh { vertices, .. } => {
                let mut min_p = [f64::INFINITY, f64::INFINITY];
                let mut max_p = [f64::NEG_INFINITY, f64::NEG_INFINITY];
                for v in vertices {
                    let px = cos * v[0] - sin * v[1] + wx;
                    let py = sin * v[0] + cos * v[1] + wy;
                    min_p[0] = min_p[0].min(px);
                    min_p[1] = min_p[1].min(py);
                    max_p[0] = max_p[0].max(px);
                    max_p[1] = max_p[1].max(py);
                }
                Aabb2d {
                    min: min_p,
                    max: max_p,
                }
            }
        }
    }

    /// Area, local centroid (collider frame) and polar moment of inertia per
    /// unit mass about that centroid.
    fn shape_props(&self) -> (f64, [f64; 2], f64) {
        const FALLBACK: (f64, [f64; 2], f64) = (1.0, [0.0, 0.0], 1.0);
        match &self.shape {
            ColliderShape::Ball { radius } => (
                std::f64::consts::PI * radius * radius,
                [0.0, 0.0],
                0.5 * radius * radius,
            ),
            ColliderShape::Box { half_extents } => {
                let w = 2.0 * half_extents[0];
                let h = 2.0 * half_extents[1];
                ((w * h).abs(), [0.0, 0.0], (w * w + h * h) / 12.0)
            }
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                // Rectangle plus two half-discs (Box2D `b2ComputeCapsuleMass`).
                let (h, r) = (half_height.abs(), radius.abs());
                let rect_area = 4.0 * h * r;
                let circle_area = std::f64::consts::PI * r * r;
                let area = rect_area + circle_area;
                if area <= 0.0 {
                    return FALLBACK;
                }
                let lc = 4.0 * r / (3.0 * std::f64::consts::PI);
                let rect_i = rect_area * (4.0 * r * r + 4.0 * h * h) / 12.0;
                let circle_i = circle_area * (0.5 * r * r + h * h + 2.0 * h * lc);
                (area, [0.0, 0.0], (rect_i + circle_i) / area)
            }
            ColliderShape::Segment { a, b } => {
                // Thin rod, 1 cm thick.
                let dx = b[0] - a[0];
                let dy = b[1] - a[1];
                let len = (dx * dx + dy * dy).sqrt();
                (
                    len * 0.01,
                    [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5],
                    len * len / 12.0,
                )
            }
            ColliderShape::ConvexHull { points } => {
                let pts: Vec<[f64; 2]> = points.iter().map(|p| [p[0], p[1]]).collect();
                polygon_mass_props(&pts).unwrap_or(FALLBACK)
            }
            ColliderShape::TriMesh { vertices, indices } => {
                let mut area = 0.0;
                let mut c = [0.0, 0.0];
                let mut second = 0.0; // Σ aᵢ (iᵢ + |cᵢ|²)
                for tri in indices {
                    let v = |i: u32| vertices.get(i as usize).map(|p| [p[0], p[1]]);
                    let (Some(p0), Some(p1), Some(p2)) = (v(tri[0]), v(tri[1]), v(tri[2])) else {
                        continue;
                    };
                    if let Some((ai, ci, ii)) = polygon_mass_props(&[p0, p1, p2]) {
                        area += ai;
                        c[0] += ai * ci[0];
                        c[1] += ai * ci[1];
                        second += ai * (ii + ci[0] * ci[0] + ci[1] * ci[1]);
                    }
                }
                if area <= 0.0 {
                    return FALLBACK;
                }
                let c = [c[0] / area, c[1] / area];
                (
                    area,
                    c,
                    (second / area - c[0] * c[0] - c[1] * c[1]).max(0.0),
                )
            }
            ColliderShape::Heightfield { .. } => FALLBACK,
        }
    }

    /// Collider centroid in the collider's local frame.
    #[cfg(test)]
    #[must_use]
    pub(super) fn local_centroid(&self) -> [f64; 2] {
        self.shape_props().1
    }

    /// `(mass, inertia about the centroid, local centroid)` in one pass.
    #[must_use]
    pub(super) fn mass_properties(&self) -> (f64, f64, [f64; 2]) {
        let (area, centroid, i_per_mass) = self.shape_props();
        let mass = match self.mass {
            Some(m) => m.max(MIN_MASS),
            None => (area * self.material.density).max(MIN_MASS),
        };
        (mass, (mass * i_per_mass).max(MIN_INERTIA), centroid)
    }

    /// Compute mass from shape and material (or the explicit `mass`). Always
    /// at least `MIN_MASS`, so dynamic bodies never divide by zero.
    #[cfg(test)]
    #[must_use]
    pub(super) fn compute_mass(&self) -> f64 {
        if let Some(m) = self.mass {
            return m.max(MIN_MASS);
        }
        (self.shape_props().0 * self.material.density).max(MIN_MASS)
    }

    /// Moment of inertia about the collider's centroid for a given mass.
    #[cfg(test)]
    #[must_use]
    pub(super) fn compute_inertia(&self, mass: f64) -> f64 {
        (mass * self.shape_props().2).max(MIN_INERTIA)
    }
}

// ---------------------------------------------------------------------------
// Internal joint representation
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub(crate) struct Joint2d {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub joint_type: crate::joint::JointType,
    pub local_anchor_a: [f64; 2],
    pub local_anchor_b: [f64; 2],
    pub motor: Option<crate::joint::JointMotor>,
    pub damping: f64,
    pub break_force: Option<f64>,
    /// Relative rotation `rot_b − rot_a` when the joint was created; Fixed and
    /// Prismatic joints hold it.
    pub reference_angle: f64,
}

// ---------------------------------------------------------------------------
// AABB for broadphase
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub(crate) struct Aabb2d {
    pub min: [f64; 2],
    pub max: [f64; 2],
}

impl Aabb2d {
    pub(super) fn overlaps(&self, other: &Aabb2d) -> bool {
        self.min[0] <= other.max[0]
            && self.max[0] >= other.min[0]
            && self.min[1] <= other.max[1]
            && self.max[1] >= other.min[1]
    }
}

// ---------------------------------------------------------------------------
// Contact for narrowphase
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub(crate) struct Contact {
    pub collider_a: ColliderHandle,
    pub collider_b: ColliderHandle,
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub normal: [f64; 2],
    pub depth: f64,
    pub point: [f64; 2],
}

// ---------------------------------------------------------------------------
// Persistent contact manifold types
// ---------------------------------------------------------------------------

/// A cached contact point with accumulated impulses for warm starting.
#[derive(Debug, Clone)]
pub(super) struct ManifoldPoint {
    /// Contact point in body A's local space.
    pub local_a: [f64; 2],
    /// Contact point in body B's local space.
    pub local_b: [f64; 2],
    /// Accumulated normal impulse (for warm starting).
    pub normal_impulse: f64,
    /// Accumulated tangent impulse (for warm starting).
    pub tangent_impulse: f64,
    /// Penetration depth.
    pub depth: f64,
}

/// A contact manifold between two colliders, persisted across frames.
#[derive(Debug, Clone)]
pub(super) struct ContactManifold {
    pub collider_a: ColliderHandle,
    pub collider_b: ColliderHandle,
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub normal: [f64; 2],
    pub points: Vec<ManifoldPoint>, // Up to MAX_MANIFOLD_POINTS (4 in 2D)
    /// Accumulated rolling-resistance angular impulse (for warm starting).
    pub rolling_impulse: f64,
}

/// Key for looking up manifolds between collider pairs.
pub(super) type ManifoldKey = (ColliderHandle, ColliderHandle);

// ---------------------------------------------------------------------------
// Simulation island manager (union-find)
// ---------------------------------------------------------------------------

/// Union-find structure for simulation islands.
pub(crate) struct IslandManager {
    parent: Vec<u32>,
    rank: Vec<u32>,
}

impl IslandManager {
    /// Create a new island manager with given capacity.
    pub fn new(capacity: usize) -> Self {
        let mut parent = Vec::with_capacity(capacity);
        let mut rank = Vec::with_capacity(capacity);
        for i in 0..capacity {
            parent.push(i as u32);
            rank.push(0);
        }
        Self { parent, rank }
    }

    /// Reset for a new frame with `count` elements.
    pub fn reset(&mut self, count: usize) {
        self.parent.clear();
        self.rank.clear();
        for i in 0..count {
            self.parent.push(i as u32);
            self.rank.push(0);
        }
    }

    /// Path-compressed find.
    pub fn find(&mut self, mut x: u32) -> u32 {
        while self.parent[x as usize] != x {
            self.parent[x as usize] = self.parent[self.parent[x as usize] as usize];
            x = self.parent[x as usize];
        }
        x
    }

    /// Union by rank.
    pub fn union(&mut self, a: u32, b: u32) {
        let ra = self.find(a);
        let rb = self.find(b);
        if ra == rb {
            return;
        }
        if self.rank[ra as usize] < self.rank[rb as usize] {
            self.parent[ra as usize] = rb;
        } else if self.rank[ra as usize] > self.rank[rb as usize] {
            self.parent[rb as usize] = ra;
        } else {
            self.parent[rb as usize] = ra;
            self.rank[ra as usize] += 1;
        }
    }
}

/// Area, centroid and polar moment of inertia per unit mass (about the
/// centroid) of a simple polygon, either winding (Box2D
/// `b2ComputePolygonMass`). `None` for fewer than three points or zero area.
fn polygon_mass_props(pts: &[[f64; 2]]) -> Option<(f64, [f64; 2], f64)> {
    if pts.len() < 3 {
        return None;
    }
    let o = pts[0];
    let (mut area, mut cx, mut cy, mut i_o) = (0.0, 0.0, 0.0, 0.0);
    for w in 1..pts.len() - 1 {
        let e1 = [pts[w][0] - o[0], pts[w][1] - o[1]];
        let e2 = [pts[w + 1][0] - o[0], pts[w + 1][1] - o[1]];
        let d = e1[0] * e2[1] - e1[1] * e2[0];
        let tri = 0.5 * d;
        area += tri;
        cx += tri * (e1[0] + e2[0]) / 3.0;
        cy += tri * (e1[1] + e2[1]) / 3.0;
        let intx2 = e1[0] * e1[0] + e2[0] * e1[0] + e2[0] * e2[0];
        let inty2 = e1[1] * e1[1] + e2[1] * e1[1] + e2[1] * e2[1];
        i_o += (0.25 / 3.0 * d) * (intx2 + inty2);
    }
    if area.abs() < EPSILON_SQ || !area.is_finite() {
        return None;
    }
    let c = [cx / area, cy / area];
    let i_c = i_o / area - (c[0] * c[0] + c[1] * c[1]);
    Some((area.abs(), [o[0] + c[0], o[1] + c[1]], i_c.max(0.0)))
}

#[cfg(test)]
mod mass_tests {
    use super::*;

    fn collider(shape: ColliderShape, offset: [f64; 2]) -> Collider2d {
        Collider2d::from_desc(
            ColliderHandle(0),
            BodyHandle(0),
            &ColliderDesc {
                shape,
                offset: [offset[0], offset[1], 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: u32::MAX,
                collision_mask: u32::MAX,
            },
        )
    }

    #[test]
    fn hull_square_matches_box() {
        for pts in [
            vec![
                [0.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [2.0, 2.0, 0.0],
                [0.0, 2.0, 0.0],
            ],
            vec![
                [0.0, 2.0, 0.0],
                [2.0, 2.0, 0.0],
                [2.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ],
        ] {
            let hull = collider(ColliderShape::ConvexHull { points: pts }, [0.0, 0.0]);
            let bx = collider(
                ColliderShape::Box {
                    half_extents: [1.0, 1.0, 0.0],
                },
                [0.0, 0.0],
            );
            assert!((hull.compute_mass() - bx.compute_mass()).abs() < 1e-12);
            let m = hull.compute_mass();
            assert!((hull.compute_inertia(m) - bx.compute_inertia(m)).abs() < 1e-12);
            let c = hull.local_centroid();
            assert!((c[0] - 1.0).abs() < 1e-12 && (c[1] - 1.0).abs() < 1e-12);
        }
    }

    #[test]
    fn capsule_inertia_includes_cap_offset() {
        let cap = collider(
            ColliderShape::Capsule {
                half_height: 1.0,
                radius: 0.5,
            },
            [0.0, 0.0],
        );
        let m = cap.compute_mass();
        let r: f64 = 0.5;
        let h: f64 = 1.0;
        let rect = 4.0 * h * r;
        let circ = std::f64::consts::PI * r * r;
        let lc = 4.0 * r / (3.0 * std::f64::consts::PI);
        let expected = m
            * (rect * (4.0 * r * r + 4.0 * h * h) / 12.0
                + circ * (0.5 * r * r + h * h + 2.0 * h * lc))
            / (rect + circ);
        assert!((cap.compute_inertia(m) - expected).abs() < 1e-12);
    }
}
