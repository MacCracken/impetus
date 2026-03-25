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
pub(super) const MAX_MANIFOLD_POINTS: usize = 2;
/// Separation tolerance for re-validating old manifold points.
pub(super) const MANIFOLD_REVALIDATION_TOLERANCE: f64 = 0.02;
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
        // Dynamic bodies need mass; kinematic bodies move from user-set velocity
        if self.is_dynamic() && self.inv_mass == 0.0 {
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

    /// Compute mass from shape and material. Returns at least a small positive value
    /// for dynamic bodies to avoid division by zero.
    pub(super) fn compute_mass(&self) -> f64 {
        if let Some(m) = self.mass {
            return m.max(MIN_MASS);
        }
        let area = match &self.shape {
            ColliderShape::Ball { radius } => std::f64::consts::PI * radius * radius,
            ColliderShape::Box { half_extents } => 4.0 * half_extents[0] * half_extents[1],
            ColliderShape::Capsule {
                half_height,
                radius,
            } => 2.0 * half_height * 2.0 * radius + std::f64::consts::PI * radius * radius,
            ColliderShape::Segment { a, b } => {
                // Treat as thin rod with small thickness
                let dx = b[0] - a[0];
                let dy = b[1] - a[1];
                let len = (dx * dx + dy * dy).sqrt();
                len * 0.01 // 1cm thick
            }
            _ => 1.0,
        };
        (area * self.material.density).max(MIN_MASS)
    }

    /// Compute moment of inertia about center of mass.
    pub(super) fn compute_inertia(&self, mass: f64) -> f64 {
        let i = match &self.shape {
            ColliderShape::Ball { radius } => 0.5 * mass * radius * radius,
            ColliderShape::Box { half_extents } => {
                let w = 2.0 * half_extents[0];
                let h = 2.0 * half_extents[1];
                mass * (w * w + h * h) / 12.0
            }
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                let rect_area = 2.0 * half_height * 2.0 * radius;
                let circle_area = std::f64::consts::PI * radius * radius;
                let total_area = rect_area + circle_area;
                let rect_mass = mass * rect_area / total_area;
                let w = 2.0 * radius;
                let h = 2.0 * half_height;
                rect_mass * (w * w + h * h) / 12.0 + (mass - rect_mass) * 0.5 * radius * radius
            }
            ColliderShape::Segment { a, b } => {
                let dx = b[0] - a[0];
                let dy = b[1] - a[1];
                let len = (dx * dx + dy * dy).sqrt();
                mass * len * len / 12.0 // thin rod
            }
            _ => mass,
        };
        i.max(MIN_INERTIA)
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
    pub points: Vec<ManifoldPoint>, // Up to MAX_MANIFOLD_POINTS (2 in 2D)
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
