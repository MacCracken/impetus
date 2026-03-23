//! Native 2D physics backend.
//!
//! Implements broadphase (AABB overlap), narrowphase (shape-vs-shape contact
//! generation), and a sequential impulse constraint solver with friction and
//! angular response. All geometry uses f64 precision.

use std::collections::{BTreeMap, BTreeSet};

use crate::arena::{Arena, ArenaHandle};
use crate::body::{BodyDesc, BodyHandle, BodyState, BodyType};
use crate::spatial_hash::SpatialHashGrid;
use crate::collider::{ColliderDesc, ColliderHandle, ColliderShape};
use crate::event::CollisionEvent;
use crate::force::{Force, Impulse, Torque};
use crate::joint::{JointDesc, JointHandle, JointMotor, JointType};
use crate::material::PhysicsMaterial;
use crate::query::RayHit;
use crate::ImpetusError;

// ---------------------------------------------------------------------------
// Handle ↔ ArenaHandle conversions (zero-cost — same u64 layout)
// ---------------------------------------------------------------------------

#[inline(always)]
fn body_ah(h: BodyHandle) -> ArenaHandle { ArenaHandle(h.0) }
#[inline(always)]
fn body_from(ah: ArenaHandle) -> BodyHandle { BodyHandle(ah.0) }
#[inline(always)]
fn coll_ah(h: ColliderHandle) -> ArenaHandle { ArenaHandle(h.0) }
#[inline(always)]
fn coll_from(ah: ArenaHandle) -> ColliderHandle { ColliderHandle(ah.0) }
#[inline(always)]
fn joint_ah(h: JointHandle) -> ArenaHandle { ArenaHandle(h.0) }
#[inline(always)]
fn joint_from(ah: ArenaHandle) -> JointHandle { JointHandle(ah.0) }

// ---------------------------------------------------------------------------
// Named constants
// ---------------------------------------------------------------------------

/// General-purpose geometry epsilon for zero-length checks (normals, axes, etc.).
const EPSILON: f64 = 1e-10;
/// Squared epsilon for distance-squared checks (avoids sqrt for near-zero vectors).
const EPSILON_SQ: f64 = 1e-20;
/// Bodies with both linear and angular speed below this are candidates for sleep.
const SLEEP_VELOCITY_THRESHOLD: f64 = 0.01;
/// How many seconds of low motion before a body is put to sleep.
const SLEEP_TIME_THRESHOLD: f64 = 0.5;
/// Minimum mass/inertia to avoid division by zero for dynamic bodies.
const MIN_MASS: f64 = 1e-6;
/// Minimum inertia to avoid division by zero.
const MIN_INERTIA: f64 = 1e-10;
/// Distance threshold for matching manifold points across frames (body-local coords).
const MANIFOLD_MATCH_THRESHOLD: f64 = 0.02;
/// Warm starting scale factor — slightly less than 1.0 for stability.
const WARM_START_FACTOR: f64 = 0.95;

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
    // Sleep state
    pub is_sleeping: bool,
    pub sleep_timer: f64,
}

impl RigidBody2d {
    fn from_desc(handle: BodyHandle, desc: &BodyDesc) -> Self {
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
            is_sleeping: false,
            sleep_timer: 0.0,
        }
    }

    fn is_dynamic(&self) -> bool {
        self.body_type == BodyType::Dynamic
    }

    fn is_static(&self) -> bool {
        self.body_type == BodyType::Static
    }

    fn integrate_velocities(&mut self, gravity: [f64; 2], dt: f64, max_velocity: f64) {
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
        let speed_sq =
            self.linear_velocity[0].powi(2) + self.linear_velocity[1].powi(2);
        if speed_sq > max_velocity * max_velocity {
            let scale = max_velocity / speed_sq.sqrt();
            self.linear_velocity[0] *= scale;
            self.linear_velocity[1] *= scale;
        }
    }

    fn integrate_positions(&mut self, dt: f64) {
        // Static bodies never move
        if self.is_static() {
            return;
        }
        // Dynamic bodies need mass; kinematic bodies move from user-set velocity
        if self.is_dynamic() && self.inv_mass == 0.0 {
            return;
        }

        self.position[0] += self.linear_velocity[0] * dt;
        self.position[1] += self.linear_velocity[1] * dt;

        if !self.fixed_rotation {
            self.rotation += self.angular_velocity * dt;
        }
    }

    fn clear_forces(&mut self) {
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
    fn from_desc(handle: ColliderHandle, body: BodyHandle, desc: &ColliderDesc) -> Self {
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
    fn compute_mass(&self) -> f64 {
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
    fn compute_inertia(&self, mass: f64) -> f64 {
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
    pub joint_type: JointType,
    pub local_anchor_a: [f64; 2],
    pub local_anchor_b: [f64; 2],
    pub motor: Option<JointMotor>,
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
    fn overlaps(&self, other: &Aabb2d) -> bool {
        self.min[0] <= other.max[0]
            && self.max[0] >= other.min[0]
            && self.min[1] <= other.max[1]
            && self.max[1] >= other.min[1]
    }
}

// ---------------------------------------------------------------------------
// Spatial hash broadphase — uses shared SpatialHashGrid from spatial_hash.rs
// ---------------------------------------------------------------------------

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
struct ManifoldPoint {
    /// Contact point in body A's local space.
    local_a: [f64; 2],
    /// Contact point in body B's local space.
    local_b: [f64; 2],
    /// Accumulated normal impulse (for warm starting).
    normal_impulse: f64,
    /// Accumulated tangent impulse (for warm starting).
    tangent_impulse: f64,
    /// Penetration depth.
    depth: f64,
}

/// A contact manifold between two colliders, persisted across frames.
#[derive(Debug, Clone)]
struct ContactManifold {
    collider_a: ColliderHandle,
    collider_b: ColliderHandle,
    body_a: BodyHandle,
    body_b: BodyHandle,
    normal: [f64; 2],
    points: Vec<ManifoldPoint>, // Up to 1 point (single-point manifolds for now)
}

/// Key for looking up manifolds between collider pairs.
type ManifoldKey = (ColliderHandle, ColliderHandle);

// ---------------------------------------------------------------------------
// Physics state
// ---------------------------------------------------------------------------

pub(crate) struct PhysicsState2d {
    pub bodies: Arena<RigidBody2d>,
    pub colliders: Arena<Collider2d>,
    pub joints: Arena<Joint2d>,
    pub body_colliders: BTreeMap<BodyHandle, Vec<ColliderHandle>>,
    /// Persistent contact manifolds keyed by ordered collider pair.
    manifolds: BTreeMap<ManifoldKey, ContactManifold>,
    /// Previous frame's manifold keys for collision event generation.
    prev_manifold_keys: BTreeSet<ManifoldKey>,
}

impl PhysicsState2d {
    pub fn new() -> Self {
        Self {
            bodies: Arena::new(),
            colliders: Arena::new(),
            joints: Arena::new(),
            body_colliders: BTreeMap::new(),
            manifolds: BTreeMap::new(),
            prev_manifold_keys: BTreeSet::new(),
        }
    }

    pub fn add_body(&mut self, desc: &BodyDesc) -> BodyHandle {
        // Insert with a placeholder handle; we'll patch it once the arena assigns the slot.
        let ah = self.bodies.insert(RigidBody2d::from_desc(BodyHandle(0), desc));
        let handle = body_from(ah);
        // SAFETY: we just inserted at `ah`, so this slot is guaranteed occupied.
        self.bodies.get_mut(ah).expect("just-inserted body").handle = handle;
        self.body_colliders.insert(handle, Vec::new());
        handle
    }

    pub fn add_collider(
        &mut self,
        body: BodyHandle,
        desc: &ColliderDesc,
    ) -> ColliderHandle {
        // Insert with placeholder handle, patch after arena assigns slot.
        let collider = Collider2d::from_desc(ColliderHandle(0), body, desc);

        // Accumulate mass properties onto the body
        if let Some(rb) = self.bodies.get_mut(body_ah(body))
            && rb.is_dynamic()
        {
            let c_mass = collider.compute_mass();
            let c_inertia = collider.compute_inertia(c_mass);
            rb.mass += c_mass;
            rb.inertia += c_inertia;
            rb.inv_mass = 1.0 / rb.mass;
            rb.inv_inertia = if rb.fixed_rotation {
                0.0
            } else {
                1.0 / rb.inertia
            };
        }

        let ah = self.colliders.insert(collider);
        let handle = coll_from(ah);
        // SAFETY: we just inserted at `ah`, so this slot is guaranteed occupied.
        self.colliders.get_mut(ah).expect("just-inserted collider").handle = handle;
        self.body_colliders.entry(body).or_default().push(handle);
        handle
    }

    pub fn add_joint(&mut self, desc: &JointDesc) -> JointHandle {
        let ah = self.joints.insert(Joint2d {
            body_a: desc.body_a,
            body_b: desc.body_b,
            joint_type: desc.joint_type.clone(),
            local_anchor_a: desc.local_anchor_a,
            local_anchor_b: desc.local_anchor_b,
            motor: desc.motor.clone(),
            damping: desc.damping,
            break_force: desc.break_force,
        });
        joint_from(ah)
    }

    pub fn apply_force(&mut self, body: BodyHandle, force: &Force) {
        if let Some(rb) = self.bodies.get_mut(body_ah(body)) {
            // Wake the body
            rb.is_sleeping = false;
            rb.sleep_timer = 0.0;
            rb.force_accumulator[0] += force.vector[0];
            rb.force_accumulator[1] += force.vector[1];
            if let Some(point) = force.point {
                rb.torque_accumulator += point[0] * force.vector[1] - point[1] * force.vector[0];
            }
        }
    }

    pub fn apply_impulse(&mut self, body: BodyHandle, impulse: &Impulse) {
        if let Some(rb) = self.bodies.get_mut(body_ah(body))
            && rb.is_dynamic()
            && rb.inv_mass > 0.0
        {
            // Wake the body
            rb.is_sleeping = false;
            rb.sleep_timer = 0.0;
            rb.linear_velocity[0] += impulse.vector[0] * rb.inv_mass;
            rb.linear_velocity[1] += impulse.vector[1] * rb.inv_mass;
            if let Some(point) = impulse.point {
                let angular_impulse =
                    point[0] * impulse.vector[1] - point[1] * impulse.vector[0];
                rb.angular_velocity += angular_impulse * rb.inv_inertia;
            }
        }
    }

    pub fn apply_torque(&mut self, body: BodyHandle, torque: &Torque) {
        if let Some(rb) = self.bodies.get_mut(body_ah(body)) {
            // Wake the body
            rb.is_sleeping = false;
            rb.sleep_timer = 0.0;
            rb.torque_accumulator += torque.value;
        }
    }

    pub fn remove_body(&mut self, handle: BodyHandle) {
        self.bodies.remove(body_ah(handle));
        if let Some(collider_handles) = self.body_colliders.remove(&handle) {
            for ch in &collider_handles {
                self.colliders.remove(coll_ah(*ch));
            }
            // Clean stale manifolds referencing removed colliders
            self.manifolds
                .retain(|(a, b), _| !collider_handles.contains(a) && !collider_handles.contains(b));
        }
        self.joints
            .retain(|_, j| j.body_a != handle && j.body_b != handle);
    }

    /// Remove a single collider and recompute the parent body's mass properties.
    pub fn remove_collider(&mut self, handle: ColliderHandle) -> Result<(), ImpetusError> {
        let collider = self.colliders.remove(coll_ah(handle))
            .ok_or_else(|| ImpetusError::ColliderNotFound(format!("{:?}", handle)))?;
        let body = collider.body;

        // Remove from parent body's collider list
        if let Some(list) = self.body_colliders.get_mut(&body) {
            list.retain(|ch| *ch != handle);
        }

        // Clean stale manifolds
        self.manifolds
            .retain(|(a, b), _| *a != handle && *b != handle);

        // Recompute mass/inertia from remaining colliders
        if let Some(rb) = self.bodies.get_mut(body_ah(body))
            && rb.is_dynamic()
        {
            let mut mass = 0.0_f64;
            let mut inertia = 0.0_f64;
            if let Some(collider_handles) = self.body_colliders.get(&body) {
                for ch in collider_handles {
                    if let Some(c) = self.colliders.get(coll_ah(*ch)) {
                        let cm = c.compute_mass();
                        mass += cm;
                        inertia += c.compute_inertia(cm);
                    }
                }
            }
            rb.mass = mass;
            rb.inertia = inertia;
            if mass > 0.0 {
                rb.inv_mass = 1.0 / mass;
                rb.inv_inertia = if rb.fixed_rotation { 0.0 } else { 1.0 / inertia };
            } else {
                rb.inv_mass = 0.0;
                rb.inv_inertia = 0.0;
            }
        }

        Ok(())
    }

    /// Remove a joint by handle.
    pub fn remove_joint(&mut self, handle: JointHandle) -> Result<(), ImpetusError> {
        self.joints.remove(joint_ah(handle))
            .ok_or_else(|| ImpetusError::JointNotFound(format!("{:?}", handle)))?;
        Ok(())
    }

    /// Insert a body at a specific handle (for snapshot restore).
    #[cfg(feature = "serialize")]
    pub fn add_body_at(&mut self, handle: BodyHandle, desc: &BodyDesc) {
        let mut rb = RigidBody2d::from_desc(handle, desc);
        rb.handle = handle;
        self.bodies.insert_at(body_ah(handle), rb);
        self.body_colliders.insert(handle, Vec::new());
    }

    /// Insert a collider at a specific handle (for snapshot restore).
    #[cfg(feature = "serialize")]
    pub fn add_collider_at(&mut self, handle: ColliderHandle, body: BodyHandle, desc: &ColliderDesc) {
        let collider = Collider2d::from_desc(handle, body, desc);
        if let Some(rb) = self.bodies.get_mut(body_ah(body))
            && rb.is_dynamic()
        {
            let c_mass = collider.compute_mass();
            let c_inertia = collider.compute_inertia(c_mass);
            rb.mass += c_mass;
            rb.inertia += c_inertia;
            rb.inv_mass = 1.0 / rb.mass;
            rb.inv_inertia = if rb.fixed_rotation { 0.0 } else { 1.0 / rb.inertia };
        }
        self.colliders.insert_at(coll_ah(handle), collider);
        self.body_colliders.entry(body).or_default().push(handle);
    }

    /// Insert a joint at a specific handle (for snapshot restore).
    #[cfg(feature = "serialize")]
    pub fn add_joint_at(&mut self, handle: JointHandle, desc: &JointDesc) {
        self.joints.insert_at(joint_ah(handle), Joint2d {
            body_a: desc.body_a,
            body_b: desc.body_b,
            joint_type: desc.joint_type.clone(),
            local_anchor_a: desc.local_anchor_a,
            local_anchor_b: desc.local_anchor_b,
            motor: desc.motor.clone(),
            damping: desc.damping,
            break_force: desc.break_force,
        });
    }

    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    pub fn get_body_state(&self, handle: BodyHandle) -> Result<BodyState, ImpetusError> {
        let rb = self
            .bodies
            .get(body_ah(handle))
            .ok_or_else(|| ImpetusError::BodyNotFound(format!("{:?}", handle)))?;
        Ok(BodyState {
            handle: rb.handle,
            body_type: rb.body_type,
            position: [rb.position[0], rb.position[1], 0.0],
            rotation: rb.rotation,
            linear_velocity: [rb.linear_velocity[0], rb.linear_velocity[1], 0.0],
            angular_velocity: rb.angular_velocity,
            is_sleeping: rb.is_sleeping,
        })
    }

    pub fn set_body_state(&mut self, handle: BodyHandle, state: &BodyState) -> Result<(), ImpetusError> {
        let rb = self.bodies.get_mut(body_ah(handle))
            .ok_or_else(|| ImpetusError::BodyNotFound(format!("{:?}", handle)))?;
        rb.position = [state.position[0], state.position[1]];
        rb.rotation = state.rotation;
        rb.linear_velocity = [state.linear_velocity[0], state.linear_velocity[1]];
        rb.angular_velocity = state.angular_velocity;
        rb.is_sleeping = false; // wake on teleport
        rb.sleep_timer = 0.0;
        Ok(())
    }

    pub fn set_body_type(&mut self, handle: BodyHandle, body_type: BodyType) -> Result<(), ImpetusError> {
        let rb = self.bodies.get_mut(body_ah(handle))
            .ok_or_else(|| ImpetusError::BodyNotFound(format!("{:?}", handle)))?;
        rb.body_type = body_type;
        // Reset mass properties if switching to/from static
        match body_type {
            BodyType::Static | BodyType::Kinematic => {
                rb.inv_mass = 0.0;
                rb.inv_inertia = 0.0;
                rb.linear_velocity = [0.0, 0.0];
                rb.angular_velocity = 0.0;
            }
            BodyType::Dynamic => {
                // Recompute from colliders if mass is zero
                if rb.mass > 0.0 {
                    rb.inv_mass = 1.0 / rb.mass;
                    rb.inv_inertia = if rb.fixed_rotation { 0.0 } else { 1.0 / rb.inertia };
                }
            }
        }
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Simulation step
    // -----------------------------------------------------------------------

    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        gravity: [f64; 3],
        dt: f64,
        velocity_iterations: u32,
        position_iterations: u32,
        slop: f64,
        correction: f64,
        max_velocity: f64,
    ) -> Vec<CollisionEvent> {
        let gravity_2d = [gravity[0], gravity[1]];
        // 1. Integrate velocities
        for rb in self.bodies.values_mut() {
            rb.integrate_velocities(gravity_2d, dt, max_velocity);
        }

        // 2. Broadphase
        let broad_pairs = self.broadphase();

        // 3. Narrowphase — raw contacts
        let contacts = self.narrowphase(&broad_pairs);

        // 4. Update manifold cache (match contacts to existing manifolds)
        self.update_manifolds(&contacts);

        // 5. Wake sleeping bodies on contact with non-sleeping moving bodies
        for manifold in self.manifolds.values() {
            let a_sleeping = self
                .bodies
                .get(body_ah(manifold.body_a))
                .is_some_and(|b| b.is_sleeping);
            let b_sleeping = self
                .bodies
                .get(body_ah(manifold.body_b))
                .is_some_and(|b| b.is_sleeping);
            let a_moving = self.bodies.get(body_ah(manifold.body_a)).is_some_and(|b| {
                !b.is_sleeping
                    && b.is_dynamic()
                    && (b.linear_velocity[0].abs() > SLEEP_VELOCITY_THRESHOLD
                        || b.linear_velocity[1].abs() > SLEEP_VELOCITY_THRESHOLD
                        || b.angular_velocity.abs() > SLEEP_VELOCITY_THRESHOLD)
            });
            let b_moving = self.bodies.get(body_ah(manifold.body_b)).is_some_and(|b| {
                !b.is_sleeping
                    && b.is_dynamic()
                    && (b.linear_velocity[0].abs() > SLEEP_VELOCITY_THRESHOLD
                        || b.linear_velocity[1].abs() > SLEEP_VELOCITY_THRESHOLD
                        || b.angular_velocity.abs() > SLEEP_VELOCITY_THRESHOLD)
            });
            if a_sleeping
                && b_moving
                && let Some(ba) = self.bodies.get_mut(body_ah(manifold.body_a))
            {
                ba.is_sleeping = false;
                ba.sleep_timer = 0.0;
            }
            if b_sleeping
                && a_moving
                && let Some(bb) = self.bodies.get_mut(body_ah(manifold.body_b))
            {
                bb.is_sleeping = false;
                bb.sleep_timer = 0.0;
            }
        }

        // 6. Warm start — apply cached impulses from previous frame
        self.warm_start();

        // 7. Solve velocity constraints (using manifolds with accumulation)
        self.solve_contacts(velocity_iterations);

        // 8. Solve joint constraints
        self.solve_joints(dt, velocity_iterations);

        // 9. Positional correction
        self.solve_positions(position_iterations, slop, correction);

        // 10. Integrate positions
        for rb in self.bodies.values_mut() {
            rb.integrate_positions(dt);
        }

        // 11. Sleep check: put nearly-stationary dynamic bodies to sleep
        for rb in self.bodies.values_mut() {
            if !rb.is_dynamic() || rb.inv_mass == 0.0 {
                continue;
            }
            let lin_speed = (rb.linear_velocity[0] * rb.linear_velocity[0]
                + rb.linear_velocity[1] * rb.linear_velocity[1])
            .sqrt();
            let ang_speed = rb.angular_velocity.abs();
            if lin_speed < SLEEP_VELOCITY_THRESHOLD && ang_speed < SLEEP_VELOCITY_THRESHOLD {
                rb.sleep_timer += dt;
                if rb.sleep_timer >= SLEEP_TIME_THRESHOLD {
                    rb.is_sleeping = true;
                }
            } else {
                rb.sleep_timer = 0.0;
                rb.is_sleeping = false;
            }
        }

        // 12. Clear forces
        for rb in self.bodies.values_mut() {
            rb.clear_forces();
        }

        // 13. Generate collision events (from manifold keys)
        self.generate_events()
    }

    // -----------------------------------------------------------------------
    // Broadphase — spatial hash grid
    // -----------------------------------------------------------------------

    fn broadphase(&self) -> Vec<(ColliderHandle, ColliderHandle)> {
        // Compute AABBs for all colliders
        let collider_aabbs: Vec<(ColliderHandle, Aabb2d)> = self
            .colliders
            .values()
            .filter_map(|c| {
                let rb = self.bodies.get(body_ah(c.body))?;
                Some((c.handle, c.world_aabb(rb.position, rb.rotation)))
            })
            .collect();

        // Build spatial hash
        let cell_size = SpatialHashGrid::<ColliderHandle>::auto_cell_size(
            collider_aabbs.iter().map(|(_, aabb)| {
                let w = aabb.max[0] - aabb.min[0];
                let h = aabb.max[1] - aabb.min[1];
                w.max(h)
            }),
            collider_aabbs.len(),
        );
        let mut grid = SpatialHashGrid::new(cell_size);
        for (handle, aabb) in &collider_aabbs {
            grid.insert_2d(*handle, aabb.min, aabb.max);
        }

        // Collect candidate pairs from shared cells
        let candidates = grid.query_pairs();

        // Build AABB lookup for overlap verification
        let aabb_map: BTreeMap<ColliderHandle, Aabb2d> =
            collider_aabbs.into_iter().collect();

        // Filter candidates
        let mut pairs = Vec::with_capacity(candidates.len());
        for (ha, hb) in candidates {
            let ca = match self.colliders.get(coll_ah(ha)) {
                Some(c) => c,
                None => continue,
            };
            let cb = match self.colliders.get(coll_ah(hb)) {
                Some(c) => c,
                None => continue,
            };
            // Skip same-body
            if ca.body == cb.body {
                continue;
            }
            // Skip static-static
            if let (Some(ba), Some(bb)) = (self.bodies.get(body_ah(ca.body)), self.bodies.get(body_ah(cb.body)))
                && ba.is_static() && bb.is_static()
            {
                continue;
            }
            // Skip sensor-sensor
            if ca.is_sensor && cb.is_sensor {
                continue;
            }
            // Skip if collision layers don't match
            if (ca.collision_layer & cb.collision_mask) == 0
                && (cb.collision_layer & ca.collision_mask) == 0
            {
                continue;
            }
            // Verify AABB overlap (spatial hash cells are conservative)
            if let (Some(aabb_a), Some(aabb_b)) = (aabb_map.get(&ha), aabb_map.get(&hb))
                && aabb_a.overlaps(aabb_b)
            {
                pairs.push((ha, hb));
            }
        }
        pairs
    }

    // -----------------------------------------------------------------------
    // Narrowphase
    // -----------------------------------------------------------------------

    fn narrowphase(
        &self,
        broad_pairs: &[(ColliderHandle, ColliderHandle)],
    ) -> Vec<Contact> {
        let mut contacts = Vec::new();

        for (ha, hb) in broad_pairs {
            let ca = match self.colliders.get(coll_ah(*ha)) {
                Some(c) => c,
                None => continue,
            };
            let cb = match self.colliders.get(coll_ah(*hb)) {
                Some(c) => c,
                None => continue,
            };
            let ba = match self.bodies.get(body_ah(ca.body)) {
                Some(b) => b,
                None => continue,
            };
            let bb = match self.bodies.get(body_ah(cb.body)) {
                Some(b) => b,
                None => continue,
            };

            let pos_a = world_pos(ba.position, ba.rotation, ca.offset);
            let pos_b = world_pos(bb.position, bb.rotation, cb.offset);

            if let Some((normal, depth, point)) =
                generate_contact(&ca.shape, pos_a, ba.rotation, &cb.shape, pos_b, bb.rotation)
            {
                contacts.push(Contact {
                    collider_a: *ha,
                    collider_b: *hb,
                    body_a: ca.body,
                    body_b: cb.body,
                    normal,
                    depth,
                    point,
                });
            }
        }

        contacts
    }

    // -----------------------------------------------------------------------
    // Manifold update — match new contacts to existing manifolds
    // -----------------------------------------------------------------------

    fn update_manifolds(&mut self, contacts: &[Contact]) {
        // Track which manifold keys appear in this frame's contacts
        let mut current_keys: BTreeSet<ManifoldKey> = BTreeSet::new();

        for contact in contacts {
            let key = ordered_manifold_key(contact.collider_a, contact.collider_b);
            current_keys.insert(key);

            // Transform contact point to body-local coordinates
            let (local_a, local_b) = {
                let pos_a = self.bodies.get(body_ah(contact.body_a))
                    .map(|b| (b.position, b.rotation))
                    .unwrap_or(([0.0, 0.0], 0.0));
                let pos_b = self.bodies.get(body_ah(contact.body_b))
                    .map(|b| (b.position, b.rotation))
                    .unwrap_or(([0.0, 0.0], 0.0));
                (
                    world_to_local(contact.point, pos_a.0, pos_a.1),
                    world_to_local(contact.point, pos_b.0, pos_b.1),
                )
            };

            if let Some(manifold) = self.manifolds.get_mut(&key) {
                // Existing manifold — update normal and match points
                manifold.normal = if key.0 == contact.collider_a {
                    contact.normal
                } else {
                    [-contact.normal[0], -contact.normal[1]]
                };
                manifold.body_a = if key.0 == contact.collider_a { contact.body_a } else { contact.body_b };
                manifold.body_b = if key.0 == contact.collider_a { contact.body_b } else { contact.body_a };

                let new_local_a = if key.0 == contact.collider_a { local_a } else { local_b };
                let new_local_b = if key.0 == contact.collider_a { local_b } else { local_a };

                // Try to match the new contact point to an existing manifold point
                let mut best_idx: Option<usize> = None;
                let mut best_dist_sq = MANIFOLD_MATCH_THRESHOLD * MANIFOLD_MATCH_THRESHOLD;
                for (i, mp) in manifold.points.iter().enumerate() {
                    let dx = mp.local_a[0] - new_local_a[0];
                    let dy = mp.local_a[1] - new_local_a[1];
                    let dist_sq = dx * dx + dy * dy;
                    if dist_sq < best_dist_sq {
                        best_dist_sq = dist_sq;
                        best_idx = Some(i);
                    }
                }

                if let Some(idx) = best_idx {
                    // Update the matched point, preserving accumulated impulses
                    manifold.points[idx].local_a = new_local_a;
                    manifold.points[idx].local_b = new_local_b;
                    manifold.points[idx].depth = contact.depth;
                } else {
                    // New point — zero impulses
                    manifold.points.clear(); // single-point manifold: replace
                    manifold.points.push(ManifoldPoint {
                        local_a: new_local_a,
                        local_b: new_local_b,
                        normal_impulse: 0.0,
                        tangent_impulse: 0.0,
                        depth: contact.depth,
                    });
                }
            } else {
                // New manifold
                let (ca, cb, ba, bb, normal, la, lb) = if key.0 == contact.collider_a {
                    (contact.collider_a, contact.collider_b, contact.body_a, contact.body_b, contact.normal, local_a, local_b)
                } else {
                    (contact.collider_b, contact.collider_a, contact.body_b, contact.body_a,
                     [-contact.normal[0], -contact.normal[1]], local_b, local_a)
                };
                self.manifolds.insert(key, ContactManifold {
                    collider_a: ca,
                    collider_b: cb,
                    body_a: ba,
                    body_b: bb,
                    normal,
                    points: vec![ManifoldPoint {
                        local_a: la,
                        local_b: lb,
                        normal_impulse: 0.0,
                        tangent_impulse: 0.0,
                        depth: contact.depth,
                    }],
                });
            }
        }

        // Remove manifolds for pairs no longer in contact
        self.manifolds.retain(|k, _| current_keys.contains(k));
    }

    // -----------------------------------------------------------------------
    // Warm starting — apply cached impulses from previous frame
    // -----------------------------------------------------------------------

    fn warm_start(&mut self) {
        // Collect warm-start data to avoid borrow conflicts
        struct WarmData {
            body_a: BodyHandle,
            body_b: BodyHandle,
            impulse_n: [f64; 2],
            impulse_t: [f64; 2],
            ra_cross_n: f64,
            rb_cross_n: f64,
            ra_cross_t: f64,
            rb_cross_t: f64,
            j_n: f64,
            jt: f64,
        }

        let warm_data: Vec<WarmData> = self.manifolds.values().flat_map(|manifold| {
            let pos_a = self.bodies.get(body_ah(manifold.body_a))
                .map(|b| b.position)
                .unwrap_or([0.0, 0.0]);
            let pos_b = self.bodies.get(body_ah(manifold.body_b))
                .map(|b| b.position)
                .unwrap_or([0.0, 0.0]);

            // Check if this is a sensor contact
            let is_sensor = match (
                self.colliders.get(coll_ah(manifold.collider_a)),
                self.colliders.get(coll_ah(manifold.collider_b)),
            ) {
                (Some(a), Some(b)) => a.is_sensor || b.is_sensor,
                _ => false,
            };
            if is_sensor {
                return Vec::new();
            }

            let n = manifold.normal;
            let tangent = [-n[1], n[0]];

            manifold.points.iter().filter_map(|mp| {
                let j_n = mp.normal_impulse * WARM_START_FACTOR;
                let jt = mp.tangent_impulse * WARM_START_FACTOR;
                if j_n.abs() < EPSILON && jt.abs() < EPSILON {
                    return None;
                }

                // Reconstruct world-space contact point from body A's local coords
                let rot_a = self.bodies.get(body_ah(manifold.body_a))
                    .map(|b| b.rotation).unwrap_or(0.0);
                let cp = local_to_world(mp.local_a, pos_a, rot_a);

                let ra = [cp[0] - pos_a[0], cp[1] - pos_a[1]];
                let rb = [cp[0] - pos_b[0], cp[1] - pos_b[1]];

                Some(WarmData {
                    body_a: manifold.body_a,
                    body_b: manifold.body_b,
                    impulse_n: [j_n * n[0], j_n * n[1]],
                    impulse_t: [jt * tangent[0], jt * tangent[1]],
                    ra_cross_n: ra[0] * n[1] - ra[1] * n[0],
                    rb_cross_n: rb[0] * n[1] - rb[1] * n[0],
                    ra_cross_t: ra[0] * tangent[1] - ra[1] * tangent[0],
                    rb_cross_t: rb[0] * tangent[1] - rb[1] * tangent[0],
                    j_n,
                    jt,
                })
            }).collect::<Vec<_>>()
        }).collect();

        for wd in &warm_data {
            let total_impulse = [
                wd.impulse_n[0] + wd.impulse_t[0],
                wd.impulse_n[1] + wd.impulse_t[1],
            ];
            if let Some(ba) = self.bodies.get_mut(body_ah(wd.body_a))
                && ba.is_dynamic()
            {
                ba.linear_velocity[0] -= total_impulse[0] * ba.inv_mass;
                ba.linear_velocity[1] -= total_impulse[1] * ba.inv_mass;
                ba.angular_velocity -= (wd.ra_cross_n * wd.j_n + wd.ra_cross_t * wd.jt) * ba.inv_inertia;
            }
            if let Some(bb) = self.bodies.get_mut(body_ah(wd.body_b))
                && bb.is_dynamic()
            {
                bb.linear_velocity[0] += total_impulse[0] * bb.inv_mass;
                bb.linear_velocity[1] += total_impulse[1] * bb.inv_mass;
                bb.angular_velocity += (wd.rb_cross_n * wd.j_n + wd.rb_cross_t * wd.jt) * bb.inv_inertia;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Contact constraint solver with accumulated impulses, friction, angular response
    // -----------------------------------------------------------------------

    fn solve_contacts(&mut self, iterations: u32) {
        use crate::material::CombineRule;

        /// Velocity threshold below which restitution is zeroed to prevent
        /// micro-bouncing of resting objects.
        const RESTITUTION_VELOCITY_THRESHOLD: f64 = 1.0;

        // Pre-extract material properties per manifold to avoid repeated lookups
        struct ManifoldMaterial {
            restitution: f64,
            friction: f64,
            rolling_friction: f64,
            is_sensor: bool,
        }

        /// Pick the higher-priority combine rule, then apply it.
        fn combine_property(a: f64, b: f64, rule_a: CombineRule, rule_b: CombineRule) -> f64 {
            let rule = rule_a.max(rule_b);
            rule.combine(a, b)
        }

        let keys: Vec<ManifoldKey> = self.manifolds.keys().copied().collect();
        let materials: Vec<ManifoldMaterial> = keys.iter().map(|key| {
            let manifold = &self.manifolds[key];
            let (rest, fric, roll_fric, sensor) = match (
                self.colliders.get(coll_ah(manifold.collider_a)),
                self.colliders.get(coll_ah(manifold.collider_b)),
            ) {
                (Some(a), Some(b)) => (
                    combine_property(
                        a.material.restitution,
                        b.material.restitution,
                        a.material.restitution_combine,
                        b.material.restitution_combine,
                    ),
                    combine_property(
                        a.material.friction,
                        b.material.friction,
                        a.material.friction_combine,
                        b.material.friction_combine,
                    ),
                    (a.material.rolling_friction + b.material.rolling_friction) * 0.5,
                    a.is_sensor || b.is_sensor,
                ),
                _ => (0.0, 0.0, 0.0, false),
            };
            ManifoldMaterial {
                restitution: rest,
                friction: fric,
                rolling_friction: roll_fric,
                is_sensor: sensor,
            }
        }).collect();

        for _ in 0..iterations {
            for (ki, key) in keys.iter().enumerate() {
                if materials[ki].is_sensor {
                    continue;
                }

                let manifold = match self.manifolds.get(key) {
                    Some(m) => m,
                    None => continue,
                };

                let (inv_mass_a, inv_inertia_a, pos_a, rot_a) = {
                    let ba = match self.bodies.get(body_ah(manifold.body_a)) {
                        Some(b) => b,
                        None => continue,
                    };
                    (ba.inv_mass, ba.inv_inertia, ba.position, ba.rotation)
                };
                let (inv_mass_b, inv_inertia_b, pos_b) = {
                    let bb = match self.bodies.get(body_ah(manifold.body_b)) {
                        Some(b) => b,
                        None => continue,
                    };
                    (bb.inv_mass, bb.inv_inertia, bb.position)
                };

                if inv_mass_a == 0.0 && inv_mass_b == 0.0 {
                    continue;
                }

                let n = manifold.normal;
                let body_a_handle = manifold.body_a;
                let body_b_handle = manifold.body_b;
                let num_points = manifold.points.len();

                // Process each manifold point
                for pi in 0..num_points {
                    let mp = &self.manifolds[key].points[pi];

                    // Reconstruct world-space contact point from body A's local coords
                    let cp = local_to_world(mp.local_a, pos_a, rot_a);
                    let ra = [cp[0] - pos_a[0], cp[1] - pos_a[1]];
                    let rb = [cp[0] - pos_b[0], cp[1] - pos_b[1]];

                    // Re-read velocities (they change during iteration)
                    let (vel_a, angvel_a) = {
                        let ba = match self.bodies.get(body_ah(body_a_handle)) {
                            Some(b) => b,
                            None => continue,
                        };
                        (ba.linear_velocity, ba.angular_velocity)
                    };
                    let (vel_b, angvel_b) = {
                        let bb = match self.bodies.get(body_ah(body_b_handle)) {
                            Some(b) => b,
                            None => continue,
                        };
                        (bb.linear_velocity, bb.angular_velocity)
                    };

                    // Relative velocity at contact point (including angular)
                    let vel_a_at_cp = [
                        vel_a[0] - angvel_a * ra[1],
                        vel_a[1] + angvel_a * ra[0],
                    ];
                    let vel_b_at_cp = [
                        vel_b[0] - angvel_b * rb[1],
                        vel_b[1] + angvel_b * rb[0],
                    ];
                    let rel_vel = [vel_b_at_cp[0] - vel_a_at_cp[0], vel_b_at_cp[1] - vel_a_at_cp[1]];
                    let vel_along_normal = rel_vel[0] * n[0] + rel_vel[1] * n[1];

                    // Angular effective mass
                    let ra_cross_n = ra[0] * n[1] - ra[1] * n[0];
                    let rb_cross_n = rb[0] * n[1] - rb[1] * n[0];
                    let inv_mass_sum = inv_mass_a + inv_mass_b
                        + ra_cross_n * ra_cross_n * inv_inertia_a
                        + rb_cross_n * rb_cross_n * inv_inertia_b;

                    // Normal impulse with accumulation
                    // Suppress restitution at low velocities to prevent micro-bouncing.
                    let restitution = if vel_along_normal.abs() < RESTITUTION_VELOCITY_THRESHOLD {
                        0.0
                    } else {
                        materials[ki].restitution
                    };
                    let j_new = -(1.0 + restitution) * vel_along_normal / inv_mass_sum;
                    let j_old = self.manifolds[key].points[pi].normal_impulse;
                    let j_accumulated = (j_old + j_new).max(0.0);
                    let j_applied = j_accumulated - j_old;
                    self.manifolds.get_mut(key).unwrap().points[pi].normal_impulse = j_accumulated;

                    let impulse_n = [j_applied * n[0], j_applied * n[1]];

                    if let Some(ba) = self.bodies.get_mut(body_ah(body_a_handle))
                        && ba.is_dynamic()
                    {
                        ba.linear_velocity[0] -= impulse_n[0] * ba.inv_mass;
                        ba.linear_velocity[1] -= impulse_n[1] * ba.inv_mass;
                        ba.angular_velocity -= ra_cross_n * j_applied * ba.inv_inertia;
                    }
                    if let Some(bb) = self.bodies.get_mut(body_ah(body_b_handle))
                        && bb.is_dynamic()
                    {
                        bb.linear_velocity[0] += impulse_n[0] * bb.inv_mass;
                        bb.linear_velocity[1] += impulse_n[1] * bb.inv_mass;
                        bb.angular_velocity += rb_cross_n * j_applied * bb.inv_inertia;
                    }

                    // Friction impulse with accumulation
                    let friction = materials[ki].friction;
                    if friction > 0.0 {
                        // Re-read velocities after normal impulse application
                        let (vel_a, angvel_a) = {
                            let ba = match self.bodies.get(body_ah(body_a_handle)) {
                                Some(b) => b,
                                None => continue,
                            };
                            (ba.linear_velocity, ba.angular_velocity)
                        };
                        let (vel_b, angvel_b) = {
                            let bb = match self.bodies.get(body_ah(body_b_handle)) {
                                Some(b) => b,
                                None => continue,
                            };
                            (bb.linear_velocity, bb.angular_velocity)
                        };

                        let vel_a_at_cp = [
                            vel_a[0] - angvel_a * ra[1],
                            vel_a[1] + angvel_a * ra[0],
                        ];
                        let vel_b_at_cp = [
                            vel_b[0] - angvel_b * rb[1],
                            vel_b[1] + angvel_b * rb[0],
                        ];
                        let rel_vel = [vel_b_at_cp[0] - vel_a_at_cp[0], vel_b_at_cp[1] - vel_a_at_cp[1]];

                        let tangent = [-n[1], n[0]];
                        let vel_along_tangent = rel_vel[0] * tangent[0] + rel_vel[1] * tangent[1];

                        let ra_cross_t = ra[0] * tangent[1] - ra[1] * tangent[0];
                        let rb_cross_t = rb[0] * tangent[1] - rb[1] * tangent[0];
                        let inv_mass_sum_t = inv_mass_a + inv_mass_b
                            + ra_cross_t * ra_cross_t * inv_inertia_a
                            + rb_cross_t * rb_cross_t * inv_inertia_b;

                        let jt_new = -vel_along_tangent / inv_mass_sum_t;
                        let jt_old = self.manifolds[key].points[pi].tangent_impulse;
                        let max_friction = j_accumulated.abs() * friction;
                        let jt_accumulated = (jt_old + jt_new).clamp(-max_friction, max_friction);
                        let jt_applied = jt_accumulated - jt_old;
                        self.manifolds.get_mut(key).unwrap().points[pi].tangent_impulse = jt_accumulated;

                        let impulse_t = [jt_applied * tangent[0], jt_applied * tangent[1]];

                        if let Some(ba) = self.bodies.get_mut(body_ah(body_a_handle))
                            && ba.is_dynamic()
                        {
                            ba.linear_velocity[0] -= impulse_t[0] * ba.inv_mass;
                            ba.linear_velocity[1] -= impulse_t[1] * ba.inv_mass;
                            ba.angular_velocity -= ra_cross_t * jt_applied * ba.inv_inertia;
                        }
                        if let Some(bb) = self.bodies.get_mut(body_ah(body_b_handle))
                            && bb.is_dynamic()
                        {
                            bb.linear_velocity[0] += impulse_t[0] * bb.inv_mass;
                            bb.linear_velocity[1] += impulse_t[1] * bb.inv_mass;
                            bb.angular_velocity += rb_cross_t * jt_applied * bb.inv_inertia;
                        }
                    }

                    // Rolling friction — apply torque opposing angular velocity
                    let rolling_friction = materials[ki].rolling_friction;
                    if rolling_friction > 0.0 {
                        let normal_force = j_accumulated.abs();
                        let roll_torque = rolling_friction * normal_force;

                        if let Some(ba) = self.bodies.get_mut(body_ah(body_a_handle))
                            && ba.is_dynamic()
                            && ba.angular_velocity.abs() > EPSILON
                        {
                            let old_sign = ba.angular_velocity > 0.0;
                            let sign = if old_sign { -1.0 } else { 1.0 };
                            ba.angular_velocity += sign * roll_torque * ba.inv_inertia;
                            // Don't reverse direction
                            if (ba.angular_velocity > 0.0) != old_sign {
                                ba.angular_velocity = 0.0;
                            }
                        }
                        if let Some(bb) = self.bodies.get_mut(body_ah(body_b_handle))
                            && bb.is_dynamic()
                            && bb.angular_velocity.abs() > EPSILON
                        {
                            let old_sign = bb.angular_velocity > 0.0;
                            let sign = if old_sign { -1.0 } else { 1.0 };
                            bb.angular_velocity += sign * roll_torque * bb.inv_inertia;
                            // Don't reverse direction
                            if (bb.angular_velocity > 0.0) != old_sign {
                                bb.angular_velocity = 0.0;
                            }
                        }
                    }
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Positional correction (Baumgarte stabilization)
    // -----------------------------------------------------------------------

    fn solve_positions(&mut self, iterations: u32, slop: f64, percent: f64) {
        // Collect positional correction data from manifolds
        struct PosCorrection {
            body_a: BodyHandle,
            body_b: BodyHandle,
            normal: [f64; 2],
            depth: f64,
            is_sensor: bool,
        }

        let corrections: Vec<PosCorrection> = self.manifolds.values().flat_map(|manifold| {
            let is_sensor = match (
                self.colliders.get(coll_ah(manifold.collider_a)),
                self.colliders.get(coll_ah(manifold.collider_b)),
            ) {
                (Some(a), Some(b)) => a.is_sensor || b.is_sensor,
                _ => false,
            };
            manifold.points.iter().map(move |mp| PosCorrection {
                body_a: manifold.body_a,
                body_b: manifold.body_b,
                normal: manifold.normal,
                depth: mp.depth,
                is_sensor,
            })
        }).collect();

        for _ in 0..iterations {
            for corr in &corrections {
                if corr.is_sensor {
                    continue;
                }

                let inv_mass_a = self.bodies.get(body_ah(corr.body_a)).map(|b| b.inv_mass).unwrap_or(0.0);
                let inv_mass_b = self.bodies.get(body_ah(corr.body_b)).map(|b| b.inv_mass).unwrap_or(0.0);
                let inv_mass_sum = inv_mass_a + inv_mass_b;

                if inv_mass_sum == 0.0 {
                    continue;
                }

                let n = corr.normal;
                let correction_mag = (corr.depth - slop).max(0.0) / inv_mass_sum * percent;
                let correction = [correction_mag * n[0], correction_mag * n[1]];

                if let Some(ba) = self.bodies.get_mut(body_ah(corr.body_a))
                    && ba.is_dynamic()
                {
                    ba.position[0] -= correction[0] * ba.inv_mass;
                    ba.position[1] -= correction[1] * ba.inv_mass;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(corr.body_b))
                    && bb.is_dynamic()
                {
                    bb.position[0] += correction[0] * bb.inv_mass;
                    bb.position[1] += correction[1] * bb.inv_mass;
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Joint constraint solver
    // -----------------------------------------------------------------------

    fn solve_joints(&mut self, dt: f64, iterations: u32) {
        // Collect (ArenaHandle, Joint2d) pairs so we can track handles for breaking.
        let joints: Vec<(ArenaHandle, Joint2d)> = self.joints.iter()
            .map(|(ah, j)| (ah, j.clone()))
            .collect();

        // Track constraint forces for joint breaking.
        // We accumulate the maximum constraint force per joint across iterations.
        let mut constraint_forces: Vec<f64> = vec![0.0; joints.len()];

        for _ in 0..iterations {
            for (ji, (_ah, joint)) in joints.iter().enumerate() {
                let force = match &joint.joint_type {
                    JointType::Fixed => self.solve_fixed_joint(joint),
                    JointType::Distance { length } => {
                        self.solve_distance_joint(joint, *length)
                    }
                    JointType::Spring {
                        rest_length,
                        stiffness,
                        damping,
                    } => {
                        self.solve_spring_joint(joint, *rest_length, *stiffness, *damping, dt)
                    }
                    JointType::Revolute { limits, .. } => {
                        let f = self.solve_revolute_joint(joint, limits.as_ref());
                        if let Some(motor) = &joint.motor {
                            self.solve_revolute_motor(joint, motor, dt);
                        }
                        f
                    }
                    JointType::Prismatic { axis, limits } => {
                        let f = self.solve_prismatic_joint(joint, *axis, limits.as_ref());
                        if let Some(motor) = &joint.motor {
                            self.solve_prismatic_motor(joint, *axis, motor, dt);
                        }
                        f
                    }
                    JointType::Wheel { axis, stiffness, damping } => {
                        self.solve_wheel_joint(joint, *axis, *stiffness, *damping, dt)
                    }
                    JointType::Rope { max_length } => {
                        self.solve_rope_joint(joint, *max_length)
                    }
                    JointType::Mouse { target, stiffness, damping, max_force } => {
                        self.solve_mouse_joint(joint, *target, *stiffness, *damping, *max_force, dt)
                    }
                };
                constraint_forces[ji] = constraint_forces[ji].max(force);
                // Apply joint damping for non-Spring joints (Spring has its own damping).
                if joint.damping > 0.0 && !matches!(joint.joint_type, JointType::Spring { .. }) {
                    self.apply_joint_damping(joint, dt);
                }
            }
        }

        // Joint breaking — remove joints whose constraint force exceeded break_force.
        let mut broken: Vec<ArenaHandle> = Vec::new();
        for (ji, (ah, joint)) in joints.iter().enumerate() {
            if let Some(bf) = joint.break_force
                && constraint_forces[ji] > bf
            {
                broken.push(*ah);
            }
        }
        for ah in broken {
            self.joints.remove(ah);
        }
    }

    /// Apply velocity damping proportional to relative velocity at anchor points.
    fn apply_joint_damping(&mut self, joint: &Joint2d, dt: f64) {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);

        let (vel_a, angvel_a, pos_a, inv_mass_a) = match self.bodies.get(body_ah(joint.body_a)) {
            Some(b) if b.is_dynamic() => (b.linear_velocity, b.angular_velocity, b.position, b.inv_mass),
            Some(b) => (b.linear_velocity, b.angular_velocity, b.position, 0.0),
            None => return,
        };
        let (vel_b, angvel_b, pos_b, inv_mass_b) = match self.bodies.get(body_ah(joint.body_b)) {
            Some(b) if b.is_dynamic() => (b.linear_velocity, b.angular_velocity, b.position, b.inv_mass),
            Some(b) => (b.linear_velocity, b.angular_velocity, b.position, 0.0),
            None => return,
        };

        let inv_mass_sum = inv_mass_a + inv_mass_b;
        if inv_mass_sum == 0.0 {
            return;
        }

        // Velocity at anchor point = linear_vel + angular_vel x r
        let ra = [anchor_a[0] - pos_a[0], anchor_a[1] - pos_a[1]];
        let rb = [anchor_b[0] - pos_b[0], anchor_b[1] - pos_b[1]];
        let va = [vel_a[0] - angvel_a * ra[1], vel_a[1] + angvel_a * ra[0]];
        let vb = [vel_b[0] - angvel_b * rb[1], vel_b[1] + angvel_b * rb[0]];
        let rel_vel = [vb[0] - va[0], vb[1] - va[1]];

        let rel_speed_sq = rel_vel[0] * rel_vel[0] + rel_vel[1] * rel_vel[1];
        if rel_speed_sq < EPSILON_SQ {
            return;
        }

        // Damping impulse: -damping * relative_velocity * dt, distributed by inverse mass
        let impulse = [
            -joint.damping * rel_vel[0] * dt,
            -joint.damping * rel_vel[1] * dt,
        ];

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity[0] -= impulse[0] * ba.inv_mass;
            ba.linear_velocity[1] -= impulse[1] * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity[0] += impulse[0] * bb.inv_mass;
            bb.linear_velocity[1] += impulse[1] * bb.inv_mass;
        }
    }

    /// Apply a revolute motor: drives relative angular velocity toward `target_velocity`,
    /// clamped by `max_force` (torque).
    fn solve_revolute_motor(&mut self, joint: &Joint2d, motor: &JointMotor, dt: f64) {
        let angvel_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .map(|b| b.angular_velocity)
            .unwrap_or(0.0);
        let angvel_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .map(|b| b.angular_velocity)
            .unwrap_or(0.0);
        let inv_inertia_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .filter(|b| b.is_dynamic())
            .map(|b| b.inv_inertia)
            .unwrap_or(0.0);
        let inv_inertia_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .filter(|b| b.is_dynamic())
            .map(|b| b.inv_inertia)
            .unwrap_or(0.0);

        let inv_inertia_sum = inv_inertia_a + inv_inertia_b;
        if inv_inertia_sum == 0.0 {
            return;
        }

        let rel_angvel = angvel_b - angvel_a;
        let error = motor.target_velocity - rel_angvel;
        // Impulse = error / inv_inertia_sum, clamped by max_force * dt
        let max_impulse = motor.max_force * dt;
        let impulse = (error / inv_inertia_sum).clamp(-max_impulse, max_impulse);

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.angular_velocity -= impulse * ba.inv_inertia;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.angular_velocity += impulse * bb.inv_inertia;
        }
    }

    /// Apply a prismatic motor: drives relative linear velocity along `axis` toward
    /// `target_velocity`, clamped by `max_force`.
    fn solve_prismatic_motor(
        &mut self,
        joint: &Joint2d,
        axis: [f64; 2],
        motor: &JointMotor,
        dt: f64,
    ) {
        let axis_len = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
        if axis_len < EPSILON {
            return;
        }
        let ax = [axis[0] / axis_len, axis[1] / axis_len];

        let vel_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let vel_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let inv_mass_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .filter(|b| b.is_dynamic())
            .map(|b| b.inv_mass)
            .unwrap_or(0.0);
        let inv_mass_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .filter(|b| b.is_dynamic())
            .map(|b| b.inv_mass)
            .unwrap_or(0.0);

        let inv_mass_sum = inv_mass_a + inv_mass_b;
        if inv_mass_sum == 0.0 {
            return;
        }

        let rel_vel = [vel_b[0] - vel_a[0], vel_b[1] - vel_a[1]];
        let rel_speed_along_axis = rel_vel[0] * ax[0] + rel_vel[1] * ax[1];
        let error = motor.target_velocity - rel_speed_along_axis;
        let max_impulse = motor.max_force * dt;
        let impulse = (error / inv_mass_sum).clamp(-max_impulse, max_impulse);

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity[0] -= impulse * ax[0] * ba.inv_mass;
            ba.linear_velocity[1] -= impulse * ax[1] * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity[0] += impulse * ax[0] * bb.inv_mass;
            bb.linear_velocity[1] += impulse * ax[1] * bb.inv_mass;
        }
    }

    fn world_anchor(&self, body: BodyHandle, local: [f64; 2]) -> [f64; 2] {
        let rb = match self.bodies.get(body_ah(body)) {
            Some(b) => b,
            None => return local,
        };
        let (sin, cos) = rb.rotation.sin_cos();
        [
            rb.position[0] + cos * local[0] - sin * local[1],
            rb.position[1] + sin * local[0] + cos * local[1],
        ]
    }

    fn solve_fixed_joint(&mut self, joint: &Joint2d) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];
        let force = (diff[0] * diff[0] + diff[1] * diff[1]).sqrt();

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position[0] += diff[0] * 0.5;
            ba.position[1] += diff[1] * 0.5;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position[0] -= diff[0] * 0.5;
            bb.position[1] -= diff[1] * 0.5;
        }
        force
    }

    fn solve_distance_joint(&mut self, joint: &Joint2d, length: f64) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];
        let dist_sq = diff[0] * diff[0] + diff[1] * diff[1];

        if dist_sq < EPSILON_SQ {
            return 0.0;
        }
        let dist = dist_sq.sqrt();
        let force = (dist - length).abs();

        let n = [diff[0] / dist, diff[1] / dist];
        let correction = (dist - length) * 0.5;

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position[0] += n[0] * correction;
            ba.position[1] += n[1] * correction;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position[0] -= n[0] * correction;
            bb.position[1] -= n[1] * correction;
        }
        force
    }

    fn solve_spring_joint(
        &mut self,
        joint: &Joint2d,
        rest_length: f64,
        stiffness: f64,
        damping: f64,
        dt: f64,
    ) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];
        let dist_sq = diff[0] * diff[0] + diff[1] * diff[1];

        if dist_sq < EPSILON_SQ {
            return 0.0;
        }
        let dist = dist_sq.sqrt();

        let n = [diff[0] / dist, diff[1] / dist];
        let spring_force = stiffness * (dist - rest_length);

        let vel_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let vel_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let rel_vel = [vel_b[0] - vel_a[0], vel_b[1] - vel_a[1]];
        let damping_force = damping * (rel_vel[0] * n[0] + rel_vel[1] * n[1]);

        let total_force = spring_force + damping_force;
        let force = [total_force * n[0] * dt, total_force * n[1] * dt];

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity[0] += force[0] * ba.inv_mass;
            ba.linear_velocity[1] += force[1] * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity[0] -= force[0] * bb.inv_mass;
            bb.linear_velocity[1] -= force[1] * bb.inv_mass;
        }
        total_force.abs()
    }

    fn solve_revolute_joint(&mut self, joint: &Joint2d, limits: Option<&[f64; 2]>) -> f64 {
        let force = self.solve_fixed_joint(joint);

        if let Some([lo, hi]) = limits {
            let rot_a = self
                .bodies
                .get(body_ah(joint.body_a))
                .map(|b| b.rotation)
                .unwrap_or(0.0);
            let rot_b = self
                .bodies
                .get(body_ah(joint.body_b))
                .map(|b| b.rotation)
                .unwrap_or(0.0);
            let rel_angle = rot_b - rot_a;

            if rel_angle < *lo {
                let c = (lo - rel_angle) * 0.5;
                if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
                    && ba.is_dynamic()
                {
                    ba.rotation -= c;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
                    && bb.is_dynamic()
                {
                    bb.rotation += c;
                }
            } else if rel_angle > *hi {
                let c = (rel_angle - hi) * 0.5;
                if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
                    && ba.is_dynamic()
                {
                    ba.rotation += c;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
                    && bb.is_dynamic()
                {
                    bb.rotation -= c;
                }
            }
        }
        force
    }

    fn solve_prismatic_joint(
        &mut self,
        joint: &Joint2d,
        axis: [f64; 2],
        limits: Option<&[f64; 2]>,
    ) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];

        let axis_len = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
        if axis_len < EPSILON {
            return 0.0;
        }
        let ax = [axis[0] / axis_len, axis[1] / axis_len];
        let perp = [-ax[1], ax[0]];
        let perp_error = diff[0] * perp[0] + diff[1] * perp[1];
        let correction = perp_error * 0.5;

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position[0] += perp[0] * correction;
            ba.position[1] += perp[1] * correction;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position[0] -= perp[0] * correction;
            bb.position[1] -= perp[1] * correction;
        }

        if let Some([lo, hi]) = limits {
            let along = diff[0] * ax[0] + diff[1] * ax[1];
            if along < *lo {
                let c = (lo - along) * 0.5;
                if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
                    && ba.is_dynamic()
                {
                    ba.position[0] -= ax[0] * c;
                    ba.position[1] -= ax[1] * c;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
                    && bb.is_dynamic()
                {
                    bb.position[0] += ax[0] * c;
                    bb.position[1] += ax[1] * c;
                }
            } else if along > *hi {
                let c = (along - hi) * 0.5;
                if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
                    && ba.is_dynamic()
                {
                    ba.position[0] += ax[0] * c;
                    ba.position[1] += ax[1] * c;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
                    && bb.is_dynamic()
                {
                    bb.position[0] -= ax[0] * c;
                    bb.position[1] -= ax[1] * c;
                }
            }
        }
        perp_error.abs()
    }

    /// Wheel joint — constrains perpendicular to axis (like prismatic), applies
    /// spring force along axis, allows free rotation.
    fn solve_wheel_joint(
        &mut self,
        joint: &Joint2d,
        axis: [f64; 2],
        stiffness: f64,
        damping: f64,
        dt: f64,
    ) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];

        let axis_len = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
        if axis_len < EPSILON {
            return 0.0;
        }
        let ax = [axis[0] / axis_len, axis[1] / axis_len];
        let perp = [-ax[1], ax[0]];

        // 1. Constrain perpendicular movement (like prismatic)
        let perp_error = diff[0] * perp[0] + diff[1] * perp[1];
        let correction = perp_error * 0.5;

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position[0] += perp[0] * correction;
            ba.position[1] += perp[1] * correction;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position[0] -= perp[0] * correction;
            bb.position[1] -= perp[1] * correction;
        }

        // 2. Spring force along the axis (suspension)
        let along = diff[0] * ax[0] + diff[1] * ax[1];
        let spring_force = stiffness * along;

        let vel_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let vel_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let rel_vel = [vel_b[0] - vel_a[0], vel_b[1] - vel_a[1]];
        let rel_vel_along = rel_vel[0] * ax[0] + rel_vel[1] * ax[1];
        let damping_force = damping * rel_vel_along;

        let total_force = spring_force + damping_force;
        let force = [total_force * ax[0] * dt, total_force * ax[1] * dt];

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity[0] += force[0] * ba.inv_mass;
            ba.linear_velocity[1] += force[1] * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity[0] -= force[0] * bb.inv_mass;
            bb.linear_velocity[1] -= force[1] * bb.inv_mass;
        }

        // No angular constraint — free rotation
        total_force.abs()
    }

    /// Rope joint — inequality constraint, only prevents exceeding max distance.
    fn solve_rope_joint(&mut self, joint: &Joint2d, max_length: f64) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];
        let dist_sq = diff[0] * diff[0] + diff[1] * diff[1];

        if dist_sq < EPSILON_SQ {
            return 0.0;
        }
        let dist = dist_sq.sqrt();

        // Inequality: only act when distance exceeds max_length
        if dist <= max_length {
            return 0.0;
        }

        let n = [diff[0] / dist, diff[1] / dist];
        let correction = (dist - max_length) * 0.5;

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position[0] += n[0] * correction;
            ba.position[1] += n[1] * correction;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position[0] -= n[0] * correction;
            bb.position[1] -= n[1] * correction;
        }
        dist - max_length
    }

    /// Mouse joint — drags body_a toward a world-space target with clamped spring force.
    fn solve_mouse_joint(
        &mut self,
        joint: &Joint2d,
        target: [f64; 3],
        stiffness: f64,
        damping: f64,
        max_force: f64,
        dt: f64,
    ) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let target_2d = [target[0], target[1]];
        let diff = [target_2d[0] - anchor_a[0], target_2d[1] - anchor_a[1]];
        let dist_sq = diff[0] * diff[0] + diff[1] * diff[1];

        if dist_sq < EPSILON_SQ {
            return 0.0;
        }

        // Spring force toward target
        let spring_force = [stiffness * diff[0], stiffness * diff[1]];

        // Damping force
        let vel_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let damping_force = [-damping * vel_a[0], -damping * vel_a[1]];

        let mut total = [
            (spring_force[0] + damping_force[0]) * dt,
            (spring_force[1] + damping_force[1]) * dt,
        ];

        // Clamp by max_force
        let force_mag = (total[0] * total[0] + total[1] * total[1]).sqrt();
        let max_impulse = max_force * dt;
        if force_mag > max_impulse && force_mag > EPSILON {
            let scale = max_impulse / force_mag;
            total[0] *= scale;
            total[1] *= scale;
        }

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity[0] += total[0] * ba.inv_mass;
            ba.linear_velocity[1] += total[1] * ba.inv_mass;
        }

        let applied = (total[0] * total[0] + total[1] * total[1]).sqrt();
        if dt > EPSILON { applied / dt } else { 0.0 }
    }

    // -----------------------------------------------------------------------
    // Collision event generation
    // -----------------------------------------------------------------------

    fn generate_events(&mut self) -> Vec<CollisionEvent> {
        let mut events = Vec::new();

        let current_keys: BTreeSet<ManifoldKey> = self.manifolds.keys().copied().collect();

        for key in &current_keys {
            if !self.prev_manifold_keys.contains(key) {
                events.push(CollisionEvent::Started {
                    collider_a: key.0,
                    collider_b: key.1,
                });
            } else {
                events.push(CollisionEvent::Ongoing {
                    collider_a: key.0,
                    collider_b: key.1,
                });
            }
        }

        for key in &self.prev_manifold_keys {
            if !current_keys.contains(key) {
                events.push(CollisionEvent::Stopped {
                    collider_a: key.0,
                    collider_b: key.1,
                });
            }
        }

        self.prev_manifold_keys = current_keys;
        events
    }

    // -----------------------------------------------------------------------
    // Raycast
    // -----------------------------------------------------------------------

    pub fn raycast(
        &self,
        origin: [f64; 3],
        direction: [f64; 3],
        max_dist: f64,
    ) -> Option<RayHit> {
        let origin_2d = [origin[0], origin[1]];
        let direction_2d = [direction[0], direction[1]];
        let dir_len = (direction_2d[0] * direction_2d[0] + direction_2d[1] * direction_2d[1]).sqrt();
        if dir_len < EPSILON {
            return None;
        }
        let dir = [direction_2d[0] / dir_len, direction_2d[1] / dir_len];

        let mut best: Option<(f64, ColliderHandle, [f64; 2], [f64; 2])> = None;

        for collider in self.colliders.values() {
            let rb = match self.bodies.get(body_ah(collider.body)) {
                Some(b) => b,
                None => continue,
            };
            let pos = world_pos(rb.position, rb.rotation, collider.offset);

            let hit = match &collider.shape {
                ColliderShape::Ball { radius } => ray_circle(origin_2d, dir, pos, *radius),
                ColliderShape::Box { half_extents } => ray_aabb_2d(
                    origin_2d,
                    dir,
                    [pos[0] - half_extents[0], pos[1] - half_extents[1]],
                    [pos[0] + half_extents[0], pos[1] + half_extents[1]],
                ),
                ColliderShape::Capsule {
                    half_height,
                    radius,
                } => ray_capsule(origin_2d, dir, pos, rb.rotation, *half_height, *radius),
                _ => None,
            };

            if let Some((t, normal)) = hit
                && t >= 0.0
                && t <= max_dist
                && (best.is_none() || t < best.as_ref().unwrap().0)
            {
                let point = [origin_2d[0] + dir[0] * t, origin_2d[1] + dir[1] * t];
                best = Some((t, collider.handle, point, normal));
            }
        }

        best.map(|(distance, collider, point, normal)| RayHit {
            collider,
            point: [point[0], point[1], 0.0],
            normal: [normal[0], normal[1], 0.0],
            distance,
        })
    }

    /// Cast a ray with a collision layer filter. Only colliders whose
    /// `collision_layer` has at least one bit in common with `layer_mask`
    /// are considered.
    pub fn raycast_filtered(
        &self,
        origin: [f64; 3],
        direction: [f64; 3],
        max_dist: f64,
        layer_mask: u32,
    ) -> Option<RayHit> {
        let origin_2d = [origin[0], origin[1]];
        let direction_2d = [direction[0], direction[1]];
        let dir_len = (direction_2d[0] * direction_2d[0] + direction_2d[1] * direction_2d[1]).sqrt();
        if dir_len < EPSILON {
            return None;
        }
        let dir = [direction_2d[0] / dir_len, direction_2d[1] / dir_len];

        let mut best: Option<(f64, ColliderHandle, [f64; 2], [f64; 2])> = None;

        for collider in self.colliders.values() {
            if (collider.collision_layer & layer_mask) == 0 {
                continue;
            }
            let rb = match self.bodies.get(body_ah(collider.body)) {
                Some(b) => b,
                None => continue,
            };
            let pos = world_pos(rb.position, rb.rotation, collider.offset);

            let hit = match &collider.shape {
                ColliderShape::Ball { radius } => ray_circle(origin_2d, dir, pos, *radius),
                ColliderShape::Box { half_extents } => ray_aabb_2d(
                    origin_2d,
                    dir,
                    [pos[0] - half_extents[0], pos[1] - half_extents[1]],
                    [pos[0] + half_extents[0], pos[1] + half_extents[1]],
                ),
                ColliderShape::Capsule {
                    half_height,
                    radius,
                } => ray_capsule(origin_2d, dir, pos, rb.rotation, *half_height, *radius),
                _ => None,
            };

            if let Some((t, normal)) = hit
                && t >= 0.0
                && t <= max_dist
                && (best.is_none() || t < best.as_ref().unwrap().0)
            {
                let point = [origin_2d[0] + dir[0] * t, origin_2d[1] + dir[1] * t];
                best = Some((t, collider.handle, point, normal));
            }
        }

        best.map(|(distance, collider, point, normal)| RayHit {
            collider,
            point: [point[0], point[1], 0.0],
            normal: [normal[0], normal[1], 0.0],
            distance,
        })
    }

    // -----------------------------------------------------------------------
    // Overlap queries (brute-force)
    // -----------------------------------------------------------------------

    /// Find all colliders overlapping a sphere (circle in 2D) at the given position.
    pub fn overlap_sphere(&self, center: [f64; 3], radius: f64) -> Vec<ColliderHandle> {
        let center_2d = [center[0], center[1]];
        let sphere_aabb = Aabb2d {
            min: [center_2d[0] - radius, center_2d[1] - radius],
            max: [center_2d[0] + radius, center_2d[1] + radius],
        };

        let mut results = Vec::new();

        for collider in self.colliders.values() {
            let rb = match self.bodies.get(body_ah(collider.body)) {
                Some(b) => b,
                None => continue,
            };

            let col_aabb = collider.world_aabb(rb.position, rb.rotation);

            // Quick AABB rejection
            if !sphere_aabb.overlaps(&col_aabb) {
                continue;
            }

            // Precise sphere-vs-shape check
            let pos = world_pos(rb.position, rb.rotation, collider.offset);
            let overlaps = match &collider.shape {
                ColliderShape::Ball {
                    radius: shape_radius,
                } => {
                    let dx = center_2d[0] - pos[0];
                    let dy = center_2d[1] - pos[1];
                    let dist_sq = dx * dx + dy * dy;
                    let sum_r = radius + shape_radius;
                    dist_sq < sum_r * sum_r
                }
                ColliderShape::Box { half_extents } => {
                    let dx = center_2d[0] - pos[0];
                    let dy = center_2d[1] - pos[1];
                    let cx = dx.clamp(-half_extents[0], half_extents[0]);
                    let cy = dy.clamp(-half_extents[1], half_extents[1]);
                    let diff_x = dx - cx;
                    let diff_y = dy - cy;
                    let dist_sq = diff_x * diff_x + diff_y * diff_y;
                    dist_sq < radius * radius
                }
                ColliderShape::Capsule {
                    half_height,
                    radius: cap_radius,
                } => {
                    let (ep_a, ep_b) = capsule_endpoints(pos, rb.rotation, *half_height);
                    let (closest, _) = closest_point_on_segment(ep_a, ep_b, center_2d);
                    let dx = center_2d[0] - closest[0];
                    let dy = center_2d[1] - closest[1];
                    let dist_sq = dx * dx + dy * dy;
                    let sum_r = radius + cap_radius;
                    dist_sq < sum_r * sum_r
                }
                // For other shapes, fall back to AABB overlap (already passed)
                _ => true,
            };

            if overlaps {
                results.push(collider.handle);
            }
        }

        results
    }

    /// Find all colliders overlapping an AABB.
    pub fn overlap_aabb(&self, min: [f64; 3], max: [f64; 3]) -> Vec<ColliderHandle> {
        let query_aabb = Aabb2d {
            min: [min[0], min[1]],
            max: [max[0], max[1]],
        };

        let mut results = Vec::new();

        for collider in self.colliders.values() {
            let rb = match self.bodies.get(body_ah(collider.body)) {
                Some(b) => b,
                None => continue,
            };

            let col_aabb = collider.world_aabb(rb.position, rb.rotation);

            if query_aabb.overlaps(&col_aabb) {
                results.push(collider.handle);
            }
        }

        results
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

fn world_pos(body_pos: [f64; 2], body_rot: f64, offset: [f64; 2]) -> [f64; 2] {
    let (sin, cos) = body_rot.sin_cos();
    [
        body_pos[0] + cos * offset[0] - sin * offset[1],
        body_pos[1] + sin * offset[0] + cos * offset[1],
    ]
}

/// Transform a world-space point into body-local coordinates.
fn world_to_local(world_pt: [f64; 2], body_pos: [f64; 2], body_rot: f64) -> [f64; 2] {
    let dx = world_pt[0] - body_pos[0];
    let dy = world_pt[1] - body_pos[1];
    let (sin, cos) = body_rot.sin_cos();
    // Inverse rotation: transpose of rotation matrix
    [cos * dx + sin * dy, -sin * dx + cos * dy]
}

/// Transform a body-local point into world-space coordinates.
fn local_to_world(local_pt: [f64; 2], body_pos: [f64; 2], body_rot: f64) -> [f64; 2] {
    let (sin, cos) = body_rot.sin_cos();
    [
        body_pos[0] + cos * local_pt[0] - sin * local_pt[1],
        body_pos[1] + sin * local_pt[0] + cos * local_pt[1],
    ]
}

/// Create an ordered manifold key from a collider pair (smaller handle first).
fn ordered_manifold_key(a: ColliderHandle, b: ColliderHandle) -> ManifoldKey {
    if a.0 <= b.0 { (a, b) } else { (b, a) }
}

// ---------------------------------------------------------------------------
// Narrowphase contact generation
// ---------------------------------------------------------------------------

fn generate_contact(
    shape_a: &ColliderShape,
    pos_a: [f64; 2],
    rot_a: f64,
    shape_b: &ColliderShape,
    pos_b: [f64; 2],
    rot_b: f64,
) -> Option<([f64; 2], f64, [f64; 2])> {
    match (shape_a, shape_b) {
        // Ball vs Ball
        (ColliderShape::Ball { radius: ra }, ColliderShape::Ball { radius: rb }) => {
            circle_circle(pos_a, *ra, pos_b, *rb)
        }
        // Ball vs Box
        (ColliderShape::Ball { radius }, ColliderShape::Box { half_extents }) => {
            circle_aabb(pos_a, *radius, pos_b, [half_extents[0], half_extents[1]])
        }
        (ColliderShape::Box { half_extents }, ColliderShape::Ball { radius }) => {
            circle_aabb(pos_b, *radius, pos_a, [half_extents[0], half_extents[1]])
                .map(|(n, d, p)| ([-n[0], -n[1]], d, p))
        }
        // Box vs Box — use OBB-OBB SAT when either box is rotated, fast AABB path otherwise
        (
            ColliderShape::Box { half_extents: he_a },
            ColliderShape::Box { half_extents: he_b },
        ) => {
            if rot_a.abs() < EPSILON && rot_b.abs() < EPSILON {
                aabb_aabb_contact(pos_a, [he_a[0], he_a[1]], pos_b, [he_b[0], he_b[1]])
            } else {
                obb_obb_contact(pos_a, rot_a, [he_a[0], he_a[1]], pos_b, rot_b, [he_b[0], he_b[1]])
            }
        }
        // Capsule vs Ball
        (
            ColliderShape::Capsule {
                half_height: hh,
                radius: cr,
            },
            ColliderShape::Ball { radius: br },
        ) => capsule_circle(pos_a, rot_a, *hh, *cr, pos_b, *br),
        (
            ColliderShape::Ball { radius: br },
            ColliderShape::Capsule {
                half_height: hh,
                radius: cr,
            },
        ) => {
            capsule_circle(pos_b, rot_b, *hh, *cr, pos_a, *br)
                .map(|(n, d, p)| ([-n[0], -n[1]], d, p))
        }
        // Capsule vs Box
        (
            ColliderShape::Capsule {
                half_height: hh,
                radius: cr,
            },
            ColliderShape::Box { half_extents },
        ) => capsule_aabb(pos_a, rot_a, *hh, *cr, pos_b, [half_extents[0], half_extents[1]]),
        (
            ColliderShape::Box { half_extents },
            ColliderShape::Capsule {
                half_height: hh,
                radius: cr,
            },
        ) => {
            capsule_aabb(pos_b, rot_b, *hh, *cr, pos_a, [half_extents[0], half_extents[1]])
                .map(|(n, d, p)| ([-n[0], -n[1]], d, p))
        }
        // Capsule vs Capsule
        (
            ColliderShape::Capsule {
                half_height: hh_a,
                radius: cr_a,
            },
            ColliderShape::Capsule {
                half_height: hh_b,
                radius: cr_b,
            },
        ) => capsule_capsule(pos_a, rot_a, *hh_a, *cr_a, pos_b, rot_b, *hh_b, *cr_b),
        // ConvexHull vs Ball
        (ColliderShape::ConvexHull { points }, ColliderShape::Ball { radius }) => {
            convex_hull_circle(points, pos_a, rot_a, pos_b, *radius)
        }
        (ColliderShape::Ball { radius }, ColliderShape::ConvexHull { points }) => {
            convex_hull_circle(points, pos_b, rot_b, pos_a, *radius)
                .map(|(n, d, p)| ([-n[0], -n[1]], d, p))
        }
        // ConvexHull vs ConvexHull
        (
            ColliderShape::ConvexHull { points: pts_a },
            ColliderShape::ConvexHull { points: pts_b },
        ) => convex_convex_contact(pts_a, pos_a, rot_a, pts_b, pos_b, rot_b),
        // ConvexHull vs Box (treat box as 4-vertex convex hull)
        (ColliderShape::ConvexHull { points }, ColliderShape::Box { half_extents }) => {
            let box_pts = box_to_convex_points(*half_extents);
            convex_convex_contact(points, pos_a, rot_a, &box_pts, pos_b, rot_b)
        }
        (ColliderShape::Box { half_extents }, ColliderShape::ConvexHull { points }) => {
            let box_pts = box_to_convex_points(*half_extents);
            convex_convex_contact(&box_pts, pos_a, rot_a, points, pos_b, rot_b)
        }
        // Segment vs Ball
        (ColliderShape::Segment { a, b }, ColliderShape::Ball { radius }) => {
            segment_circle(pos_a, rot_a, *a, *b, pos_b, *radius)
        }
        (ColliderShape::Ball { radius }, ColliderShape::Segment { a, b }) => {
            segment_circle(pos_b, rot_b, *a, *b, pos_a, *radius)
                .map(|(n, d, p)| ([-n[0], -n[1]], d, p))
        }
        // Segment vs Box
        (ColliderShape::Segment { a, b }, ColliderShape::Box { half_extents }) => {
            segment_box(pos_a, rot_a, *a, *b, pos_b, rot_b, [half_extents[0], half_extents[1]])
        }
        (ColliderShape::Box { half_extents }, ColliderShape::Segment { a, b }) => {
            segment_box(pos_b, rot_b, *a, *b, pos_a, rot_a, [half_extents[0], half_extents[1]])
                .map(|(n, d, p)| ([-n[0], -n[1]], d, p))
        }
        _ => None,
    }
}

/// ConvexHull vs Circle contact.
/// `hull_points` are in the hull's local space. `hull_pos`/`hull_rot` transform them to world.
/// Returns (normal_from_hull_to_circle, depth, contact_point).
fn convex_hull_circle(
    hull_points: &[[f64; 3]],
    hull_pos: [f64; 2],
    hull_rot: f64,
    circle_pos: [f64; 2],
    radius: f64,
) -> Option<([f64; 2], f64, [f64; 2])> {
    if hull_points.len() < 2 {
        return None;
    }

    let (sin, cos) = hull_rot.sin_cos();

    // Transform hull points to world space (2D)
    let world_pts: Vec<[f64; 2]> = hull_points
        .iter()
        .map(|p| {
            [
                hull_pos[0] + cos * p[0] - sin * p[1],
                hull_pos[1] + sin * p[0] + cos * p[1],
            ]
        })
        .collect();

    // Find closest point on hull perimeter to circle center
    let n = world_pts.len();
    let mut best_dist_sq = f64::INFINITY;
    let mut best_closest = world_pts[0];

    for i in 0..n {
        let a = world_pts[i];
        let b = world_pts[(i + 1) % n];
        let (closest, _) = closest_point_on_segment(a, b, circle_pos);
        let dx = circle_pos[0] - closest[0];
        let dy = circle_pos[1] - closest[1];
        let dist_sq = dx * dx + dy * dy;
        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_closest = closest;
        }
    }

    if best_dist_sq >= radius * radius {
        return None;
    }

    let dx = circle_pos[0] - best_closest[0];
    let dy = circle_pos[1] - best_closest[1];
    let dist = best_dist_sq.sqrt();

    let (normal, depth) = if dist < EPSILON {
        // Circle center is on the hull edge; use edge normal
        // Find the edge that gave the closest point and compute its outward normal
        ([0.0, 1.0], radius)
    } else {
        ([dx / dist, dy / dist], radius - dist)
    };

    Some((normal, depth, best_closest))
}

fn circle_circle(
    pos_a: [f64; 2],
    ra: f64,
    pos_b: [f64; 2],
    rb: f64,
) -> Option<([f64; 2], f64, [f64; 2])> {
    let dx = pos_b[0] - pos_a[0];
    let dy = pos_b[1] - pos_a[1];
    let dist_sq = dx * dx + dy * dy;
    let sum_r = ra + rb;

    if dist_sq >= sum_r * sum_r {
        return None;
    }

    let dist = dist_sq.sqrt();
    let (normal, depth) = if dist < EPSILON {
        ([0.0, 1.0], sum_r)
    } else {
        ([dx / dist, dy / dist], sum_r - dist)
    };

    let point = [pos_a[0] + normal[0] * ra, pos_a[1] + normal[1] * ra];
    Some((normal, depth, point))
}

fn circle_aabb(
    circle_pos: [f64; 2],
    radius: f64,
    box_pos: [f64; 2],
    half_extents: [f64; 2],
) -> Option<([f64; 2], f64, [f64; 2])> {
    let dx = circle_pos[0] - box_pos[0];
    let dy = circle_pos[1] - box_pos[1];

    let closest_x = dx.clamp(-half_extents[0], half_extents[0]);
    let closest_y = dy.clamp(-half_extents[1], half_extents[1]);

    let diff_x = dx - closest_x;
    let diff_y = dy - closest_y;
    let dist_sq = diff_x * diff_x + diff_y * diff_y;

    if dist_sq >= radius * radius {
        return None;
    }

    let dist = dist_sq.sqrt();
    let (normal, depth) = if dist < EPSILON {
        let face_dists = [half_extents[0] - dx.abs(), half_extents[1] - dy.abs()];
        if face_dists[0] < face_dists[1] {
            let sign = if dx >= 0.0 { 1.0 } else { -1.0 };
            ([sign, 0.0], face_dists[0] + radius)
        } else {
            let sign = if dy >= 0.0 { 1.0 } else { -1.0 };
            ([0.0, sign], face_dists[1] + radius)
        }
    } else {
        ([diff_x / dist, diff_y / dist], radius - dist)
    };

    let point = [box_pos[0] + closest_x, box_pos[1] + closest_y];
    Some((normal, depth, point))
}

fn aabb_aabb_contact(
    pos_a: [f64; 2],
    he_a: [f64; 2],
    pos_b: [f64; 2],
    he_b: [f64; 2],
) -> Option<([f64; 2], f64, [f64; 2])> {
    let dx = pos_b[0] - pos_a[0];
    let dy = pos_b[1] - pos_a[1];

    let overlap_x = he_a[0] + he_b[0] - dx.abs();
    let overlap_y = he_a[1] + he_b[1] - dy.abs();

    if overlap_x <= 0.0 || overlap_y <= 0.0 {
        return None;
    }

    let (normal, depth) = if overlap_x < overlap_y {
        let sign = if dx >= 0.0 { 1.0 } else { -1.0 };
        ([sign, 0.0], overlap_x)
    } else {
        let sign = if dy >= 0.0 { 1.0 } else { -1.0 };
        ([0.0, sign], overlap_y)
    };

    let point = [
        pos_a[0] + normal[0] * he_a[0],
        pos_a[1] + normal[1] * he_a[1],
    ];
    Some((normal, depth, point))
}

// ---------------------------------------------------------------------------
// OBB-OBB contact (Separating Axis Theorem for 2D oriented bounding boxes)
// ---------------------------------------------------------------------------

/// OBB-OBB contact using 2D SAT with 4 separating axes (2 edge normals per box).
/// Returns (normal_from_a_to_b, penetration_depth, contact_point).
#[allow(clippy::too_many_arguments)]
fn obb_obb_contact(
    pos_a: [f64; 2],
    rot_a: f64,
    he_a: [f64; 2],
    pos_b: [f64; 2],
    rot_b: f64,
    he_b: [f64; 2],
) -> Option<([f64; 2], f64, [f64; 2])> {
    let (sin_a, cos_a) = rot_a.sin_cos();
    let (sin_b, cos_b) = rot_b.sin_cos();

    // Local axes for each box
    let axes_a = [[cos_a, sin_a], [-sin_a, cos_a]];
    let axes_b = [[cos_b, sin_b], [-sin_b, cos_b]];

    // Centre-to-centre vector
    let d = [pos_b[0] - pos_a[0], pos_b[1] - pos_a[1]];

    let mut min_overlap = f64::INFINITY;
    let mut best_axis = [0.0f64; 2];

    // Test all 4 axes (2 per box)
    let all_axes = [axes_a[0], axes_a[1], axes_b[0], axes_b[1]];

    for axis in &all_axes {
        // Project half-extents of A onto axis
        let proj_a = he_a[0] * (axes_a[0][0] * axis[0] + axes_a[0][1] * axis[1]).abs()
            + he_a[1] * (axes_a[1][0] * axis[0] + axes_a[1][1] * axis[1]).abs();
        // Project half-extents of B onto axis
        let proj_b = he_b[0] * (axes_b[0][0] * axis[0] + axes_b[0][1] * axis[1]).abs()
            + he_b[1] * (axes_b[1][0] * axis[0] + axes_b[1][1] * axis[1]).abs();
        // Distance between centres along this axis
        let dist = d[0] * axis[0] + d[1] * axis[1];

        let overlap = proj_a + proj_b - dist.abs();
        if overlap <= 0.0 {
            return None; // Separating axis found
        }

        if overlap < min_overlap {
            min_overlap = overlap;
            // Normal should point from A to B
            if dist >= 0.0 {
                best_axis = *axis;
            } else {
                best_axis = [-axis[0], -axis[1]];
            }
        }
    }

    // Contact point: midpoint of support points on each box's surface toward the other.
    let cp_a = obb_support_point(pos_a, he_a, &axes_a, best_axis);
    let cp_b = obb_support_point(pos_b, he_b, &axes_b, [-best_axis[0], -best_axis[1]]);
    let point = [(cp_a[0] + cp_b[0]) * 0.5, (cp_a[1] + cp_b[1]) * 0.5];

    Some((best_axis, min_overlap, point))
}

/// Compute the support point of an OBB in a given direction.
/// Returns the corner of the box that is furthest in `dir`.
fn obb_support_point(
    center: [f64; 2],
    half_extents: [f64; 2],
    axes: &[[f64; 2]; 2],
    dir: [f64; 2],
) -> [f64; 2] {
    let mut point = center;
    for i in 0..2 {
        let dot = axes[i][0] * dir[0] + axes[i][1] * dir[1];
        let sign = if dot >= 0.0 { 1.0 } else { -1.0 };
        point[0] += sign * half_extents[i] * axes[i][0];
        point[1] += sign * half_extents[i] * axes[i][1];
    }
    point
}

// ---------------------------------------------------------------------------
// Capsule helpers
// ---------------------------------------------------------------------------

/// Capsule endpoints in world space. A 2D capsule is a segment with radius.
fn capsule_endpoints(pos: [f64; 2], rot: f64, half_height: f64) -> ([f64; 2], [f64; 2]) {
    let (sin, cos) = rot.sin_cos();
    // Capsule axis is along local Y
    let dx = -sin * half_height;
    let dy = cos * half_height;
    (
        [pos[0] - dx, pos[1] - dy],
        [pos[0] + dx, pos[1] + dy],
    )
}

/// Closest point on segment (a, b) to point p. Returns (closest_point, t_parameter).
fn closest_point_on_segment(a: [f64; 2], b: [f64; 2], p: [f64; 2]) -> ([f64; 2], f64) {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let len_sq = ab[0] * ab[0] + ab[1] * ab[1];
    if len_sq < EPSILON_SQ {
        return (a, 0.0);
    }
    let t = ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / len_sq;
    let t = t.clamp(0.0, 1.0);
    ([a[0] + ab[0] * t, a[1] + ab[1] * t], t)
}

/// Closest points between two segments. Returns (point_on_ab, point_on_cd).
/// Closest points between two segments.
fn closest_points_segments(
    a: [f64; 2],
    b: [f64; 2],
    c: [f64; 2],
    d: [f64; 2],
) -> ([f64; 2], [f64; 2]) {
    fn dist_sq(p: [f64; 2], q: [f64; 2]) -> f64 {
        (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2)
    }

    let ab = [b[0] - a[0], b[1] - a[1]];
    let cd = [d[0] - c[0], d[1] - c[1]];

    let d1 = ab[0] * ab[0] + ab[1] * ab[1];
    let d2 = cd[0] * cd[0] + cd[1] * cd[1];

    // Start with 4 endpoint-to-segment projections
    let (pa, _) = closest_point_on_segment(c, d, a);
    let (pb, _) = closest_point_on_segment(c, d, b);
    let (pc, _) = closest_point_on_segment(a, b, c);
    let (pd, _) = closest_point_on_segment(a, b, d);

    let mut best_p1 = a;
    let mut best_p2 = pa;
    let mut best_d = dist_sq(a, pa);

    for (p1, p2) in [(b, pb), (pc, c), (pd, d)] {
        let dd = dist_sq(p1, p2);
        if dd < best_d {
            best_p1 = p1;
            best_p2 = p2;
            best_d = dd;
        }
    }

    // Also try the analytical closest pair with iterative clamping.
    // Uses r = a - c (not c - a) so the formula signs are standard:
    //   s = (d4*d2 + d5*d3) / denom, t = (d3*s - d5) / d2
    if d1 > EPSILON_SQ && d2 > EPSILON_SQ {
        let r = [a[0] - c[0], a[1] - c[1]];
        let d3 = ab[0] * cd[0] + ab[1] * cd[1]; // AB·CD
        let d4 = ab[0] * r[0] + ab[1] * r[1]; // AB·r
        let d5 = cd[0] * r[0] + cd[1] * r[1]; // CD·r
        let denom = d1 * d2 - d3 * d3;

        if denom.abs() > EPSILON_SQ {
            let mut s = ((d3 * d5 - d4 * d2) / denom).clamp(0.0, 1.0);
            let mut t = ((d3 * s + d5) / d2).clamp(0.0, 1.0);
            s = ((t * d3 - d4) / d1).clamp(0.0, 1.0);
            t = ((d3 * s + d5) / d2).clamp(0.0, 1.0);

            let p1 = [a[0] + ab[0] * s, a[1] + ab[1] * s];
            let p2 = [c[0] + cd[0] * t, c[1] + cd[1] * t];
            let dd = dist_sq(p1, p2);
            if dd < best_d {
                best_p1 = p1;
                best_p2 = p2;
            }
        }
    }

    (best_p1, best_p2)
}

/// Capsule vs circle contact.
fn capsule_circle(
    cap_pos: [f64; 2],
    cap_rot: f64,
    half_height: f64,
    cap_radius: f64,
    circle_pos: [f64; 2],
    circle_radius: f64,
) -> Option<([f64; 2], f64, [f64; 2])> {
    let (ep_a, ep_b) = capsule_endpoints(cap_pos, cap_rot, half_height);
    let (closest, _) = closest_point_on_segment(ep_a, ep_b, circle_pos);
    // Now it's a circle-circle test between closest point (with cap_radius) and the ball
    circle_circle(closest, cap_radius, circle_pos, circle_radius)
}

/// Capsule vs AABB contact. Treats capsule as circle at closest segment point to box.
fn capsule_aabb(
    cap_pos: [f64; 2],
    cap_rot: f64,
    half_height: f64,
    cap_radius: f64,
    box_pos: [f64; 2],
    half_extents: [f64; 2],
) -> Option<([f64; 2], f64, [f64; 2])> {
    let (ep_a, ep_b) = capsule_endpoints(cap_pos, cap_rot, half_height);

    // Find closest point on capsule segment to box center, then test as circle vs AABB
    // For better accuracy, test both endpoints and midpoint, take deepest
    let candidates = [ep_a, ep_b, cap_pos];
    let mut best: Option<([f64; 2], f64, [f64; 2])> = None;

    for &pt in &candidates {
        if let Some((n, d, p)) = circle_aabb(pt, cap_radius, box_pos, half_extents)
            && (best.is_none() || d > best.as_ref().unwrap().1)
        {
            best = Some((n, d, p));
        }
    }

    // Also test closest point on segment to box center
    let (closest, _) = closest_point_on_segment(ep_a, ep_b, box_pos);
    if let Some((n, d, p)) = circle_aabb(closest, cap_radius, box_pos, half_extents)
        && (best.is_none() || d > best.as_ref().unwrap().1)
    {
        best = Some((n, d, p));
    }

    best
}

/// Capsule vs capsule contact.
#[allow(clippy::too_many_arguments)]
fn capsule_capsule(
    pos_a: [f64; 2],
    rot_a: f64,
    hh_a: f64,
    r_a: f64,
    pos_b: [f64; 2],
    rot_b: f64,
    hh_b: f64,
    r_b: f64,
) -> Option<([f64; 2], f64, [f64; 2])> {
    let (a1, a2) = capsule_endpoints(pos_a, rot_a, hh_a);
    let (b1, b2) = capsule_endpoints(pos_b, rot_b, hh_b);
    let (cp_a, cp_b) = closest_points_segments(a1, a2, b1, b2);
    circle_circle(cp_a, r_a, cp_b, r_b)
}

// ---------------------------------------------------------------------------
// Ray intersection helpers
// ---------------------------------------------------------------------------

fn ray_circle(
    origin: [f64; 2],
    dir: [f64; 2],
    center: [f64; 2],
    radius: f64,
) -> Option<(f64, [f64; 2])> {
    let oc = [origin[0] - center[0], origin[1] - center[1]];
    let half_b = oc[0] * dir[0] + oc[1] * dir[1];
    let c = oc[0] * oc[0] + oc[1] * oc[1] - radius * radius;
    let discriminant = half_b * half_b - c;

    if discriminant < 0.0 {
        return None;
    }

    let sqrt_d = discriminant.sqrt();
    let t1 = -half_b - sqrt_d;
    let t2 = -half_b + sqrt_d;

    let t = if t1 >= 0.0 {
        t1
    } else if t2 >= 0.0 {
        t2
    } else {
        return None;
    };

    let point = [origin[0] + dir[0] * t, origin[1] + dir[1] * t];
    let nl = ((point[0] - center[0]).powi(2) + (point[1] - center[1]).powi(2)).sqrt();
    let normal = if nl > EPSILON {
        [(point[0] - center[0]) / nl, (point[1] - center[1]) / nl]
    } else {
        [0.0, 1.0]
    };

    Some((t, normal))
}

fn ray_aabb_2d(
    origin: [f64; 2],
    dir: [f64; 2],
    min: [f64; 2],
    max: [f64; 2],
) -> Option<(f64, [f64; 2])> {
    let mut t_min = f64::NEG_INFINITY;
    let mut t_max = f64::INFINITY;
    let mut normal = [0.0, 0.0];

    for i in 0..2 {
        if dir[i].abs() < EPSILON {
            if origin[i] < min[i] || origin[i] > max[i] {
                return None;
            }
        } else {
            let inv_d = 1.0 / dir[i];
            let mut t1 = (min[i] - origin[i]) * inv_d;
            let mut t2 = (max[i] - origin[i]) * inv_d;
            let mut n = if i == 0 { [-1.0, 0.0] } else { [0.0, -1.0] };
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
                n[i] = -n[i];
            }
            if t1 > t_min {
                t_min = t1;
                normal = n;
            }
            t_max = t_max.min(t2);
            if t_min > t_max {
                return None;
            }
        }
    }

    let t = if t_min >= 0.0 {
        t_min
    } else if t_max >= 0.0 {
        t_max
    } else {
        return None;
    };

    Some((t, normal))
}

fn ray_capsule(
    origin: [f64; 2],
    dir: [f64; 2],
    cap_pos: [f64; 2],
    cap_rot: f64,
    half_height: f64,
    radius: f64,
) -> Option<(f64, [f64; 2])> {
    let (ep_a, ep_b) = capsule_endpoints(cap_pos, cap_rot, half_height);

    // Test ray against circles at both endpoints and pick closest hit
    let hit_a = ray_circle(origin, dir, ep_a, radius);
    let hit_b = ray_circle(origin, dir, ep_b, radius);

    let mut best = hit_a;
    if let Some((tb, nb)) = hit_b
        && (best.is_none() || tb < best.unwrap().0)
    {
        best = Some((tb, nb));
    }

    // Test ray against the rectangle between endpoints (the shaft)
    // Project onto capsule axis and check if ray hits the swept region
    let axis = [ep_b[0] - ep_a[0], ep_b[1] - ep_a[1]];
    let axis_len = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
    if axis_len > EPSILON {
        let ax = [axis[0] / axis_len, axis[1] / axis_len];
        let perp = [-ax[1], ax[0]];

        // The shaft is an AABB in capsule-local space: along axis [-hh, hh], perpendicular [-r, r]
        // Transform ray to capsule-local coordinates
        let local_ox = (origin[0] - cap_pos[0]) * ax[0] + (origin[1] - cap_pos[1]) * ax[1];
        let local_oy = (origin[0] - cap_pos[0]) * perp[0] + (origin[1] - cap_pos[1]) * perp[1];
        let local_dx = dir[0] * ax[0] + dir[1] * ax[1];
        let local_dy = dir[0] * perp[0] + dir[1] * perp[1];

        if let Some((t, local_n)) = ray_aabb_2d(
            [local_ox, local_oy],
            [local_dx, local_dy],
            [-half_height, -radius],
            [half_height, radius],
        ) {
            // Transform normal back to world space
            let world_n = [
                local_n[0] * ax[0] + local_n[1] * perp[0],
                local_n[0] * ax[1] + local_n[1] * perp[1],
            ];
            if best.is_none() || t < best.unwrap().0 {
                best = Some((t, world_n));
            }
        }
    }

    best
}

// ---------------------------------------------------------------------------
// ConvexHull-vs-ConvexHull (2D SAT)
// ---------------------------------------------------------------------------

/// Convert a box (half_extents) into 4 convex hull points in local space (CCW).
fn box_to_convex_points(half_extents: [f64; 3]) -> Vec<[f64; 3]> {
    let hx = half_extents[0];
    let hy = half_extents[1];
    vec![
        [-hx, -hy, 0.0],
        [hx, -hy, 0.0],
        [hx, hy, 0.0],
        [-hx, hy, 0.0],
    ]
}

/// Transform hull points from local to world 2D.
fn transform_hull(points: &[[f64; 3]], pos: [f64; 2], rot: f64) -> Vec<[f64; 2]> {
    let (sin, cos) = rot.sin_cos();
    points
        .iter()
        .map(|p| {
            [
                pos[0] + cos * p[0] - sin * p[1],
                pos[1] + sin * p[0] + cos * p[1],
            ]
        })
        .collect()
}

/// 2D SAT (Separating Axis Theorem) for two convex polygons.
/// Returns (normal_from_a_to_b, penetration_depth, contact_point).
fn convex_convex_contact(
    points_a: &[[f64; 3]],
    pos_a: [f64; 2],
    rot_a: f64,
    points_b: &[[f64; 3]],
    pos_b: [f64; 2],
    rot_b: f64,
) -> Option<([f64; 2], f64, [f64; 2])> {
    let world_a = transform_hull(points_a, pos_a, rot_a);
    let world_b = transform_hull(points_b, pos_b, rot_b);

    if world_a.len() < 2 || world_b.len() < 2 {
        return None;
    }

    let mut min_overlap = f64::INFINITY;
    let mut best_axis = [0.0f64; 2];

    // Centre-to-centre direction for consistent normal orientation
    let center_d = [pos_b[0] - pos_a[0], pos_b[1] - pos_a[1]];

    // Test edge normals from both polygons
    for hull in [&world_a, &world_b] {
        let n = hull.len();
        for i in 0..n {
            let j = (i + 1) % n;
            let edge = [hull[j][0] - hull[i][0], hull[j][1] - hull[i][1]];
            let len = (edge[0] * edge[0] + edge[1] * edge[1]).sqrt();
            if len < EPSILON {
                continue;
            }
            // Outward normal (perpendicular to edge)
            let axis = [-edge[1] / len, edge[0] / len];

            // Project both hulls onto this axis
            let (min_a, max_a) = project_hull(&world_a, axis);
            let (min_b, max_b) = project_hull(&world_b, axis);

            let overlap = (max_a.min(max_b)) - (min_a.max(min_b));
            if overlap <= 0.0 {
                return None; // Separating axis found
            }

            if overlap < min_overlap {
                min_overlap = overlap;
                // Ensure normal points from A to B
                let dot = center_d[0] * axis[0] + center_d[1] * axis[1];
                if dot >= 0.0 {
                    best_axis = axis;
                } else {
                    best_axis = [-axis[0], -axis[1]];
                }
            }
        }
    }

    // Contact point: average of support points
    let cp_a = support_point_poly(&world_a, best_axis);
    let cp_b = support_point_poly(&world_b, [-best_axis[0], -best_axis[1]]);
    let point = [(cp_a[0] + cp_b[0]) * 0.5, (cp_a[1] + cp_b[1]) * 0.5];

    Some((best_axis, min_overlap, point))
}

/// Project all points of a 2D polygon onto an axis, return (min, max).
fn project_hull(points: &[[f64; 2]], axis: [f64; 2]) -> (f64, f64) {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for p in points {
        let proj = p[0] * axis[0] + p[1] * axis[1];
        min = min.min(proj);
        max = max.max(proj);
    }
    (min, max)
}

/// Find the vertex of a 2D polygon furthest in a given direction.
fn support_point_poly(points: &[[f64; 2]], dir: [f64; 2]) -> [f64; 2] {
    let mut best = points[0];
    let mut best_dot = best[0] * dir[0] + best[1] * dir[1];
    for p in &points[1..] {
        let d = p[0] * dir[0] + p[1] * dir[1];
        if d > best_dot {
            best_dot = d;
            best = *p;
        }
    }
    best
}

// ---------------------------------------------------------------------------
// Segment narrowphase contacts
// ---------------------------------------------------------------------------

/// Segment-vs-Ball: find closest point on segment to ball center, test distance vs radius.
/// The segment endpoints are in the segment body's local space.
fn segment_circle(
    seg_pos: [f64; 2],
    seg_rot: f64,
    seg_a: [f64; 3],
    seg_b: [f64; 3],
    circle_pos: [f64; 2],
    circle_radius: f64,
) -> Option<([f64; 2], f64, [f64; 2])> {
    let (sin, cos) = seg_rot.sin_cos();
    let wa = [
        seg_pos[0] + cos * seg_a[0] - sin * seg_a[1],
        seg_pos[1] + sin * seg_a[0] + cos * seg_a[1],
    ];
    let wb = [
        seg_pos[0] + cos * seg_b[0] - sin * seg_b[1],
        seg_pos[1] + sin * seg_b[0] + cos * seg_b[1],
    ];
    let (closest, _) = closest_point_on_segment(wa, wb, circle_pos);
    // Segment has zero radius, so it's like a capsule_circle with r=0
    let dx = circle_pos[0] - closest[0];
    let dy = circle_pos[1] - closest[1];
    let dist_sq = dx * dx + dy * dy;

    if dist_sq >= circle_radius * circle_radius {
        return None;
    }

    let dist = dist_sq.sqrt();
    let (normal, depth) = if dist < EPSILON {
        // Degenerate: circle center is on the segment
        ([0.0, 1.0], circle_radius)
    } else {
        ([dx / dist, dy / dist], circle_radius - dist)
    };

    Some((normal, depth, closest))
}

/// Segment-vs-Box: find overlap between a line segment and an axis-aligned box.
#[allow(clippy::too_many_arguments)]
fn segment_box(
    seg_pos: [f64; 2],
    seg_rot: f64,
    seg_a: [f64; 3],
    seg_b: [f64; 3],
    box_pos: [f64; 2],
    _box_rot: f64,
    half_extents: [f64; 2],
) -> Option<([f64; 2], f64, [f64; 2])> {
    // Transform segment to world space
    let (sin, cos) = seg_rot.sin_cos();
    let wa = [
        seg_pos[0] + cos * seg_a[0] - sin * seg_a[1],
        seg_pos[1] + sin * seg_a[0] + cos * seg_a[1],
    ];
    let wb = [
        seg_pos[0] + cos * seg_b[0] - sin * seg_b[1],
        seg_pos[1] + sin * seg_b[0] + cos * seg_b[1],
    ];

    // Find closest point on segment to box center
    let (closest_on_seg, _) = closest_point_on_segment(wa, wb, box_pos);

    // Clamp that point to box bounds to find closest point on box
    let dx = closest_on_seg[0] - box_pos[0];
    let dy = closest_on_seg[1] - box_pos[1];
    let cx = dx.clamp(-half_extents[0], half_extents[0]);
    let cy = dy.clamp(-half_extents[1], half_extents[1]);
    let box_closest = [box_pos[0] + cx, box_pos[1] + cy];

    // Now find closest point on segment to that box point
    let (seg_closest, _) = closest_point_on_segment(wa, wb, box_closest);

    let diff_x = seg_closest[0] - box_closest[0];
    let diff_y = seg_closest[1] - box_closest[1];
    let dist_sq = diff_x * diff_x + diff_y * diff_y;

    // Segment has zero radius — we need the segment point to be inside the box
    // or touching it. Check if segment point is inside the box.
    let rel_x = seg_closest[0] - box_pos[0];
    let rel_y = seg_closest[1] - box_pos[1];

    if rel_x.abs() <= half_extents[0] && rel_y.abs() <= half_extents[1] {
        // Segment point is inside the box — push out along minimum penetration axis
        let pen_x = half_extents[0] - rel_x.abs();
        let pen_y = half_extents[1] - rel_y.abs();
        if pen_x < pen_y {
            let sign = if rel_x >= 0.0 { 1.0 } else { -1.0 };
            Some(([sign, 0.0], pen_x, seg_closest))
        } else {
            let sign = if rel_y >= 0.0 { 1.0 } else { -1.0 };
            Some(([0.0, sign], pen_y, seg_closest))
        }
    } else if dist_sq < EPSILON_SQ {
        // Touching but not inside — very shallow contact
        None
    } else {
        // Not overlapping
        None
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-6;

    // -- Narrowphase contact tests --

    #[test]
    fn circle_circle_overlap() {
        let r = circle_circle([0.0, 0.0], 1.0, [1.5, 0.0], 1.0);
        assert!(r.is_some());
        let (n, d, _p) = r.unwrap();
        assert!((n[0] - 1.0).abs() < EPS);
        assert!(n[1].abs() < EPS);
        assert!((d - 0.5).abs() < EPS);
    }

    #[test]
    fn circle_circle_no_overlap() {
        assert!(circle_circle([0.0, 0.0], 1.0, [3.0, 0.0], 1.0).is_none());
    }

    #[test]
    fn circle_circle_coincident() {
        let r = circle_circle([0.0, 0.0], 1.0, [0.0, 0.0], 1.0);
        assert!(r.is_some());
        let (_, d, _) = r.unwrap();
        assert!((d - 2.0).abs() < EPS);
    }

    #[test]
    fn circle_aabb_overlap() {
        let r = circle_aabb([1.8, 0.0], 0.5, [0.0, 0.0], [1.5, 1.0]);
        assert!(r.is_some());
        let (n, d, _) = r.unwrap();
        assert!(d > 0.0);
        assert!((n[0] - 1.0).abs() < EPS); // normal points away from box
    }

    #[test]
    fn circle_aabb_no_overlap() {
        assert!(circle_aabb([5.0, 0.0], 0.5, [0.0, 0.0], [1.0, 1.0]).is_none());
    }

    #[test]
    fn circle_aabb_inside() {
        let r = circle_aabb([0.0, 0.0], 0.5, [0.0, 0.0], [2.0, 2.0]);
        assert!(r.is_some());
        let (_, d, _) = r.unwrap();
        assert!(d > 0.0);
    }

    #[test]
    fn aabb_aabb_overlap() {
        let r = aabb_aabb_contact([0.0, 0.0], [1.0, 1.0], [1.5, 0.0], [1.0, 1.0]);
        assert!(r.is_some());
        let (n, d, _) = r.unwrap();
        assert!((n[0] - 1.0).abs() < EPS);
        assert!((d - 0.5).abs() < EPS);
    }

    #[test]
    fn aabb_aabb_no_overlap() {
        assert!(aabb_aabb_contact([0.0, 0.0], [1.0, 1.0], [5.0, 0.0], [1.0, 1.0]).is_none());
    }

    #[test]
    fn aabb_aabb_vertical_overlap() {
        let r = aabb_aabb_contact([0.0, 0.0], [1.0, 1.0], [0.0, 1.5], [1.0, 1.0]);
        assert!(r.is_some());
        let (n, _, _) = r.unwrap();
        assert!((n[1] - 1.0).abs() < EPS);
    }

    // -- Ray intersection tests --

    #[test]
    fn ray_circle_hit() {
        let r = ray_circle([0.0, 0.0], [1.0, 0.0], [5.0, 0.0], 1.0);
        assert!(r.is_some());
        let (t, n) = r.unwrap();
        assert!((t - 4.0).abs() < EPS);
        assert!((n[0] - (-1.0)).abs() < EPS);
    }

    #[test]
    fn ray_circle_miss() {
        assert!(ray_circle([0.0, 0.0], [0.0, 1.0], [5.0, 0.0], 1.0).is_none());
    }

    #[test]
    fn ray_circle_inside() {
        let r = ray_circle([5.0, 0.0], [1.0, 0.0], [5.0, 0.0], 1.0);
        assert!(r.is_some());
        let (t, _) = r.unwrap();
        assert!((t - 1.0).abs() < EPS);
    }

    #[test]
    fn ray_aabb_hit() {
        let r = ray_aabb_2d([0.0, 0.0], [1.0, 0.0], [4.0, -1.0], [6.0, 1.0]);
        assert!(r.is_some());
        let (t, n) = r.unwrap();
        assert!((t - 4.0).abs() < EPS);
        assert!((n[0] - (-1.0)).abs() < EPS);
    }

    #[test]
    fn ray_aabb_miss() {
        assert!(ray_aabb_2d([0.0, 0.0], [0.0, 1.0], [4.0, -1.0], [6.0, 1.0]).is_none());
    }

    #[test]
    fn ray_aabb_inside() {
        let r = ray_aabb_2d([5.0, 0.0], [1.0, 0.0], [4.0, -1.0], [6.0, 1.0]);
        assert!(r.is_some());
    }

    // -- AABB overlap tests --

    #[test]
    fn aabb_overlaps() {
        let a = Aabb2d { min: [0.0, 0.0], max: [2.0, 2.0] };
        let b = Aabb2d { min: [1.0, 1.0], max: [3.0, 3.0] };
        assert!(a.overlaps(&b));
        assert!(b.overlaps(&a));
    }

    #[test]
    fn aabb_no_overlap() {
        let a = Aabb2d { min: [0.0, 0.0], max: [1.0, 1.0] };
        let b = Aabb2d { min: [2.0, 2.0], max: [3.0, 3.0] };
        assert!(!a.overlaps(&b));
    }

    // -- Mass/inertia tests --

    #[test]
    fn ball_mass() {
        let c = Collider2d::from_desc(
            ColliderHandle(0),
            BodyHandle(0),
            &ColliderDesc {
                shape: ColliderShape::Ball { radius: 1.0 },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        let m = c.compute_mass();
        assert!((m - std::f64::consts::PI).abs() < EPS);
    }

    #[test]
    fn box_mass() {
        let c = Collider2d::from_desc(
            ColliderHandle(0),
            BodyHandle(0),
            &ColliderDesc {
                shape: ColliderShape::Box { half_extents: [1.0, 1.0, 0.0] },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        assert!((c.compute_mass() - 4.0).abs() < EPS);
    }

    #[test]
    fn segment_mass_nonzero() {
        let c = Collider2d::from_desc(
            ColliderHandle(0),
            BodyHandle(0),
            &ColliderDesc {
                shape: ColliderShape::Segment { a: [0.0, 0.0, 0.0], b: [10.0, 0.0, 0.0] },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        assert!(c.compute_mass() > 0.0);
    }

    #[test]
    fn explicit_mass_overrides() {
        let c = Collider2d::from_desc(
            ColliderHandle(0),
            BodyHandle(0),
            &ColliderDesc {
                shape: ColliderShape::Ball { radius: 1.0 },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: Some(42.0),
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        assert!((c.compute_mass() - 42.0).abs() < EPS);
    }

    #[test]
    fn multiple_colliders_accumulate_mass() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc::default());

        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        let mass_after_first = state.bodies.get(body_ah(bh)).unwrap().mass;

        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [1.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        let mass_after_second = state.bodies.get(body_ah(bh)).unwrap().mass;

        assert!(mass_after_second > mass_after_first);
        assert!((mass_after_second - 2.0 * mass_after_first).abs() < EPS);
    }

    // -- Capsule contact tests --

    #[test]
    fn capsule_circle_overlap() {
        // Vertical capsule at origin, circle to the right
        let r = capsule_circle([0.0, 0.0], 0.0, 1.0, 0.5, [0.8, 0.0], 0.5);
        assert!(r.is_some());
        let (n, d, _) = r.unwrap();
        assert!(d > 0.0);
        assert!((n[0] - 1.0).abs() < EPS); // normal points toward circle
    }

    #[test]
    fn capsule_circle_miss() {
        assert!(capsule_circle([0.0, 0.0], 0.0, 1.0, 0.5, [5.0, 0.0], 0.5).is_none());
    }

    #[test]
    fn capsule_circle_endpoint() {
        // Circle near the top endpoint of a vertical capsule
        let r = capsule_circle([0.0, 0.0], 0.0, 1.0, 0.5, [0.0, 1.3], 0.5);
        assert!(r.is_some());
        let (n, d, _) = r.unwrap();
        assert!(d > 0.0);
        assert!(n[1] > 0.5); // normal should point upward
    }

    #[test]
    fn capsule_aabb_overlap() {
        let r = capsule_aabb([0.0, 0.0], 0.0, 1.0, 0.5, [0.8, 0.0], [0.5, 0.5]);
        assert!(r.is_some());
        let (_, d, _) = r.unwrap();
        assert!(d > 0.0);
    }

    #[test]
    fn capsule_aabb_miss() {
        assert!(capsule_aabb([0.0, 0.0], 0.0, 1.0, 0.5, [5.0, 0.0], [0.5, 0.5]).is_none());
    }

    #[test]
    fn capsule_capsule_overlap() {
        // Two vertical capsules side by side
        let r = capsule_capsule(
            [0.0, 0.0], 0.0, 1.0, 0.5,
            [0.8, 0.0], 0.0, 1.0, 0.5,
        );
        assert!(r.is_some());
        let (n, d, _) = r.unwrap();
        assert!(d > 0.0);
        assert!((n[0] - 1.0).abs() < EPS);
    }

    #[test]
    fn capsule_capsule_miss() {
        assert!(capsule_capsule(
            [0.0, 0.0], 0.0, 1.0, 0.5,
            [5.0, 0.0], 0.0, 1.0, 0.5,
        ).is_none());
    }

    #[test]
    fn capsule_capsule_perpendicular() {
        // Vertical capsule at x=0, horizontal capsule at x=0.2
        // With radius 0.5 each, sum=1.0, so overlap when segment dist < 1.0
        let r = capsule_capsule(
            [0.0, 0.0], 0.0, 1.0, 0.5,
            [0.2, 0.0], std::f64::consts::FRAC_PI_2, 1.0, 0.5,
        );
        assert!(r.is_some(), "capsule-capsule should overlap");
        let (_, d, _) = r.unwrap();
        assert!(d > 0.0);
    }

    // -- Capsule ray tests --

    #[test]
    fn ray_capsule_hit_shaft() {
        // Horizontal ray hitting the shaft of a vertical capsule
        let r = ray_capsule([0.0, 0.0], [1.0, 0.0], [5.0, 0.0], 0.0, 1.0, 0.5);
        assert!(r.is_some());
        let (t, _) = r.unwrap();
        assert!((t - 4.5).abs() < 0.1); // should hit at ~4.5 (5 - radius 0.5)
    }

    #[test]
    fn ray_capsule_hit_endpoint() {
        // Ray aimed at the top endpoint
        let r = ray_capsule([0.0, 2.0], [1.0, 0.0], [5.0, 0.0], 0.0, 2.0, 0.5);
        assert!(r.is_some());
    }

    #[test]
    fn ray_capsule_miss() {
        assert!(ray_capsule([0.0, 5.0], [1.0, 0.0], [5.0, 0.0], 0.0, 1.0, 0.5).is_none());
    }

    // -- World AABB tests --

    #[test]
    fn world_aabb_ball_no_rotation() {
        let c = Collider2d::from_desc(
            ColliderHandle(0),
            BodyHandle(0),
            &ColliderDesc {
                shape: ColliderShape::Ball { radius: 1.0 },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        let aabb = c.world_aabb([5.0, 3.0], 0.0);
        assert!((aabb.min[0] - 4.0).abs() < EPS);
        assert!((aabb.max[0] - 6.0).abs() < EPS);
        assert!((aabb.min[1] - 2.0).abs() < EPS);
        assert!((aabb.max[1] - 4.0).abs() < EPS);
    }

    #[test]
    fn world_aabb_box_with_rotation() {
        let c = Collider2d::from_desc(
            ColliderHandle(0),
            BodyHandle(0),
            &ColliderDesc {
                shape: ColliderShape::Box { half_extents: [2.0, 1.0, 0.0] },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        // 90 degree rotation swaps extents
        let aabb = c.world_aabb([0.0, 0.0], std::f64::consts::FRAC_PI_2);
        // After 90deg rotation, a 2x1 box becomes ~1x2 in AABB
        assert!((aabb.max[0] - aabb.min[0] - 2.0).abs() < 0.01);
        assert!((aabb.max[1] - aabb.min[1] - 4.0).abs() < 0.01);
    }

    #[test]
    fn world_aabb_with_offset() {
        let c = Collider2d::from_desc(
            ColliderHandle(0),
            BodyHandle(0),
            &ColliderDesc {
                shape: ColliderShape::Ball { radius: 0.5 },
                offset: [3.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        let aabb = c.world_aabb([0.0, 0.0], 0.0);
        assert!((aabb.min[0] - 2.5).abs() < EPS);
        assert!((aabb.max[0] - 3.5).abs() < EPS);
    }

    // -- Sensor tests --

    #[test]
    fn sensor_generates_events_no_physics() {
        let mut state = PhysicsState2d::new();

        // Static floor
        let floor = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(floor, &ColliderDesc {
            shape: ColliderShape::Box { half_extents: [10.0, 0.5, 0.0] },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: true, // Sensor!
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // Dynamic ball overlapping the sensor
        let ball = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(ball, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        let vel_before = state.bodies.get(body_ah(ball)).unwrap().linear_velocity;
        let events = state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);

        // Should generate events
        assert!(!events.is_empty());
        // But sensor should not affect velocity (no physical response)
        let vel_after = state.bodies.get(body_ah(ball)).unwrap().linear_velocity;
        assert!((vel_after[0] - vel_before[0]).abs() < EPS);
        assert!((vel_after[1] - vel_before[1]).abs() < EPS);
    }

    // -- Kinematic body tests --

    #[test]
    fn kinematic_body_moves_from_velocity() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc {
            body_type: BodyType::Kinematic,
            position: [0.0, 0.0, 0.0],
            linear_velocity: [10.0, 0.0, 0.0],
            ..BodyDesc::default()
        });

        let dt = 1.0 / 60.0;
        state.step([0.0, -9.81, 0.0], dt, 4, 1, 0.01, 0.2, 100.0);

        let rb = &state.bodies.get(body_ah(bh)).unwrap();
        // Should have moved from velocity
        assert!(rb.position[0] > 0.0);
        // Should NOT have been affected by gravity
        assert!((rb.linear_velocity[1]).abs() < EPS);
    }

    // -- Stale collision pair cleanup --

    #[test]
    fn remove_body_cleans_collision_pairs() {
        let mut state = PhysicsState2d::new();

        let a = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(a, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.5, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // Step to generate collision manifolds
        state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        assert!(!state.manifolds.is_empty());

        // Remove body b — should clean manifolds
        state.remove_body(b);
        assert!(state.manifolds.is_empty());
    }

    // -- Spatial hash tests (delegated to spatial_hash module, but verify integration) --

    #[test]
    fn spatial_hash_finds_nearby_pair() {
        let mut grid: SpatialHashGrid<ColliderHandle> = SpatialHashGrid::new(2.0);
        grid.insert_2d(ColliderHandle(0), [0.0, 0.0], [1.0, 1.0]);
        grid.insert_2d(ColliderHandle(1), [0.5, 0.5], [1.5, 1.5]);
        let pairs = grid.query_pairs();
        assert!(pairs.contains(&(ColliderHandle(0), ColliderHandle(1))));
    }

    #[test]
    fn spatial_hash_no_false_pair() {
        let mut grid: SpatialHashGrid<ColliderHandle> = SpatialHashGrid::new(1.0);
        grid.insert_2d(ColliderHandle(0), [0.0, 0.0], [0.5, 0.5]);
        grid.insert_2d(ColliderHandle(1), [10.0, 10.0], [10.5, 10.5]);
        let pairs = grid.query_pairs();
        assert!(pairs.is_empty());
    }

    #[test]
    fn spatial_hash_auto_cell_size() {
        let dims = vec![1.0_f64, 2.0];
        let size = SpatialHashGrid::<ColliderHandle>::auto_cell_size(dims.into_iter(), 2);
        // avg max dimension = (1+2)/2 = 1.5, *2 = 3.0
        assert!((size - 3.0).abs() < EPS);
    }

    #[test]
    fn spatial_hash_empty() {
        let size = SpatialHashGrid::<ColliderHandle>::auto_cell_size(std::iter::empty(), 0);
        assert!((size - 1.0).abs() < EPS);
    }

    // -- Segment closest point tests --

    #[test]
    fn closest_point_on_segment_midpoint() {
        let (pt, t) = closest_point_on_segment([0.0, 0.0], [10.0, 0.0], [5.0, 3.0]);
        assert!((pt[0] - 5.0).abs() < EPS);
        assert!(pt[1].abs() < EPS);
        assert!((t - 0.5).abs() < EPS);
    }

    #[test]
    fn closest_point_on_segment_endpoint_a() {
        let (pt, t) = closest_point_on_segment([0.0, 0.0], [10.0, 0.0], [-5.0, 0.0]);
        assert!(pt[0].abs() < EPS);
        assert!((t).abs() < EPS);
    }

    #[test]
    fn closest_point_on_segment_endpoint_b() {
        let (pt, t) = closest_point_on_segment([0.0, 0.0], [10.0, 0.0], [15.0, 0.0]);
        assert!((pt[0] - 10.0).abs() < EPS);
        assert!((t - 1.0).abs() < EPS);
    }

    #[test]
    fn closest_points_segments_perpendicular() {
        // Vertical (0,-1)→(0,1) vs horizontal (1,0)→(3,0)
        let (p1, p2) = closest_points_segments(
            [0.0, -1.0], [0.0, 1.0],
            [1.0, 0.0], [3.0, 0.0],
        );
        assert!((p1[0]).abs() < EPS);
        assert!((p1[1]).abs() < EPS);
        assert!((p2[0] - 1.0).abs() < EPS);
        assert!((p2[1]).abs() < EPS);
    }

    #[test]
    fn closest_points_segments_parallel() {
        // Two parallel horizontal segments offset in Y
        let (p1, p2) = closest_points_segments(
            [0.0, 0.0], [10.0, 0.0],
            [0.0, 2.0], [10.0, 2.0],
        );
        // Should find closest pair — any point pair with dist=2
        let dist = ((p2[0] - p1[0]).powi(2) + (p2[1] - p1[1]).powi(2)).sqrt();
        assert!((dist - 2.0).abs() < 0.01);
    }

    // =======================================================================
    // Feature 1: Sleep / deactivation tests
    // =======================================================================

    #[test]
    fn body_falls_asleep_when_stationary() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // Zero gravity, zero velocity — body should go to sleep after SLEEP_TIME_THRESHOLD
        let dt = 1.0 / 60.0;
        let steps_needed = (SLEEP_TIME_THRESHOLD / dt).ceil() as usize + 10;
        for _ in 0..steps_needed {
            state.step([0.0, 0.0, 0.0], dt, 4, 1, 0.01, 0.2, 100.0);
        }

        assert!(state.bodies.get(body_ah(bh)).unwrap().is_sleeping, "body should be sleeping after sitting still");
    }

    #[test]
    fn sleeping_body_skips_integration() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // Manually put to sleep
        state.bodies.get_mut(body_ah(bh)).unwrap().is_sleeping = true;

        let pos_before = state.bodies.get(body_ah(bh)).unwrap().position;
        // Step with gravity — sleeping body should not move
        state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);

        let pos_after = state.bodies.get(body_ah(bh)).unwrap().position;
        assert!(
            (pos_after[0] - pos_before[0]).abs() < EPS
                && (pos_after[1] - pos_before[1]).abs() < EPS,
            "sleeping body should not have moved"
        );
    }

    #[test]
    fn force_wakes_sleeping_body() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc::default());
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // Put to sleep
        state.bodies.get_mut(body_ah(bh)).unwrap().is_sleeping = true;
        state.bodies.get_mut(body_ah(bh)).unwrap().sleep_timer = 1.0;

        // Apply force should wake it
        state.apply_force(bh, &Force::new(10.0, 0.0, 0.0));
        assert!(!state.bodies.get(body_ah(bh)).unwrap().is_sleeping);
        assert!((state.bodies.get(body_ah(bh)).unwrap().sleep_timer).abs() < EPS);
    }

    #[test]
    fn impulse_wakes_sleeping_body() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc::default());
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        state.bodies.get_mut(body_ah(bh)).unwrap().is_sleeping = true;
        state.apply_impulse(bh, &Impulse::new(10.0, 0.0, 0.0));
        assert!(!state.bodies.get(body_ah(bh)).unwrap().is_sleeping);
    }

    #[test]
    fn torque_wakes_sleeping_body() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc::default());
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        state.bodies.get_mut(body_ah(bh)).unwrap().is_sleeping = true;
        state.apply_torque(bh, &Torque::new(5.0));
        assert!(!state.bodies.get(body_ah(bh)).unwrap().is_sleeping);
    }

    #[test]
    fn moving_body_does_not_sleep() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            linear_velocity: [5.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // Step many times — body is moving so should not sleep
        for _ in 0..100 {
            state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        }
        assert!(!state.bodies.get(body_ah(bh)).unwrap().is_sleeping);
    }

    #[test]
    fn get_body_state_reports_sleeping() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc::default());
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        assert!(!state.get_body_state(bh).unwrap().is_sleeping);

        state.bodies.get_mut(body_ah(bh)).unwrap().is_sleeping = true;
        assert!(state.get_body_state(bh).unwrap().is_sleeping);
    }

    #[test]
    fn contact_wakes_sleeping_body() {
        let mut state = PhysicsState2d::new();

        // A sleeping body at origin
        let a = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(a, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        state.bodies.get_mut(body_ah(a)).unwrap().is_sleeping = true;

        // A moving body heading toward it
        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [3.0, 0.0, 0.0],
            linear_velocity: [-5.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // Step until contact
        for _ in 0..60 {
            state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        }

        // The sleeping body should have been woken by the impact
        assert!(!state.bodies.get(body_ah(a)).unwrap().is_sleeping, "sleeping body should wake on contact");
    }

    // =======================================================================
    // Feature 2: Collision layer filtering tests
    // =======================================================================

    #[test]
    fn collision_layers_prevent_collision() {
        let mut state = PhysicsState2d::new();

        // Two overlapping bodies on different layers
        let a = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(a, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0x01,  // layer 1
            collision_mask: 0x01,   // only collide with layer 1
        });

        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.5, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0x02,  // layer 2
            collision_mask: 0x02,   // only collide with layer 2
        });

        // Step — no collision events should be generated
        let events = state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        assert!(events.is_empty(), "different collision layers should not generate events");
    }

    #[test]
    fn collision_layers_allow_same_layer() {
        let mut state = PhysicsState2d::new();

        // Two overlapping bodies on the same layer
        let a = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(a, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0x01,
            collision_mask: 0x01,
        });

        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.5, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0x01,
            collision_mask: 0x01,
        });

        let events = state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        assert!(!events.is_empty(), "same collision layer should generate events");
    }

    #[test]
    fn collision_layers_asymmetric_mask() {
        let mut state = PhysicsState2d::new();

        // A can see B's layer, but B cannot see A's layer
        let a = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(a, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0x01,
            collision_mask: 0x03, // sees layer 1 and 2
        });

        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.5, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0x02,
            collision_mask: 0x02, // only sees layer 2
        });

        // A's mask includes B's layer (0x02 & 0x03 != 0), so collision should happen
        let events = state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        assert!(!events.is_empty(), "asymmetric mask should still allow collision when one side matches");
    }

    #[test]
    fn collision_layers_default_collide_everything() {
        // Default layers (0xFFFF_FFFF) should collide with everything
        let mut state = PhysicsState2d::new();

        let a = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(a, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.5, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        let events = state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        assert!(!events.is_empty(), "default layers should collide");
    }

    // =======================================================================
    // Feature 3: Overlap query tests
    // =======================================================================

    #[test]
    fn overlap_sphere_finds_ball() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [5.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        let ch = ColliderHandle(0);
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // Sphere query overlapping the ball
        let hits = state.overlap_sphere([5.0, 0.0, 0.0], 0.5);
        assert!(hits.contains(&ch), "should find overlapping ball");

        // Sphere query far away
        let misses = state.overlap_sphere([100.0, 0.0, 0.0], 0.5);
        assert!(misses.is_empty(), "should not find distant ball");
    }

    #[test]
    fn overlap_sphere_finds_box() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        let ch = ColliderHandle(0);
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Box { half_extents: [2.0, 2.0, 0.0] },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        let hits = state.overlap_sphere([2.3, 0.0, 0.0], 0.5);
        assert!(hits.contains(&ch), "sphere near box edge should overlap");

        let misses = state.overlap_sphere([10.0, 0.0, 0.0], 0.5);
        assert!(misses.is_empty(), "distant sphere should not overlap");
    }

    #[test]
    fn overlap_sphere_misses_distant() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        let results = state.overlap_sphere([10.0, 10.0, 0.0], 0.5);
        assert!(results.is_empty());
    }

    #[test]
    fn overlap_aabb_finds_colliders() {
        let mut state = PhysicsState2d::new();

        let b1 = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        let c1 = ColliderHandle(0);
        state.add_collider(b1, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        let b2 = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [10.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        let c2 = ColliderHandle(1);
        state.add_collider(b2, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // AABB covering only the first collider
        let hits = state.overlap_aabb([-2.0, -2.0, 0.0], [2.0, 2.0, 0.0]);
        assert!(hits.contains(&c1), "should find collider at origin");
        assert!(!hits.contains(&c2), "should not find collider at x=10");

        // AABB covering both
        let all_hits = state.overlap_aabb([-2.0, -2.0, 0.0], [12.0, 2.0, 0.0]);
        assert!(all_hits.contains(&c1));
        assert!(all_hits.contains(&c2));
    }

    #[test]
    fn overlap_aabb_empty() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        let results = state.overlap_aabb([50.0, 50.0, 0.0], [60.0, 60.0, 0.0]);
        assert!(results.is_empty());
    }

    #[test]
    fn overlap_sphere_finds_capsule() {
        let mut state = PhysicsState2d::new();
        let bh = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        let ch = ColliderHandle(0);
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Capsule { half_height: 2.0, radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // Query near the capsule endpoint
        let hits = state.overlap_sphere([0.0, 2.3, 0.0], 0.5);
        assert!(hits.contains(&ch), "sphere near capsule endpoint should overlap");

        let misses = state.overlap_sphere([5.0, 0.0, 0.0], 0.5);
        assert!(misses.is_empty(), "distant sphere should not overlap capsule");
    }

    // =======================================================================
    // Feature: Persistent contact manifolds & warm starting
    // =======================================================================

    #[test]
    fn manifold_persistence() {
        let mut state = PhysicsState2d::new();

        // Static floor
        let floor = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, -1.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(floor, &ColliderDesc {
            shape: ColliderShape::Box { half_extents: [10.0, 1.0, 0.0] },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial { friction: 0.5, restitution: 0.0, density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // Dynamic box sitting on floor (overlapping slightly — bottom at y=-0.1, floor top at y=0)
        let box_body = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.4, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(box_body, &ColliderDesc {
            shape: ColliderShape::Box { half_extents: [0.5, 0.5, 0.0] },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial { friction: 0.5, restitution: 0.0, density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // Step once — manifold should be created
        state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 8, 4, 0.01, 0.2, 100.0);
        assert!(!state.manifolds.is_empty(), "manifold should exist after first step with contact");

        // Step again — manifold should persist and accumulate impulses
        state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 8, 4, 0.01, 0.2, 100.0);
        assert!(!state.manifolds.is_empty(), "manifold should persist across frames");

        // Check that accumulated impulse is non-zero (warm starting cached from previous frame)
        let has_nonzero_impulse = state.manifolds.values().any(|m| {
            m.points.iter().any(|p| p.normal_impulse.abs() > EPS)
        });
        assert!(has_nonzero_impulse, "manifold should have non-zero accumulated impulse after two frames");
    }

    #[test]
    fn warm_start_stabilizes_stack() {
        let mut state = PhysicsState2d::new();

        // Static floor
        let floor = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, -0.5, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(floor, &ColliderDesc {
            shape: ColliderShape::Box { half_extents: [20.0, 0.5, 0.0] },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial { friction: 0.8, restitution: 0.0, density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        // Stack of 5 boxes
        let box_size = 0.5;
        let mut top_handle = BodyHandle(0);
        for i in 0..5 {
            let y = box_size + (i as f64) * (2.0 * box_size);
            let bh = state.add_body(&BodyDesc {
                body_type: BodyType::Dynamic,
                position: [0.0, y, 0.0],
                ..BodyDesc::default()
            });
            state.add_collider(bh, &ColliderDesc {
                shape: ColliderShape::Box { half_extents: [box_size, box_size, 0.0] },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial { friction: 0.8, restitution: 0.0, density: 1.0, ..PhysicsMaterial::default() },
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            });
            if i == 4 {
                top_handle = bh;
            }
        }

        let initial_y = state.bodies.get(body_ah(top_handle)).unwrap().position[1];

        // Step 300 frames
        let dt = 1.0 / 60.0;
        for _ in 0..300 {
            state.step([0.0, -9.81, 0.0], dt, 8, 4, 0.01, 0.2, 100.0);
        }

        let final_y = state.bodies.get(body_ah(top_handle)).unwrap().position[1];

        // The top box should have settled near its expected resting position.
        // Expected: floor is at y=-0.5..0.0, boxes are stacked starting at y=0.
        // Top of 5th box should be near y = 5 * 1.0 = 5.0, center near y = 4.5.
        // With warm starting, the stack should be stable. Allow some settling.
        assert!(
            final_y > 3.0,
            "top box should be stable in stack (y={final_y}), expected > 3.0"
        );
        // It should have settled (not flying upward)
        assert!(
            final_y < initial_y + 1.0,
            "top box should not have been launched (y={final_y})"
        );
    }

    // -- Wheel joint tests --

    #[test]
    fn wheel_joint_constrains_perpendicular() {
        let mut state = PhysicsState2d::new();
        let a = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, -1.0, 0.0],
            ..Default::default()
        });
        state.add_collider(b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        state.add_joint(&JointDesc {
            body_a: a,
            body_b: b,
            joint_type: JointType::Wheel {
                axis: [0.0, 1.0],
                stiffness: 500.0,
                damping: 10.0,
            },
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
            damping: 0.0,
            break_force: None,
        });

        for _ in 0..60 {
            state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        }

        let pos = state.bodies.get(body_ah(b)).unwrap().position;
        // Should be constrained horizontally near x=0
        assert!(pos[0].abs() < 0.1, "wheel should constrain x (x={})", pos[0]);
    }

    // -- Rope joint tests --

    #[test]
    fn rope_joint_allows_closer_than_max() {
        let mut state = PhysicsState2d::new();
        let a = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [1.0, 0.0, 0.0],
            ..Default::default()
        });
        state.add_collider(b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        state.add_joint(&JointDesc {
            body_a: a,
            body_b: b,
            joint_type: JointType::Rope { max_length: 5.0 },
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
            damping: 0.0,
            break_force: None,
        });

        // Body is within max_length, should fall freely with gravity
        let initial_y = state.bodies.get(body_ah(b)).unwrap().position[1];
        for _ in 0..30 {
            state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        }
        let final_y = state.bodies.get(body_ah(b)).unwrap().position[1];
        assert!(final_y < initial_y, "body should fall under gravity (y={})", final_y);
    }

    #[test]
    fn rope_joint_prevents_exceeding_max_length() {
        let mut state = PhysicsState2d::new();
        let a = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, -1.0, 0.0],
            ..Default::default()
        });
        state.add_collider(b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        state.add_joint(&JointDesc {
            body_a: a,
            body_b: b,
            joint_type: JointType::Rope { max_length: 2.0 },
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
            damping: 0.0,
            break_force: None,
        });

        // Let gravity pull body down
        for _ in 0..120 {
            state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        }

        let pos = state.bodies.get(body_ah(b)).unwrap().position;
        let dist = (pos[0] * pos[0] + pos[1] * pos[1]).sqrt();
        assert!(
            dist < 2.5,
            "rope should constrain distance (dist={dist}), max_length=2.0"
        );
    }

    // -- Mouse joint tests --

    #[test]
    fn mouse_joint_drags_body_toward_target() {
        let mut state = PhysicsState2d::new();
        let a = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        state.add_collider(a, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        // body_b is ignored for Mouse joint, but still required
        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            ..Default::default()
        });
        state.add_joint(&JointDesc {
            body_a: a,
            body_b: b,
            joint_type: JointType::Mouse {
                target: [5.0, 0.0, 0.0],
                stiffness: 500.0,
                damping: 20.0,
                max_force: 1000.0,
            },
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
            damping: 0.0,
            break_force: None,
        });

        for _ in 0..120 {
            state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        }

        let pos = state.bodies.get(body_ah(a)).unwrap().position;
        assert!(pos[0] > 2.0, "mouse joint should pull body toward x=5 (x={})", pos[0]);
    }

    // -- Joint breaking tests --

    #[test]
    fn joint_breaking_removes_joint() {
        let mut state = PhysicsState2d::new();
        let a = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, -3.0, 0.0],
            ..Default::default()
        });
        state.add_collider(b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        // Very weak joint that should break under gravity
        let jh = state.add_joint(&JointDesc {
            body_a: a,
            body_b: b,
            joint_type: JointType::Fixed,
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
            damping: 0.0,
            break_force: Some(0.001), // very low break force
        });

        assert!(state.joints.contains(joint_ah(jh)));

        for _ in 0..10 {
            state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        }

        // Joint should have been broken and removed
        assert!(!state.joints.contains(joint_ah(jh)), "joint should be broken");
    }

    #[test]
    fn joint_not_broken_when_force_below_threshold() {
        let mut state = PhysicsState2d::new();
        let a = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        state.add_collider(b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        // Strong joint that should NOT break
        let jh = state.add_joint(&JointDesc {
            body_a: a,
            body_b: b,
            joint_type: JointType::Fixed,
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
            damping: 0.0,
            break_force: Some(100000.0),
        });

        for _ in 0..10 {
            state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        }

        assert!(state.joints.contains(joint_ah(jh)), "strong joint should survive");
    }

    // -- Combine rule tests --

    #[test]
    fn combine_rule_max_priority_wins() {
        use crate::material::CombineRule;
        // Max rule should override Average
        assert_eq!(CombineRule::Max.max(CombineRule::Average), CombineRule::Max);
        // When one material wants Max and other wants Min, Max wins
        let val = CombineRule::Max.combine(0.2, 0.8);
        assert!((val - 0.8).abs() < EPS);
    }
}
