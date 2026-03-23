//! Native 3D physics backend.
//!
//! Implements broadphase (spatial hash), narrowphase (shape-vs-shape contact
//! generation), and a sequential impulse constraint solver with friction and
//! angular response. All geometry uses f64 precision.

use std::collections::{BTreeMap, BTreeSet};

use hisab::{DQuat, DVec3};

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
#[allow(dead_code)]
fn joint_ah(h: JointHandle) -> ArenaHandle { ArenaHandle(h.0) }
#[inline(always)]
fn joint_from(ah: ArenaHandle) -> JointHandle { JointHandle(ah.0) }

// ---------------------------------------------------------------------------
// Named constants
// ---------------------------------------------------------------------------

/// General-purpose geometry epsilon for zero-length checks.
const EPSILON: f64 = 1e-10;
/// Squared epsilon for distance-squared checks.
const EPSILON_SQ: f64 = 1e-20;
/// Bodies with both linear and angular speed below this are candidates for sleep.
const SLEEP_VELOCITY_THRESHOLD_3D: f64 = 0.01;
/// How many seconds of low motion before a body is put to sleep.
const SLEEP_TIME_THRESHOLD_3D: f64 = 0.5;
/// Minimum mass to avoid division by zero for dynamic bodies.
const MIN_MASS: f64 = 1e-6;
/// Minimum inertia to avoid division by zero.
const MIN_INERTIA: f64 = 1e-10;

// ---------------------------------------------------------------------------
// Internal body representation
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub(crate) struct RigidBody3d {
    pub handle: BodyHandle,
    pub body_type: BodyType,
    pub position: DVec3,
    pub rotation: DQuat,
    pub linear_velocity: DVec3,
    pub angular_velocity: DVec3,
    pub linear_damping: f64,
    pub angular_damping: f64,
    pub fixed_rotation: bool,
    pub gravity_scale: f64,
    pub force_accumulator: DVec3,
    pub torque_accumulator: DVec3,
    pub mass: f64,
    pub inv_mass: f64,
    pub inertia: DVec3, // diagonal inertia tensor
    pub inv_inertia: DVec3,
    // Sleep state
    pub is_sleeping: bool,
    pub sleep_timer: f64,
}

impl RigidBody3d {
    fn from_desc(handle: BodyHandle, desc: &BodyDesc) -> Self {
        Self {
            handle,
            body_type: desc.body_type,
            position: DVec3::from_array(desc.position),
            rotation: DQuat::from_rotation_z(desc.rotation),
            linear_velocity: DVec3::from_array(desc.linear_velocity),
            angular_velocity: DVec3::new(0.0, 0.0, desc.angular_velocity),
            linear_damping: desc.linear_damping,
            angular_damping: desc.angular_damping,
            fixed_rotation: desc.fixed_rotation,
            gravity_scale: desc.gravity_scale.unwrap_or(1.0),
            force_accumulator: DVec3::ZERO,
            torque_accumulator: DVec3::ZERO,
            mass: 0.0,
            inv_mass: 0.0,
            inertia: DVec3::ZERO,
            inv_inertia: DVec3::ZERO,
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

    fn integrate_velocities(&mut self, gravity: DVec3, dt: f64, max_velocity: f64) {
        if !self.is_dynamic() || self.inv_mass == 0.0 {
            return;
        }
        if self.is_sleeping {
            return;
        }

        self.linear_velocity += gravity * (self.gravity_scale * dt);
        self.linear_velocity += self.force_accumulator * (self.inv_mass * dt);

        if !self.fixed_rotation {
            let torque_effect = self.torque_accumulator * self.inv_inertia * dt;
            self.angular_velocity += torque_effect;
        }

        let damp = 1.0 / (1.0 + dt * self.linear_damping);
        self.linear_velocity *= damp;
        let adamp = 1.0 / (1.0 + dt * self.angular_damping);
        self.angular_velocity *= adamp;

        // CCD: clamp velocity magnitude to prevent tunneling
        let speed_sq = self.linear_velocity.dot(self.linear_velocity);
        if speed_sq > max_velocity * max_velocity {
            let scale = max_velocity / speed_sq.sqrt();
            self.linear_velocity *= scale;
        }
    }

    fn integrate_positions(&mut self, dt: f64) {
        if self.is_static() {
            return;
        }
        if self.is_dynamic() && self.inv_mass == 0.0 {
            return;
        }

        self.position += self.linear_velocity * dt;

        if !self.fixed_rotation {
            let w = self.angular_velocity;
            let half_dt = dt * 0.5;
            let dq = DQuat::from_xyzw(w.x * half_dt, w.y * half_dt, w.z * half_dt, 0.0)
                * self.rotation;
            self.rotation = DQuat::from_xyzw(
                self.rotation.x + dq.x,
                self.rotation.y + dq.y,
                self.rotation.z + dq.z,
                self.rotation.w + dq.w,
            )
            .normalize();
        }
    }

    fn clear_forces(&mut self) {
        self.force_accumulator = DVec3::ZERO;
        self.torque_accumulator = DVec3::ZERO;
    }
}

// ---------------------------------------------------------------------------
// Internal collider
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub(crate) struct Collider3d {
    pub handle: ColliderHandle,
    pub body: BodyHandle,
    pub shape: ColliderShape,
    pub offset: DVec3,
    pub material: PhysicsMaterial,
    pub is_sensor: bool,
    pub mass: Option<f64>,
    pub collision_layer: u32,
    pub collision_mask: u32,
}

impl Collider3d {
    fn from_desc(handle: ColliderHandle, body: BodyHandle, desc: &ColliderDesc) -> Self {
        Self {
            handle,
            body,
            shape: desc.shape.clone(),
            offset: DVec3::from_array(desc.offset),
            material: desc.material.clone(),
            is_sensor: desc.is_sensor,
            mass: desc.mass,
            collision_layer: desc.collision_layer,
            collision_mask: desc.collision_mask,
        }
    }

    fn world_aabb(&self, body_pos: DVec3, body_rot: DQuat) -> Aabb3d {
        let wp = body_pos + body_rot * self.offset;

        match &self.shape {
            ColliderShape::Ball { radius } => {
                let r = DVec3::splat(*radius);
                Aabb3d {
                    min: wp - r,
                    max: wp + r,
                }
            }
            ColliderShape::Box { half_extents } => {
                // Conservative AABB for rotated box
                let he = *half_extents;
                let corners = [
                    DVec3::new(-he[0], -he[1], -he[2]),
                    DVec3::new(he[0], -he[1], -he[2]),
                    DVec3::new(-he[0], he[1], -he[2]),
                    DVec3::new(he[0], he[1], -he[2]),
                    DVec3::new(-he[0], -he[1], he[2]),
                    DVec3::new(he[0], -he[1], he[2]),
                    DVec3::new(-he[0], he[1], he[2]),
                    DVec3::new(he[0], he[1], he[2]),
                ];
                let mut min = DVec3::splat(f64::INFINITY);
                let mut max = DVec3::splat(f64::NEG_INFINITY);
                for c in &corners {
                    let wc = wp + body_rot * *c;
                    min = min.min(wc);
                    max = max.max(wc);
                }
                Aabb3d { min, max }
            }
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                let axis = body_rot * DVec3::new(0.0, *half_height, 0.0);
                let r = DVec3::splat(*radius);
                Aabb3d {
                    min: wp - axis.abs() - r,
                    max: wp + axis.abs() + r,
                }
            }
            ColliderShape::TriMesh { vertices, .. } => {
                let mut min = DVec3::splat(f64::INFINITY);
                let mut max = DVec3::splat(f64::NEG_INFINITY);
                for v in vertices {
                    let wv = wp + body_rot * DVec3::from_array(*v);
                    min = min.min(wv);
                    max = max.max(wv);
                }
                if min.x > max.x { Aabb3d { min: wp, max: wp } } else { Aabb3d { min, max } }
            }
            ColliderShape::ConvexHull { points } => {
                let mut min = DVec3::splat(f64::INFINITY);
                let mut max = DVec3::splat(f64::NEG_INFINITY);
                for p in points {
                    let wv = wp + body_rot * DVec3::new(p[0], p[1], p[2]);
                    min = min.min(wv);
                    max = max.max(wv);
                }
                if min.x > max.x { Aabb3d { min: wp, max: wp } } else { Aabb3d { min, max } }
            }
            ColliderShape::Segment { a, b } => {
                let wa = wp + body_rot * DVec3::from_array(*a);
                let wb = wp + body_rot * DVec3::from_array(*b);
                Aabb3d { min: wa.min(wb), max: wa.max(wb) }
            }
            ColliderShape::Heightfield { heights, scale } => {
                let w = scale[0] * (heights.len().max(1) - 1) as f64;
                let h_min = heights.iter().copied().fold(f64::INFINITY, f64::min);
                let h_max = heights.iter().copied().fold(f64::NEG_INFINITY, f64::max);
                Aabb3d {
                    min: DVec3::new(wp.x, wp.y + h_min * scale[1], wp.z),
                    max: DVec3::new(wp.x + w, wp.y + h_max * scale[1], wp.z + scale[2]),
                }
            }
        }
    }

    fn compute_mass(&self) -> f64 {
        if let Some(m) = self.mass {
            return m.max(MIN_MASS);
        }
        let vol = match &self.shape {
            ColliderShape::Ball { radius } => (4.0 / 3.0) * std::f64::consts::PI * radius.powi(3),
            ColliderShape::Box { half_extents } => {
                8.0 * half_extents[0] * half_extents[1] * half_extents[2]
            }
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                std::f64::consts::PI * radius * radius * 2.0 * half_height
                    + (4.0 / 3.0) * std::f64::consts::PI * radius.powi(3)
            }
            _ => 1.0,
        };
        (vol * self.material.density).max(MIN_MASS)
    }

    fn compute_inertia(&self, mass: f64) -> DVec3 {
        let i = match &self.shape {
            ColliderShape::Ball { radius } => {
                let i = 0.4 * mass * radius * radius;
                DVec3::splat(i)
            }
            ColliderShape::Box { half_extents } => {
                let w = 2.0 * half_extents[0];
                let h = 2.0 * half_extents[1];
                let d = 2.0 * half_extents[2];
                DVec3::new(
                    mass * (h * h + d * d) / 12.0,
                    mass * (w * w + d * d) / 12.0,
                    mass * (w * w + h * h) / 12.0,
                )
            }
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                // Proper capsule inertia: cylinder + two hemispheres with parallel axis theorem
                let r = *radius;
                let hh = *half_height;
                let r2 = r * r;
                let r3 = r2 * r;
                let h = 2.0 * hh;

                let cyl_vol = std::f64::consts::PI * r2 * h;
                let sphere_vol = (4.0 / 3.0) * std::f64::consts::PI * r3;
                let total_vol = cyl_vol + sphere_vol;
                let cyl_frac = cyl_vol / total_vol;
                let cyl_mass = mass * cyl_frac;
                let sph_mass = mass * (1.0 - cyl_frac);

                // Cylinder (axis along Y): Ixx = Izz = m*(3r²+h²)/12, Iyy = m*r²/2
                let ix_cyl = cyl_mass * (3.0 * r2 + h * h) / 12.0;
                let iy_cyl = cyl_mass * r2 / 2.0;

                // Two hemispheres: I_cm = 2/5 * m * r², offset from center by (hh + 3r/8)
                let offset = hh + 3.0 * r / 8.0;
                let ix_sph = sph_mass * (2.0 * r2 / 5.0 + offset * offset);
                let iy_sph = sph_mass * 2.0 * r2 / 5.0;

                DVec3::new(ix_cyl + ix_sph, iy_cyl + iy_sph, ix_cyl + ix_sph)
            }
            _ => DVec3::splat(mass),
        };
        i.max(DVec3::splat(MIN_INERTIA))
    }
}

// ---------------------------------------------------------------------------
// Internal joint
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub(crate) struct Joint3d {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub joint_type: JointType,
    pub local_anchor_a: DVec3,
    pub local_anchor_b: DVec3,
    pub motor: Option<JointMotor>,
    pub damping: f64,
}

// ---------------------------------------------------------------------------
// 3D AABB
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub(crate) struct Aabb3d {
    pub min: DVec3,
    pub max: DVec3,
}

impl Aabb3d {
    fn overlaps(&self, other: &Aabb3d) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
    }
}

// ---------------------------------------------------------------------------
// Spatial hash broadphase — uses shared SpatialHashGrid from spatial_hash.rs
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Contact
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub(crate) struct Contact3d {
    pub collider_a: ColliderHandle,
    pub collider_b: ColliderHandle,
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub normal: DVec3,
    pub depth: f64,
    pub point: DVec3,
}

// ---------------------------------------------------------------------------
// Physics state
// ---------------------------------------------------------------------------

pub(crate) struct PhysicsState3d {
    pub bodies: Arena<RigidBody3d>,
    pub colliders: Arena<Collider3d>,
    pub joints: Arena<Joint3d>,
    pub body_colliders: BTreeMap<BodyHandle, Vec<ColliderHandle>>,
    prev_collision_pairs: BTreeSet<(ColliderHandle, ColliderHandle)>,
}

impl PhysicsState3d {
    pub fn new() -> Self {
        Self {
            bodies: Arena::new(),
            colliders: Arena::new(),
            joints: Arena::new(),
            body_colliders: BTreeMap::new(),
            prev_collision_pairs: BTreeSet::new(),
        }
    }

    pub fn add_body(&mut self, desc: &BodyDesc) -> BodyHandle {
        let ah = self.bodies.insert(RigidBody3d::from_desc(BodyHandle(0), desc));
        let handle = body_from(ah);
        self.bodies.get_mut(ah).expect("just-inserted body").handle = handle;
        self.body_colliders.insert(handle, Vec::new());
        handle
    }

    pub fn add_collider(&mut self, body: BodyHandle, desc: &ColliderDesc) -> ColliderHandle {
        let collider = Collider3d::from_desc(ColliderHandle(0), body, desc);

        if let Some(rb) = self.bodies.get_mut(body_ah(body))
            && rb.is_dynamic()
        {
            let c_mass = collider.compute_mass();
            let c_inertia = collider.compute_inertia(c_mass);
            rb.mass += c_mass;
            rb.inertia += c_inertia;
            rb.inv_mass = 1.0 / rb.mass;
            rb.inv_inertia = if rb.fixed_rotation {
                DVec3::ZERO
            } else {
                DVec3::new(
                    1.0 / rb.inertia.x,
                    1.0 / rb.inertia.y,
                    1.0 / rb.inertia.z,
                )
            };
        }

        let ah = self.colliders.insert(collider);
        let handle = coll_from(ah);
        self.colliders.get_mut(ah).expect("just-inserted collider").handle = handle;
        self.body_colliders.entry(body).or_default().push(handle);
        handle
    }

    pub fn add_joint(&mut self, desc: &JointDesc) -> JointHandle {
        let ah = self.joints.insert(Joint3d {
            body_a: desc.body_a,
            body_b: desc.body_b,
            joint_type: desc.joint_type.clone(),
            local_anchor_a: DVec3::new(desc.local_anchor_a[0], desc.local_anchor_a[1], 0.0),
            local_anchor_b: DVec3::new(desc.local_anchor_b[0], desc.local_anchor_b[1], 0.0),
            motor: desc.motor.clone(),
            damping: desc.damping,
        });
        joint_from(ah)
    }

    pub fn apply_force(&mut self, body: BodyHandle, force: &Force) {
        if let Some(rb) = self.bodies.get_mut(body_ah(body)) {
            // Wake the body
            rb.is_sleeping = false;
            rb.sleep_timer = 0.0;
            let fv = DVec3::from_array(force.vector);
            rb.force_accumulator += fv;
            if let Some(point) = force.point {
                let p = DVec3::from_array(point);
                rb.torque_accumulator += p.cross(fv);
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
            let iv = DVec3::from_array(impulse.vector);
            rb.linear_velocity += iv * rb.inv_mass;
            if let Some(point) = impulse.point {
                let p = DVec3::from_array(point);
                let ang = p.cross(iv);
                rb.angular_velocity += ang * rb.inv_inertia;
            }
        }
    }

    pub fn apply_torque(&mut self, body: BodyHandle, torque: &Torque) {
        if let Some(rb) = self.bodies.get_mut(body_ah(body)) {
            // Wake the body
            rb.is_sleeping = false;
            rb.sleep_timer = 0.0;
            rb.torque_accumulator.z += torque.value;
        }
    }

    pub fn remove_body(&mut self, handle: BodyHandle) {
        self.bodies.remove(body_ah(handle));
        if let Some(collider_handles) = self.body_colliders.remove(&handle) {
            for ch in &collider_handles {
                self.colliders.remove(coll_ah(*ch));
            }
            self.prev_collision_pairs
                .retain(|(a, b)| !collider_handles.contains(a) && !collider_handles.contains(b));
        }
        self.joints
            .retain(|_, j| j.body_a != handle && j.body_b != handle);
    }

    /// Insert a body at a specific handle (for snapshot restore).
    #[cfg(feature = "serialize")]
    pub fn add_body_at(&mut self, handle: BodyHandle, desc: &BodyDesc) {
        let mut rb = RigidBody3d::from_desc(handle, desc);
        rb.handle = handle;
        self.bodies.insert_at(body_ah(handle), rb);
        self.body_colliders.insert(handle, Vec::new());
    }

    /// Insert a collider at a specific handle (for snapshot restore).
    #[cfg(feature = "serialize")]
    pub fn add_collider_at(&mut self, handle: ColliderHandle, body: BodyHandle, desc: &ColliderDesc) {
        let collider = Collider3d::from_desc(handle, body, desc);
        if let Some(rb) = self.bodies.get_mut(body_ah(body))
            && rb.is_dynamic()
        {
            let c_mass = collider.compute_mass();
            let c_inertia = collider.compute_inertia(c_mass);
            rb.mass += c_mass;
            rb.inertia += c_inertia;
            rb.inv_mass = 1.0 / rb.mass;
            rb.inv_inertia = if rb.fixed_rotation {
                DVec3::ZERO
            } else {
                DVec3::new(1.0 / rb.inertia.x, 1.0 / rb.inertia.y, 1.0 / rb.inertia.z)
            };
        }
        self.colliders.insert_at(coll_ah(handle), collider);
        self.body_colliders.entry(body).or_default().push(handle);
    }

    /// Insert a joint at a specific handle (for snapshot restore).
    #[cfg(feature = "serialize")]
    pub fn add_joint_at(&mut self, handle: JointHandle, desc: &JointDesc) {
        self.joints.insert_at(joint_ah(handle), Joint3d {
            body_a: desc.body_a,
            body_b: desc.body_b,
            joint_type: desc.joint_type.clone(),
            local_anchor_a: DVec3::new(desc.local_anchor_a[0], desc.local_anchor_a[1], 0.0),
            local_anchor_b: DVec3::new(desc.local_anchor_b[0], desc.local_anchor_b[1], 0.0),
            motor: desc.motor.clone(),
            damping: desc.damping,
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
            position: rb.position.to_array(),
            rotation: rb.rotation.z.atan2(rb.rotation.w) * 2.0, // extract z-rotation
            linear_velocity: rb.linear_velocity.to_array(),
            angular_velocity: rb.angular_velocity.z,
            is_sleeping: rb.is_sleeping,
        })
    }

    pub fn set_body_state(&mut self, handle: BodyHandle, state: &BodyState) -> Result<(), ImpetusError> {
        let rb = self.bodies.get_mut(body_ah(handle))
            .ok_or_else(|| ImpetusError::BodyNotFound(format!("{:?}", handle)))?;
        rb.position = DVec3::from_array(state.position);
        rb.rotation = DQuat::from_rotation_z(state.rotation);
        rb.linear_velocity = DVec3::from_array(state.linear_velocity);
        rb.angular_velocity = DVec3::new(0.0, 0.0, state.angular_velocity);
        rb.is_sleeping = false;
        rb.sleep_timer = 0.0;
        Ok(())
    }

    pub fn set_body_type(&mut self, handle: BodyHandle, body_type: BodyType) -> Result<(), ImpetusError> {
        let rb = self.bodies.get_mut(body_ah(handle))
            .ok_or_else(|| ImpetusError::BodyNotFound(format!("{:?}", handle)))?;
        rb.body_type = body_type;
        match body_type {
            BodyType::Static | BodyType::Kinematic => {
                rb.inv_mass = 0.0;
                rb.inv_inertia = DVec3::ZERO;
                rb.linear_velocity = DVec3::ZERO;
                rb.angular_velocity = DVec3::ZERO;
            }
            BodyType::Dynamic => {
                if rb.mass > 0.0 {
                    rb.inv_mass = 1.0 / rb.mass;
                    rb.inv_inertia = if rb.fixed_rotation {
                        DVec3::ZERO
                    } else {
                        DVec3::new(
                            1.0 / rb.inertia.x,
                            1.0 / rb.inertia.y,
                            1.0 / rb.inertia.z,
                        )
                    };
                }
            }
        }
        Ok(())
    }

    // -----------------------------------------------------------------------
    // Step
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
        let g = DVec3::from_array(gravity);
        // 1. Integrate velocities
        for rb in self.bodies.values_mut() {
            rb.integrate_velocities(g, dt, max_velocity);
        }

        // 2-3. Broadphase + narrowphase
        let broad_pairs = self.broadphase();
        let contacts = self.narrowphase(&broad_pairs);

        // 4. Wake sleeping bodies on contact with non-sleeping moving bodies
        for contact in &contacts {
            let a_sleeping = self
                .bodies
                .get(body_ah(contact.body_a))
                .is_some_and(|b| b.is_sleeping);
            let b_sleeping = self
                .bodies
                .get(body_ah(contact.body_b))
                .is_some_and(|b| b.is_sleeping);
            let a_moving = self.bodies.get(body_ah(contact.body_a)).is_some_and(|b| {
                !b.is_sleeping
                    && b.is_dynamic()
                    && (b.linear_velocity.length() > SLEEP_VELOCITY_THRESHOLD_3D
                        || b.angular_velocity.length() > SLEEP_VELOCITY_THRESHOLD_3D)
            });
            let b_moving = self.bodies.get(body_ah(contact.body_b)).is_some_and(|b| {
                !b.is_sleeping
                    && b.is_dynamic()
                    && (b.linear_velocity.length() > SLEEP_VELOCITY_THRESHOLD_3D
                        || b.angular_velocity.length() > SLEEP_VELOCITY_THRESHOLD_3D)
            });
            if a_sleeping
                && b_moving
                && let Some(ba) = self.bodies.get_mut(body_ah(contact.body_a))
            {
                ba.is_sleeping = false;
                ba.sleep_timer = 0.0;
            }
            if b_sleeping
                && a_moving
                && let Some(bb) = self.bodies.get_mut(body_ah(contact.body_b))
            {
                bb.is_sleeping = false;
                bb.sleep_timer = 0.0;
            }
        }

        // 5. Solve velocity constraints
        self.solve_contacts(&contacts, velocity_iterations);

        // 6. Solve joint constraints
        self.solve_joints(dt, velocity_iterations);

        // 7. Positional correction
        self.solve_positions(&contacts, position_iterations, slop, correction);

        // 8. Integrate positions
        for rb in self.bodies.values_mut() {
            rb.integrate_positions(dt);
        }

        // 9. Sleep check: put nearly-stationary dynamic bodies to sleep
        for rb in self.bodies.values_mut() {
            if !rb.is_dynamic() || rb.inv_mass == 0.0 {
                continue;
            }
            let lin_speed = rb.linear_velocity.length();
            let ang_speed = rb.angular_velocity.length();
            if lin_speed < SLEEP_VELOCITY_THRESHOLD_3D && ang_speed < SLEEP_VELOCITY_THRESHOLD_3D {
                rb.sleep_timer += dt;
                if rb.sleep_timer >= SLEEP_TIME_THRESHOLD_3D {
                    rb.is_sleeping = true;
                }
            } else {
                rb.sleep_timer = 0.0;
                rb.is_sleeping = false;
            }
        }

        // 10. Clear forces
        for rb in self.bodies.values_mut() {
            rb.clear_forces();
        }

        // 11. Generate collision events
        self.generate_events(&contacts)
    }

    // -----------------------------------------------------------------------
    // Broadphase
    // -----------------------------------------------------------------------

    fn broadphase(&self) -> Vec<(ColliderHandle, ColliderHandle)> {
        let collider_aabbs: Vec<(ColliderHandle, Aabb3d)> = self
            .colliders
            .values()
            .filter_map(|c| {
                let rb = self.bodies.get(body_ah(c.body))?;
                Some((c.handle, c.world_aabb(rb.position, rb.rotation)))
            })
            .collect();

        let cell_size = SpatialHashGrid::<ColliderHandle>::auto_cell_size(
            collider_aabbs.iter().map(|(_, aabb)| {
                let size = aabb.max - aabb.min;
                size.x.max(size.y).max(size.z)
            }),
            collider_aabbs.len(),
        );
        let mut grid = SpatialHashGrid::new(cell_size);
        for (handle, aabb) in &collider_aabbs {
            grid.insert_3d(*handle, aabb.min.to_array(), aabb.max.to_array());
        }

        let candidates = grid.query_pairs();
        let aabb_map: BTreeMap<ColliderHandle, Aabb3d> = collider_aabbs.into_iter().collect();

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
            if ca.body == cb.body {
                continue;
            }
            if let (Some(ba), Some(bb)) = (self.bodies.get(body_ah(ca.body)), self.bodies.get(body_ah(cb.body)))
                && ba.is_static() && bb.is_static()
            {
                continue;
            }
            if ca.is_sensor && cb.is_sensor {
                continue;
            }
            // Collision layer filtering
            if (ca.collision_layer & cb.collision_mask) == 0
                && (cb.collision_layer & ca.collision_mask) == 0
            {
                continue;
            }
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

    fn narrowphase(&self, broad_pairs: &[(ColliderHandle, ColliderHandle)]) -> Vec<Contact3d> {
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

            let pos_a = ba.position + ba.rotation * ca.offset;
            let pos_b = bb.position + bb.rotation * cb.offset;

            if let Some((normal, depth, point)) =
                generate_contact_3d(&ca.shape, pos_a, ba.rotation, &cb.shape, pos_b, bb.rotation)
            {
                contacts.push(Contact3d {
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
    // Contact solver
    // -----------------------------------------------------------------------

    fn solve_contacts(&mut self, contacts: &[Contact3d], iterations: u32) {
        struct ContactMat {
            restitution: f64,
            friction: f64,
            is_sensor: bool,
        }
        let materials: Vec<ContactMat> = contacts
            .iter()
            .map(|c| {
                let (r, f, s) = match (
                    self.colliders.get(coll_ah(c.collider_a)),
                    self.colliders.get(coll_ah(c.collider_b)),
                ) {
                    (Some(a), Some(b)) => (
                        a.material.restitution.min(b.material.restitution),
                        (a.material.friction * b.material.friction).sqrt(),
                        a.is_sensor || b.is_sensor,
                    ),
                    _ => (0.0, 0.0, false),
                };
                ContactMat {
                    restitution: r,
                    friction: f,
                    is_sensor: s,
                }
            })
            .collect();

        for _ in 0..iterations {
            for (ci, contact) in contacts.iter().enumerate() {
                if materials[ci].is_sensor {
                    continue;
                }

                let (inv_mass_a, inv_inertia_a, vel_a, angvel_a, pos_a) = {
                    let ba = match self.bodies.get(body_ah(contact.body_a)) {
                        Some(b) => b,
                        None => continue,
                    };
                    (ba.inv_mass, ba.inv_inertia, ba.linear_velocity, ba.angular_velocity, ba.position)
                };
                let (inv_mass_b, inv_inertia_b, vel_b, angvel_b, pos_b) = {
                    let bb = match self.bodies.get(body_ah(contact.body_b)) {
                        Some(b) => b,
                        None => continue,
                    };
                    (bb.inv_mass, bb.inv_inertia, bb.linear_velocity, bb.angular_velocity, bb.position)
                };

                if inv_mass_a == 0.0 && inv_mass_b == 0.0 {
                    continue;
                }

                let n = contact.normal;
                let cp = contact.point;
                let ra = cp - pos_a;
                let rb = cp - pos_b;

                let vel_a_at_cp = vel_a + angvel_a.cross(ra);
                let vel_b_at_cp = vel_b + angvel_b.cross(rb);
                let rel_vel = vel_b_at_cp - vel_a_at_cp;
                let vel_along_normal = rel_vel.dot(n);

                if vel_along_normal > 0.0 {
                    continue;
                }

                let ra_cross_n = ra.cross(n);
                let rb_cross_n = rb.cross(n);
                let ang_eff_a = ra_cross_n.dot(ra_cross_n * inv_inertia_a);
                let ang_eff_b = rb_cross_n.dot(rb_cross_n * inv_inertia_b);
                let inv_mass_sum = inv_mass_a + inv_mass_b + ang_eff_a + ang_eff_b;

                let j = -(1.0 + materials[ci].restitution) * vel_along_normal / inv_mass_sum;
                let impulse_n = n * j;

                if let Some(ba) = self.bodies.get_mut(body_ah(contact.body_a))
                    && ba.is_dynamic()
                {
                    ba.linear_velocity -= impulse_n * ba.inv_mass;
                    let ang_imp = ra.cross(impulse_n);
                    ba.angular_velocity -= ang_imp * ba.inv_inertia;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(contact.body_b))
                    && bb.is_dynamic()
                {
                    bb.linear_velocity += impulse_n * bb.inv_mass;
                    let ang_imp = rb.cross(impulse_n);
                    bb.angular_velocity += ang_imp * bb.inv_inertia;
                }

                // Friction
                let friction = materials[ci].friction;
                if friction > 0.0 {
                    let tangent_vel = rel_vel - n * vel_along_normal;
                    let tangent_speed = tangent_vel.length();
                    if tangent_speed > EPSILON {
                        let tangent = tangent_vel / tangent_speed;
                        let jt = (-tangent_speed / inv_mass_sum)
                            .clamp(-j.abs() * friction, j.abs() * friction);
                        let impulse_t = tangent * jt;

                        if let Some(ba) = self.bodies.get_mut(body_ah(contact.body_a))
                            && ba.is_dynamic()
                        {
                            ba.linear_velocity -= impulse_t * ba.inv_mass;
                            let ang_t = ra.cross(impulse_t);
                            ba.angular_velocity -= ang_t * ba.inv_inertia;
                        }
                        if let Some(bb) = self.bodies.get_mut(body_ah(contact.body_b))
                            && bb.is_dynamic()
                        {
                            bb.linear_velocity += impulse_t * bb.inv_mass;
                            let ang_t = rb.cross(impulse_t);
                            bb.angular_velocity += ang_t * bb.inv_inertia;
                        }
                    }
                }
            }
        }
    }

    fn solve_positions(&mut self, contacts: &[Contact3d], iterations: u32, slop: f64, percent: f64) {

        for _ in 0..iterations {
            for contact in contacts {
                let is_sensor = match (
                    self.colliders.get(coll_ah(contact.collider_a)),
                    self.colliders.get(coll_ah(contact.collider_b)),
                ) {
                    (Some(a), Some(b)) => a.is_sensor || b.is_sensor,
                    _ => false,
                };
                if is_sensor {
                    continue;
                }

                let inv_mass_a = self.bodies.get(body_ah(contact.body_a)).map(|b| b.inv_mass).unwrap_or(0.0);
                let inv_mass_b = self.bodies.get(body_ah(contact.body_b)).map(|b| b.inv_mass).unwrap_or(0.0);
                let inv_mass_sum = inv_mass_a + inv_mass_b;
                if inv_mass_sum == 0.0 {
                    continue;
                }

                let correction_mag = (contact.depth - slop).max(0.0) / inv_mass_sum * percent;
                let correction = contact.normal * correction_mag;

                if let Some(ba) = self.bodies.get_mut(body_ah(contact.body_a))
                    && ba.is_dynamic()
                {
                    ba.position -= correction * ba.inv_mass;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(contact.body_b))
                    && bb.is_dynamic()
                {
                    bb.position += correction * bb.inv_mass;
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Joint solver
    // -----------------------------------------------------------------------

    fn solve_joints(&mut self, dt: f64, iterations: u32) {
        let joints: Vec<Joint3d> = self.joints.values().cloned().collect();

        for _ in 0..iterations {
            for joint in &joints {
                match &joint.joint_type {
                    JointType::Fixed => self.solve_fixed_joint_3d(joint),
                    JointType::Distance { length } => {
                        self.solve_distance_joint_3d(joint, *length);
                    }
                    JointType::Spring {
                        rest_length,
                        stiffness,
                        damping,
                    } => {
                        self.solve_spring_joint_3d(joint, *rest_length, *stiffness, *damping, dt);
                    }
                    _ => {} // Revolute/Prismatic: 3D versions need axis definitions, skip for now
                }
                // Apply joint damping for non-Spring joints (Spring has its own damping).
                if joint.damping > 0.0 && !matches!(joint.joint_type, JointType::Spring { .. }) {
                    self.apply_joint_damping_3d(joint, dt);
                }
                // Motors: applied for Revolute/Prismatic when 3D axis support is added
                let _ = &joint.motor;
            }
        }
    }

    /// Apply velocity damping proportional to relative velocity at anchor points.
    fn apply_joint_damping_3d(&mut self, joint: &Joint3d, dt: f64) {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);

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

        let ra = anchor_a - pos_a;
        let rb = anchor_b - pos_b;
        let va = vel_a + angvel_a.cross(ra);
        let vb = vel_b + angvel_b.cross(rb);
        let rel_vel = vb - va;

        if rel_vel.dot(rel_vel) < EPSILON_SQ {
            return;
        }

        let impulse = rel_vel * (-joint.damping * dt);

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity -= impulse * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity += impulse * bb.inv_mass;
        }
    }

    fn world_anchor_3d(&self, body: BodyHandle, local: DVec3) -> DVec3 {
        let rb = match self.bodies.get(body_ah(body)) {
            Some(b) => b,
            None => return local,
        };
        rb.position + rb.rotation * local
    }

    fn solve_fixed_joint_3d(&mut self, joint: &Joint3d) {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);
        let diff = anchor_b - anchor_a;

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position += diff * 0.5;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position -= diff * 0.5;
        }
    }

    fn solve_distance_joint_3d(&mut self, joint: &Joint3d, length: f64) {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);
        let diff = anchor_b - anchor_a;
        let dist_sq = diff.dot(diff);

        if dist_sq < EPSILON_SQ {
            return;
        }
        let dist = dist_sq.sqrt();

        let n = diff / dist;
        let correction = (dist - length) * 0.5;

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position += n * correction;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position -= n * correction;
        }
    }

    fn solve_spring_joint_3d(
        &mut self,
        joint: &Joint3d,
        rest_length: f64,
        stiffness: f64,
        damping: f64,
        dt: f64,
    ) {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);
        let diff = anchor_b - anchor_a;
        let dist_sq = diff.dot(diff);

        if dist_sq < EPSILON_SQ {
            return;
        }
        let dist = dist_sq.sqrt();

        let n = diff / dist;
        let spring_force = stiffness * (dist - rest_length);

        let vel_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .map(|b| b.linear_velocity)
            .unwrap_or(DVec3::ZERO);
        let vel_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .map(|b| b.linear_velocity)
            .unwrap_or(DVec3::ZERO);
        let rel_vel = vel_b - vel_a;
        let damping_force = damping * rel_vel.dot(n);

        let total_force = spring_force + damping_force;
        let force = n * (total_force * dt);

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity += force * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity -= force * bb.inv_mass;
        }
    }

    // -----------------------------------------------------------------------
    // Events
    // -----------------------------------------------------------------------

    fn generate_events(&mut self, contacts: &[Contact3d]) -> Vec<CollisionEvent> {
        let mut events = Vec::new();
        let current_pairs: BTreeSet<(ColliderHandle, ColliderHandle)> = contacts
            .iter()
            .map(|c| {
                if c.collider_a.0 < c.collider_b.0 {
                    (c.collider_a, c.collider_b)
                } else {
                    (c.collider_b, c.collider_a)
                }
            })
            .collect();

        for pair in &current_pairs {
            if !self.prev_collision_pairs.contains(pair) {
                events.push(CollisionEvent::Started {
                    collider_a: pair.0,
                    collider_b: pair.1,
                });
            }
        }
        for pair in &self.prev_collision_pairs {
            if !current_pairs.contains(pair) {
                events.push(CollisionEvent::Stopped {
                    collider_a: pair.0,
                    collider_b: pair.1,
                });
            }
        }
        self.prev_collision_pairs = current_pairs;
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
        let origin = DVec3::from_array(origin);
        let dir = DVec3::from_array(direction).normalize_or(DVec3::Y);

        let mut best: Option<(f64, ColliderHandle, DVec3, DVec3)> = None;

        for collider in self.colliders.values() {
            let rb = match self.bodies.get(body_ah(collider.body)) {
                Some(b) => b,
                None => continue,
            };
            let pos = rb.position + rb.rotation * collider.offset;

            let hit = match &collider.shape {
                ColliderShape::Ball { radius } => ray_sphere(origin, dir, pos, *radius),
                ColliderShape::Box { half_extents } => {
                    let he = DVec3::from_array(*half_extents);
                    ray_aabb_3d(origin, dir, pos - he, pos + he)
                }
                _ => None,
            };

            if let Some((t, normal)) = hit
                && t >= 0.0
                && t <= max_dist
                && (best.is_none() || t < best.as_ref().unwrap().0)
            {
                let point = origin + dir * t;
                best = Some((t, collider.handle, point, normal));
            }
        }

        best.map(|(distance, collider, point, normal)| RayHit {
            collider,
            point: point.to_array(),
            normal: normal.to_array(),
            distance,
        })
    }
}

// ---------------------------------------------------------------------------
// Narrowphase contact generation
// ---------------------------------------------------------------------------

fn is_identity_quat(q: DQuat) -> bool {
    (q.x.abs() < EPSILON) && (q.y.abs() < EPSILON) && (q.z.abs() < EPSILON) && ((q.w.abs() - 1.0).abs() < EPSILON)
}

fn generate_contact_3d(
    shape_a: &ColliderShape,
    pos_a: DVec3,
    rot_a: DQuat,
    shape_b: &ColliderShape,
    pos_b: DVec3,
    rot_b: DQuat,
) -> Option<(DVec3, f64, DVec3)> {
    match (shape_a, shape_b) {
        // Ball vs Ball
        (ColliderShape::Ball { radius: ra }, ColliderShape::Ball { radius: rb }) => {
            sphere_sphere(pos_a, *ra, pos_b, *rb)
        }
        // Ball vs Box
        (ColliderShape::Ball { radius }, ColliderShape::Box { half_extents }) => {
            sphere_obb(pos_a, *radius, pos_b, rot_b, DVec3::from_array(*half_extents))
        }
        (ColliderShape::Box { half_extents }, ColliderShape::Ball { radius }) => {
            sphere_obb(pos_b, *radius, pos_a, rot_a, DVec3::from_array(*half_extents))
                .map(|(n, d, p)| (-n, d, p))
        }
        // Box vs Box — OBB when rotated, AABB fast path otherwise
        (
            ColliderShape::Box { half_extents: he_a },
            ColliderShape::Box { half_extents: he_b },
        ) => {
            let hea = DVec3::from_array(*he_a);
            let heb = DVec3::from_array(*he_b);
            if is_identity_quat(rot_a) && is_identity_quat(rot_b) {
                aabb_aabb_3d(pos_a, hea, pos_b, heb)
            } else {
                obb_obb_3d(pos_a, rot_a, hea, pos_b, rot_b, heb)
            }
        }
        // Capsule vs Sphere
        (
            ColliderShape::Capsule {
                half_height,
                radius: cr,
            },
            ColliderShape::Ball { radius: br },
        ) => capsule_sphere_3d(pos_a, rot_a, *half_height, *cr, pos_b, *br),
        (
            ColliderShape::Ball { radius: br },
            ColliderShape::Capsule {
                half_height,
                radius: cr,
            },
        ) => {
            capsule_sphere_3d(pos_b, rot_b, *half_height, *cr, pos_a, *br)
                .map(|(n, d, p)| (-n, d, p))
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
        ) => capsule_capsule_3d(pos_a, rot_a, *hh_a, *cr_a, pos_b, rot_b, *hh_b, *cr_b),
        // Capsule vs Box
        (
            ColliderShape::Capsule {
                half_height,
                radius: cr,
            },
            ColliderShape::Box { half_extents },
        ) => capsule_box_3d(pos_a, rot_a, *half_height, *cr, pos_b, rot_b, DVec3::from_array(*half_extents)),
        (
            ColliderShape::Box { half_extents },
            ColliderShape::Capsule {
                half_height,
                radius: cr,
            },
        ) => {
            capsule_box_3d(pos_b, rot_b, *half_height, *cr, pos_a, rot_a, DVec3::from_array(*half_extents))
                .map(|(n, d, p)| (-n, d, p))
        }
        // Segment vs Ball
        (ColliderShape::Segment { a, b }, ColliderShape::Ball { radius }) => {
            segment_sphere_3d(pos_a, rot_a, DVec3::from_array(*a), DVec3::from_array(*b), pos_b, *radius)
        }
        (ColliderShape::Ball { radius }, ColliderShape::Segment { a, b }) => {
            segment_sphere_3d(pos_b, rot_b, DVec3::from_array(*a), DVec3::from_array(*b), pos_a, *radius)
                .map(|(n, d, p)| (-n, d, p))
        }
        // Segment vs Box
        (ColliderShape::Segment { a, b }, ColliderShape::Box { half_extents }) => {
            segment_box_3d(pos_a, rot_a, DVec3::from_array(*a), DVec3::from_array(*b), pos_b, rot_b, DVec3::from_array(*half_extents))
        }
        (ColliderShape::Box { half_extents }, ColliderShape::Segment { a, b }) => {
            segment_box_3d(pos_b, rot_b, DVec3::from_array(*a), DVec3::from_array(*b), pos_a, rot_a, DVec3::from_array(*half_extents))
                .map(|(n, d, p)| (-n, d, p))
        }
        // ConvexHull vs Ball
        (ColliderShape::ConvexHull { points }, ColliderShape::Ball { radius }) => {
            convex_hull_sphere_3d(points, pos_a, rot_a, pos_b, *radius)
        }
        (ColliderShape::Ball { radius }, ColliderShape::ConvexHull { points }) => {
            convex_hull_sphere_3d(points, pos_b, rot_b, pos_a, *radius)
                .map(|(n, d, p)| (-n, d, p))
        }
        _ => None,
    }
}

fn sphere_sphere(
    pos_a: DVec3,
    ra: f64,
    pos_b: DVec3,
    rb: f64,
) -> Option<(DVec3, f64, DVec3)> {
    let d = pos_b - pos_a;
    let dist_sq = d.dot(d);
    let sum_r = ra + rb;

    if dist_sq >= sum_r * sum_r {
        return None;
    }

    let dist = dist_sq.sqrt();
    let (normal, depth) = if dist < EPSILON {
        (DVec3::Y, sum_r)
    } else {
        (d / dist, sum_r - dist)
    };

    let point = pos_a + normal * ra;
    Some((normal, depth, point))
}

/// Sphere vs OBB (oriented bounding box): transform sphere into box-local space.
fn sphere_obb(
    sphere_pos: DVec3,
    radius: f64,
    box_pos: DVec3,
    box_rot: DQuat,
    half_extents: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    // Transform sphere center into box-local space
    let inv_rot = box_rot.inverse();
    let local_sphere = inv_rot * (sphere_pos - box_pos);

    let closest = local_sphere.clamp(-half_extents, half_extents);
    let diff = local_sphere - closest;
    let dist_sq = diff.dot(diff);

    if dist_sq >= radius * radius {
        return None;
    }

    let dist = dist_sq.sqrt();
    let (local_normal, depth) = if dist < EPSILON {
        // Sphere center inside box — push out along minimum-penetration face
        let face_dists = DVec3::new(
            half_extents.x - local_sphere.x.abs(),
            half_extents.y - local_sphere.y.abs(),
            half_extents.z - local_sphere.z.abs(),
        );
        let min_axis = if face_dists.x <= face_dists.y && face_dists.x <= face_dists.z {
            0
        } else if face_dists.y <= face_dists.z {
            1
        } else {
            2
        };
        let mut n = DVec3::ZERO;
        n[min_axis] = if local_sphere[min_axis] >= 0.0 { 1.0 } else { -1.0 };
        (n, face_dists[min_axis] + radius)
    } else {
        (diff / dist, radius - dist)
    };

    // Transform normal and contact point back to world space
    let normal = box_rot * local_normal;
    let point = box_rot * closest + box_pos;
    Some((normal, depth, point))
}

fn aabb_aabb_3d(
    pos_a: DVec3,
    he_a: DVec3,
    pos_b: DVec3,
    he_b: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    let d = pos_b - pos_a;
    let overlap = DVec3::new(
        he_a.x + he_b.x - d.x.abs(),
        he_a.y + he_b.y - d.y.abs(),
        he_a.z + he_b.z - d.z.abs(),
    );

    if overlap.x <= 0.0 || overlap.y <= 0.0 || overlap.z <= 0.0 {
        return None;
    }

    let min_axis = if overlap.x <= overlap.y && overlap.x <= overlap.z {
        0
    } else if overlap.y <= overlap.z {
        1
    } else {
        2
    };

    let mut normal = DVec3::ZERO;
    normal[min_axis] = if d[min_axis] >= 0.0 { 1.0 } else { -1.0 };
    let depth = overlap[min_axis];

    let mut point = pos_a;
    point[min_axis] += normal[min_axis] * he_a[min_axis];

    Some((normal, depth, point))
}

// ---------------------------------------------------------------------------
// Capsule helpers

fn closest_point_on_segment_3d(a: DVec3, b: DVec3, p: DVec3) -> DVec3 {
    let ab = b - a;
    let len_sq = ab.dot(ab);
    if len_sq < EPSILON_SQ {
        return a;
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    a + ab * t
}

/// Capsule endpoints in world space given position, rotation, and half-height.
fn capsule_endpoints_3d(pos: DVec3, rot: DQuat, half_height: f64) -> (DVec3, DVec3) {
    let axis = rot * DVec3::new(0.0, half_height, 0.0);
    (pos - axis, pos + axis)
}

fn capsule_sphere_3d(
    cap_pos: DVec3,
    cap_rot: DQuat,
    half_height: f64,
    cap_radius: f64,
    sphere_pos: DVec3,
    sphere_radius: f64,
) -> Option<(DVec3, f64, DVec3)> {
    let (ep_a, ep_b) = capsule_endpoints_3d(cap_pos, cap_rot, half_height);
    let closest = closest_point_on_segment_3d(ep_a, ep_b, sphere_pos);
    sphere_sphere(closest, cap_radius, sphere_pos, sphere_radius)
}

/// Closest points between two 3D line segments. Returns (point_on_ab, point_on_cd).
fn closest_points_segments_3d(a: DVec3, b: DVec3, c: DVec3, d: DVec3) -> (DVec3, DVec3) {
    let r = b - a; // direction of segment 1
    let s = d - c; // direction of segment 2
    let w = a - c;

    let rr = r.dot(r); // |r|^2
    let ss = s.dot(s); // |s|^2
    let rs = r.dot(s);
    let rw = r.dot(w);
    let sw = s.dot(w);

    let denom = rr * ss - rs * rs;

    let (sc, tc);

    if denom.abs() < EPSILON_SQ {
        // Nearly parallel
        sc = 0.0;
        tc = if ss.abs() < EPSILON_SQ { 0.0 } else { (sw / ss).clamp(0.0, 1.0) };
    } else {
        let sn = (rs * sw - ss * rw) / denom;
        let tn = (rr * sw - rs * rw) / denom;

        if sn < 0.0 {
            let t = if ss.abs() < EPSILON_SQ { 0.0 } else { (sw / ss).clamp(0.0, 1.0) };
            sc = 0.0;
            tc = t;
        } else if sn > 1.0 {
            let t = if ss.abs() < EPSILON_SQ { 0.0 } else { ((sw + rs) / ss).clamp(0.0, 1.0) };
            sc = 1.0;
            tc = t;
        } else if tn < 0.0 {
            tc = 0.0;
            sc = if rr.abs() < EPSILON_SQ { 0.0 } else { (-rw / rr).clamp(0.0, 1.0) };
        } else if tn > 1.0 {
            tc = 1.0;
            sc = if rr.abs() < EPSILON_SQ { 0.0 } else { ((rs - rw) / rr).clamp(0.0, 1.0) };
        } else {
            sc = sn;
            tc = tn;
        }
    }

    (a + r * sc, c + s * tc)
}

/// Capsule vs Capsule: closest points between two segments, then sphere-sphere test.
#[allow(clippy::too_many_arguments)]
fn capsule_capsule_3d(
    pos_a: DVec3,
    rot_a: DQuat,
    hh_a: f64,
    cr_a: f64,
    pos_b: DVec3,
    rot_b: DQuat,
    hh_b: f64,
    cr_b: f64,
) -> Option<(DVec3, f64, DVec3)> {
    let (a1, a2) = capsule_endpoints_3d(pos_a, rot_a, hh_a);
    let (b1, b2) = capsule_endpoints_3d(pos_b, rot_b, hh_b);
    let (pa, pb) = closest_points_segments_3d(a1, a2, b1, b2);
    sphere_sphere(pa, cr_a, pb, cr_b)
}

/// Capsule vs Box: find closest point on capsule segment to box, then sphere-OBB test.
fn capsule_box_3d(
    cap_pos: DVec3,
    cap_rot: DQuat,
    half_height: f64,
    cap_radius: f64,
    box_pos: DVec3,
    box_rot: DQuat,
    half_extents: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    let (ep_a, ep_b) = capsule_endpoints_3d(cap_pos, cap_rot, half_height);

    // Transform capsule endpoints into box-local space
    let inv_rot = box_rot.inverse();
    let local_a = inv_rot * (ep_a - box_pos);
    let local_b = inv_rot * (ep_b - box_pos);

    // Find the point on the capsule segment closest to the box in local space.
    // We find the closest point on the segment to the box by clamping.
    let ab = local_b - local_a;
    let len_sq = ab.dot(ab);

    // Sample along the segment and find the parameter t that minimizes distance to the box
    let best_t = if len_sq < EPSILON_SQ {
        0.0
    } else {
        // Analytical: project box center (origin in local space) onto segment, clamp
        let t_center = (-local_a).dot(ab) / len_sq;
        t_center.clamp(0.0, 1.0)
    };

    let closest_on_seg = local_a + ab * best_t;
    // The closest point on the capsule segment in world space
    let world_seg_pt = box_rot * closest_on_seg + box_pos;

    // Now do a sphere-OBB test with the sphere centered at the closest segment point
    sphere_obb(world_seg_pt, cap_radius, box_pos, box_rot, half_extents)
}

/// OBB vs OBB using SAT with 6 face normals (3 per box).
fn obb_obb_3d(
    pos_a: DVec3,
    rot_a: DQuat,
    he_a: DVec3,
    pos_b: DVec3,
    rot_b: DQuat,
    he_b: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    // Get the 3 local axes for each box
    let axes_a = [rot_a * DVec3::X, rot_a * DVec3::Y, rot_a * DVec3::Z];
    let axes_b = [rot_b * DVec3::X, rot_b * DVec3::Y, rot_b * DVec3::Z];
    let he_a_arr = [he_a.x, he_a.y, he_a.z];
    let he_b_arr = [he_b.x, he_b.y, he_b.z];

    let d = pos_b - pos_a;

    let mut min_overlap = f64::INFINITY;
    let mut best_axis = DVec3::ZERO;

    // Test 6 face normals (3 per box)
    // Test all 6 face normals (3 per box)
    let all_axes = [axes_a[0], axes_a[1], axes_a[2], axes_b[0], axes_b[1], axes_b[2]];

    for axis in &all_axes {
        // Project half-extents of both boxes onto this axis
        let proj_a = he_a_arr[0] * axes_a[0].dot(*axis).abs()
            + he_a_arr[1] * axes_a[1].dot(*axis).abs()
            + he_a_arr[2] * axes_a[2].dot(*axis).abs();
        let proj_b = he_b_arr[0] * axes_b[0].dot(*axis).abs()
            + he_b_arr[1] * axes_b[1].dot(*axis).abs()
            + he_b_arr[2] * axes_b[2].dot(*axis).abs();

        let dist = d.dot(*axis).abs();
        let overlap = proj_a + proj_b - dist;

        if overlap <= 0.0 {
            return None; // Separating axis found
        }

        if overlap < min_overlap {
            min_overlap = overlap;
            best_axis = *axis;
            // Ensure normal points from A to B
            if d.dot(best_axis) < 0.0 {
                best_axis = -best_axis;
            }
        }
    }

    // Contact point: midpoint of the overlap region projected onto the separating axis
    let point = pos_a + best_axis * (he_a.x * axes_a[0].dot(best_axis).abs()
        + he_a.y * axes_a[1].dot(best_axis).abs()
        + he_a.z * axes_a[2].dot(best_axis).abs());

    // Better contact point: average of the face centers along the normal
    let face_a = pos_a + best_axis * (he_a.x * axes_a[0].dot(best_axis)
        + he_a.y * axes_a[1].dot(best_axis)
        + he_a.z * axes_a[2].dot(best_axis));
    let face_b = pos_b - best_axis * (he_b.x * axes_b[0].dot(best_axis)
        + he_b.y * axes_b[1].dot(best_axis)
        + he_b.z * axes_b[2].dot(best_axis));
    let contact_point = (face_a + face_b) * 0.5;

    let _ = point;

    Some((best_axis, min_overlap, contact_point))
}

/// Segment vs Sphere: closest point on segment to sphere center.
fn segment_sphere_3d(
    seg_pos: DVec3,
    seg_rot: DQuat,
    local_a: DVec3,
    local_b: DVec3,
    sphere_pos: DVec3,
    radius: f64,
) -> Option<(DVec3, f64, DVec3)> {
    let wa = seg_pos + seg_rot * local_a;
    let wb = seg_pos + seg_rot * local_b;
    let closest = closest_point_on_segment_3d(wa, wb, sphere_pos);
    // Treat segment as zero-radius, then test against sphere
    sphere_sphere(closest, 0.0, sphere_pos, radius)
}

/// Segment vs Box: closest point on segment to box surface.
fn segment_box_3d(
    seg_pos: DVec3,
    seg_rot: DQuat,
    local_a: DVec3,
    local_b: DVec3,
    box_pos: DVec3,
    box_rot: DQuat,
    half_extents: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    let wa = seg_pos + seg_rot * local_a;
    let wb = seg_pos + seg_rot * local_b;

    // Transform segment into box-local space
    let inv_rot = box_rot.inverse();
    let la = inv_rot * (wa - box_pos);
    let lb = inv_rot * (wb - box_pos);

    // Find closest point on segment to box (clamped to box surface)
    let ab = lb - la;
    let len_sq = ab.dot(ab);

    // Sample several points along the segment to find best contact
    let steps = 8;
    let mut best_depth = f64::NEG_INFINITY;
    let mut best_normal = DVec3::ZERO;
    let mut best_point = DVec3::ZERO;

    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let seg_pt = la + ab * t;
        let clamped = seg_pt.clamp(-half_extents, half_extents);
        let diff = seg_pt - clamped;
        let dist_sq = diff.dot(diff);

        if dist_sq < EPSILON_SQ {
            // Point is inside the box
            let face_dists = DVec3::new(
                half_extents.x - seg_pt.x.abs(),
                half_extents.y - seg_pt.y.abs(),
                half_extents.z - seg_pt.z.abs(),
            );
            let min_axis = if face_dists.x <= face_dists.y && face_dists.x <= face_dists.z {
                0
            } else if face_dists.y <= face_dists.z {
                1
            } else {
                2
            };
            let depth = face_dists[min_axis];
            if depth > best_depth {
                best_depth = depth;
                let mut n = DVec3::ZERO;
                n[min_axis] = if seg_pt[min_axis] >= 0.0 { 1.0 } else { -1.0 };
                best_normal = n;
                best_point = clamped;
            }
        } else {
            let dist = dist_sq.sqrt();
            // Negative depth means penetration — segment point is outside
            let depth = -dist;
            if depth > best_depth && dist_sq < EPSILON {
                best_depth = depth;
            }
        }
    }

    // Also check: closest point on segment to box center
    let t_center = if len_sq < EPSILON_SQ {
        0.0
    } else {
        (-la).dot(ab) / len_sq
    };
    let t_center = t_center.clamp(0.0, 1.0);
    let seg_at_center = la + ab * t_center;
    let clamped = seg_at_center.clamp(-half_extents, half_extents);
    let diff = seg_at_center - clamped;
    let dist_sq = diff.dot(diff);

    if dist_sq < EPSILON_SQ {
        let face_dists = DVec3::new(
            half_extents.x - seg_at_center.x.abs(),
            half_extents.y - seg_at_center.y.abs(),
            half_extents.z - seg_at_center.z.abs(),
        );
        let min_axis = if face_dists.x <= face_dists.y && face_dists.x <= face_dists.z {
            0
        } else if face_dists.y <= face_dists.z {
            1
        } else {
            2
        };
        let depth = face_dists[min_axis];
        if depth > best_depth {
            best_depth = depth;
            let mut n = DVec3::ZERO;
            n[min_axis] = if seg_at_center[min_axis] >= 0.0 { 1.0 } else { -1.0 };
            best_normal = n;
            best_point = clamped;
        }
    }

    if best_depth <= 0.0 {
        return None;
    }

    // Transform back to world space
    let normal = box_rot * best_normal;
    let point = box_rot * best_point + box_pos;
    Some((normal, best_depth, point))
}

/// ConvexHull vs Sphere: closest point on hull surface to sphere center.
fn convex_hull_sphere_3d(
    hull_points: &[[f64; 3]],
    hull_pos: DVec3,
    hull_rot: DQuat,
    sphere_pos: DVec3,
    radius: f64,
) -> Option<(DVec3, f64, DVec3)> {
    if hull_points.len() < 3 {
        return None;
    }

    // Transform hull points to world space
    let world_pts: Vec<DVec3> = hull_points
        .iter()
        .map(|p| hull_pos + hull_rot * DVec3::from_array(*p))
        .collect();

    // Find closest point on hull to sphere center.
    // For a convex hull, we test all edges (treating the hull as a wireframe).
    // This is a v1.0 approximation — works well for sphere tests.
    let n = world_pts.len();
    let mut best_dist_sq = f64::INFINITY;
    let mut best_closest = world_pts[0];

    for i in 0..n {
        let a = world_pts[i];
        let b = world_pts[(i + 1) % n];
        let closest = closest_point_on_segment_3d(a, b, sphere_pos);
        let dist_sq = (sphere_pos - closest).dot(sphere_pos - closest);
        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_closest = closest;
        }
    }

    if best_dist_sq >= radius * radius {
        return None;
    }

    let diff = sphere_pos - best_closest;
    let dist = best_dist_sq.sqrt();

    let (normal, depth) = if dist < EPSILON {
        (DVec3::Y, radius)
    } else {
        (diff / dist, radius - dist)
    };

    Some((normal, depth, best_closest))
}

// ---------------------------------------------------------------------------
// Ray helpers
// ---------------------------------------------------------------------------

fn ray_sphere(
    origin: DVec3,
    dir: DVec3,
    center: DVec3,
    radius: f64,
) -> Option<(f64, DVec3)> {
    let oc = origin - center;
    let half_b = oc.dot(dir);
    let c = oc.dot(oc) - radius * radius;
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

    let point = origin + dir * t;
    let normal = (point - center).normalize_or(DVec3::Y);
    Some((t, normal))
}

fn ray_aabb_3d(
    origin: DVec3,
    dir: DVec3,
    min: DVec3,
    max: DVec3,
) -> Option<(f64, DVec3)> {
    let mut t_min = f64::NEG_INFINITY;
    let mut t_max = f64::INFINITY;
    let mut normal = DVec3::ZERO;

    for i in 0..3 {
        if dir[i].abs() < EPSILON {
            if origin[i] < min[i] || origin[i] > max[i] {
                return None;
            }
        } else {
            let inv_d = 1.0 / dir[i];
            let mut t1 = (min[i] - origin[i]) * inv_d;
            let mut t2 = (max[i] - origin[i]) * inv_d;
            let mut n = DVec3::ZERO;
            n[i] = -1.0;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
                n[i] = 1.0;
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

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-6;

    #[test]
    fn sphere_sphere_overlap() {
        let r = sphere_sphere(DVec3::ZERO, 1.0, DVec3::new(1.5, 0.0, 0.0), 1.0);
        assert!(r.is_some());
        let (n, d, _) = r.unwrap();
        assert!((n.x - 1.0).abs() < EPS);
        assert!((d - 0.5).abs() < EPS);
    }

    #[test]
    fn sphere_sphere_no_overlap() {
        assert!(sphere_sphere(DVec3::ZERO, 1.0, DVec3::new(5.0, 0.0, 0.0), 1.0).is_none());
    }

    #[test]
    fn sphere_obb_overlap() {
        let r = sphere_obb(
            DVec3::new(1.8, 0.0, 0.0),
            0.5,
            DVec3::ZERO,
            DQuat::IDENTITY,
            DVec3::new(1.5, 1.0, 1.0),
        );
        assert!(r.is_some());
    }

    #[test]
    fn sphere_obb_miss() {
        assert!(sphere_obb(
            DVec3::new(5.0, 0.0, 0.0),
            0.5,
            DVec3::ZERO,
            DQuat::IDENTITY,
            DVec3::new(1.0, 1.0, 1.0),
        )
        .is_none());
    }

    #[test]
    fn sphere_obb_rotated() {
        // Box rotated 45° around Z — a sphere that would miss an AABB should hit the rotated box
        let rot = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_4);
        // Box half_extents [2, 0.5, 1], rotated 45° around Z.
        // The corner extends to sqrt(2^2 + 0.5^2) ≈ 2.06 along the diagonal.
        let r = sphere_obb(
            DVec3::new(1.5, 1.5, 0.0),
            0.5,
            DVec3::ZERO,
            rot,
            DVec3::new(2.0, 0.5, 1.0),
        );
        assert!(r.is_some());
    }

    #[test]
    fn aabb_3d_overlap() {
        let r = aabb_aabb_3d(
            DVec3::ZERO,
            DVec3::ONE,
            DVec3::new(1.5, 0.0, 0.0),
            DVec3::ONE,
        );
        assert!(r.is_some());
        let (n, d, _) = r.unwrap();
        assert!((n.x - 1.0).abs() < EPS);
        assert!((d - 0.5).abs() < EPS);
    }

    #[test]
    fn aabb_3d_no_overlap() {
        assert!(aabb_aabb_3d(
            DVec3::ZERO,
            DVec3::ONE,
            DVec3::new(5.0, 0.0, 0.0),
            DVec3::ONE,
        )
        .is_none());
    }

    #[test]
    fn ray_sphere_hit() {
        let r = ray_sphere(DVec3::ZERO, DVec3::X, DVec3::new(5.0, 0.0, 0.0), 1.0);
        assert!(r.is_some());
        let (t, _) = r.unwrap();
        assert!((t - 4.0).abs() < EPS);
    }

    #[test]
    fn ray_sphere_miss() {
        assert!(ray_sphere(DVec3::ZERO, DVec3::Y, DVec3::new(5.0, 0.0, 0.0), 1.0).is_none());
    }

    #[test]
    fn ray_aabb_3d_hit() {
        let r = ray_aabb_3d(
            DVec3::ZERO,
            DVec3::X,
            DVec3::new(4.0, -1.0, -1.0),
            DVec3::new(6.0, 1.0, 1.0),
        );
        assert!(r.is_some());
        let (t, _) = r.unwrap();
        assert!((t - 4.0).abs() < EPS);
    }

    #[test]
    fn quaternion_identity_rotation() {
        let q = DQuat::IDENTITY;
        let v = DVec3::X;
        let result = q * v;
        assert!((result.x - 1.0).abs() < EPS);
        assert!(result.y.abs() < EPS);
        assert!(result.z.abs() < EPS);
    }

    #[test]
    fn quaternion_z_rotation() {
        let q = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
        let v = DVec3::X;
        let result = q * v;
        assert!(result.x.abs() < EPS);
        assert!((result.y - 1.0).abs() < EPS);
        assert!(result.z.abs() < EPS);
    }

    #[test]
    fn gravity_moves_body_3d() {
        let mut state = PhysicsState3d::new();
        let bh = state.add_body(
            &BodyDesc {
                body_type: BodyType::Dynamic,
                position: [0.0, 10.0, 0.0],
                ..BodyDesc::default()
            },
        );
        state.add_collider(
            bh,
            &ColliderDesc {
                shape: ColliderShape::Ball { radius: 0.5 },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );

        for _ in 0..60 {
            state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        }

        assert!(state.bodies.get(body_ah(bh)).unwrap().position.y < 10.0, "body should fall");
    }

    #[test]
    fn sphere_collision_3d() {
        let mut state = PhysicsState3d::new();

        let floor = state.add_body(
            &BodyDesc {
                body_type: BodyType::Static,
                position: [0.0, 0.0, 0.0],
                ..BodyDesc::default()
            },
        );
        state.add_collider(
            floor,
            &ColliderDesc {
                shape: ColliderShape::Box {
                    half_extents: [50.0, 0.5, 50.0],
                },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );

        let ball = state.add_body(
            &BodyDesc {
                body_type: BodyType::Dynamic,
                position: [0.0, 2.0, 0.0],
                ..BodyDesc::default()
            },
        );
        state.add_collider(
            ball,
            &ColliderDesc {
                shape: ColliderShape::Ball { radius: 0.5 },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );

        let mut found_event = false;
        for _ in 0..120 {
            let events = state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
            if !events.is_empty() {
                found_event = true;
                break;
            }
        }
        assert!(found_event, "should generate collision events");
    }

    #[test]
    fn raycast_3d() {
        let mut state = PhysicsState3d::new();
        let bh = state.add_body(
            &BodyDesc {
                body_type: BodyType::Static,
                position: [5.0, 0.0, 0.0],
                ..BodyDesc::default()
            },
        );
        state.add_collider(
            bh,
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

        let hit = state.raycast([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 100.0);
        assert!(hit.is_some());
        let hit = hit.unwrap();
        assert!((hit.distance - 4.0).abs() < 0.1);
    }

    #[test]
    fn body_count_3d() {
        let mut state = PhysicsState3d::new();
        assert_eq!(state.body_count(), 0);
        state.add_body(&BodyDesc::default());
        assert_eq!(state.body_count(), 1);
        state.remove_body(BodyHandle(0));
        assert_eq!(state.body_count(), 0);
    }

    #[test]
    fn sphere_mass_3d() {
        let c = Collider3d::from_desc(
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
        let expected = (4.0 / 3.0) * std::f64::consts::PI;
        assert!((m - expected).abs() < EPS);
    }

    #[test]
    fn box_mass_3d() {
        let c = Collider3d::from_desc(
            ColliderHandle(0),
            BodyHandle(0),
            &ColliderDesc {
                shape: ColliderShape::Box { half_extents: [1.0, 1.0, 1.0] },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        assert!((c.compute_mass() - 8.0).abs() < EPS);
    }

    #[test]
    fn multiple_colliders_accumulate_mass_3d() {
        let mut state = PhysicsState3d::new();
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
        let mass_first = state.bodies.get(body_ah(bh)).unwrap().mass;

        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [2.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        assert!((state.bodies.get(body_ah(bh)).unwrap().mass - 2.0 * mass_first).abs() < EPS);
    }

    #[test]
    fn capsule_sphere_3d_overlap() {
        let r = capsule_sphere_3d(DVec3::ZERO, DQuat::IDENTITY, 1.0, 0.5, DVec3::new(0.8, 0.0, 0.0), 0.5);
        assert!(r.is_some());
    }

    #[test]
    fn capsule_sphere_3d_miss() {
        assert!(capsule_sphere_3d(DVec3::ZERO, DQuat::IDENTITY, 1.0, 0.5, DVec3::new(5.0, 0.0, 0.0), 0.5).is_none());
    }

    #[test]
    fn impulse_changes_velocity_3d() {
        let mut state = PhysicsState3d::new();
        let bh = state.add_body(&BodyDesc::default());
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        state.apply_impulse(bh, &Impulse::new(10.0, 0.0, 0.0));
        assert!(state.bodies.get(body_ah(bh)).unwrap().linear_velocity.x > 0.0);
    }

    #[test]
    fn remove_cleans_collision_pairs_3d() {
        let mut state = PhysicsState3d::new();

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

        state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        assert!(!state.prev_collision_pairs.is_empty());

        state.remove_body(b);
        assert!(state.prev_collision_pairs.is_empty());
    }

    #[test]
    fn fixed_joint_3d() {
        let mut state = PhysicsState3d::new();
        let a = BodyHandle(0);
        let b = state.add_body(&BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 5.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 3.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        state.add_joint(&JointDesc {
            body_a: a,
            body_b: b,
            joint_type: JointType::Fixed,
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
            damping: 0.0,
        });

        for _ in 0..10 {
            state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1, 0.01, 0.2, 100.0);
        }
        // Joint should prevent body from falling far
        assert!(state.bodies.get(body_ah(b)).unwrap().position.y > 2.0);
    }

    #[test]
    fn spatial_hash_3d_finds_pair() {
        let mut grid: SpatialHashGrid<ColliderHandle> = SpatialHashGrid::new(2.0);
        grid.insert_3d(ColliderHandle(0), [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]);
        grid.insert_3d(ColliderHandle(1), [0.5, 0.5, 0.5], [1.5, 1.5, 1.5]);
        let pairs = grid.query_pairs();
        assert!(pairs.contains(&(ColliderHandle(0), ColliderHandle(1))));
    }

    #[test]
    fn spatial_hash_3d_no_false_pair() {
        let mut grid: SpatialHashGrid<ColliderHandle> = SpatialHashGrid::new(1.0);
        grid.insert_3d(ColliderHandle(0), [0.0, 0.0, 0.0], [0.5, 0.5, 0.5]);
        grid.insert_3d(ColliderHandle(1), [10.0, 10.0, 10.0], [10.5, 10.5, 10.5]);
        assert!(grid.query_pairs().is_empty());
    }

    // -----------------------------------------------------------------------
    // set_body_state / set_body_type tests
    // -----------------------------------------------------------------------

    #[test]
    fn set_body_state_teleports() {
        let mut state = PhysicsState3d::new();
        let bh = state.add_body(&BodyDesc::default());
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        let new_state = BodyState {
            handle: bh,
            body_type: BodyType::Dynamic,
            position: [10.0, 20.0, 30.0],
            rotation: 1.0,
            linear_velocity: [1.0, 2.0, 3.0],
            angular_velocity: 0.5,
            is_sleeping: false,
        };
        state.set_body_state(bh, &new_state).unwrap();

        let rb = &state.bodies.get(body_ah(bh)).unwrap();
        assert!((rb.position.x - 10.0).abs() < EPS);
        assert!((rb.position.y - 20.0).abs() < EPS);
        assert!((rb.position.z - 30.0).abs() < EPS);
        assert!(!rb.is_sleeping);
    }

    #[test]
    fn set_body_state_not_found() {
        let mut state = PhysicsState3d::new();
        let result = state.set_body_state(BodyHandle(999), &BodyState {
            handle: BodyHandle(999),
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            rotation: 0.0,
            linear_velocity: [0.0, 0.0, 0.0],
            angular_velocity: 0.0,
            is_sleeping: false,
        });
        assert!(result.is_err());
    }

    #[test]
    fn set_body_type_dynamic_to_static() {
        let mut state = PhysicsState3d::new();
        let bh = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            linear_velocity: [5.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        state.set_body_type(bh, BodyType::Static).unwrap();
        let rb = &state.bodies.get(body_ah(bh)).unwrap();
        assert_eq!(rb.body_type, BodyType::Static);
        assert_eq!(rb.inv_mass, 0.0);
        assert_eq!(rb.linear_velocity, DVec3::ZERO);
    }

    #[test]
    fn set_body_type_static_to_dynamic() {
        let mut state = PhysicsState3d::new();
        let bh = state.add_body(&BodyDesc {
            body_type: BodyType::Dynamic,
            ..BodyDesc::default()
        });
        state.add_collider(bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        let mass_before = state.bodies.get(body_ah(bh)).unwrap().mass;
        assert!(mass_before > 0.0);

        // Switch to static and back
        state.set_body_type(bh, BodyType::Static).unwrap();
        state.set_body_type(bh, BodyType::Dynamic).unwrap();
        let rb = &state.bodies.get(body_ah(bh)).unwrap();
        assert!(rb.inv_mass > 0.0);
        assert_eq!(rb.body_type, BodyType::Dynamic);
    }

    // -----------------------------------------------------------------------
    // OBB-OBB tests
    // -----------------------------------------------------------------------

    #[test]
    fn obb_obb_aligned_overlap() {
        // Two axis-aligned boxes overlapping
        let r = obb_obb_3d(
            DVec3::ZERO, DQuat::IDENTITY, DVec3::ONE,
            DVec3::new(1.5, 0.0, 0.0), DQuat::IDENTITY, DVec3::ONE,
        );
        assert!(r.is_some());
        let (n, d, _) = r.unwrap();
        assert!((n.x - 1.0).abs() < EPS);
        assert!((d - 0.5).abs() < EPS);
    }

    #[test]
    fn obb_obb_no_overlap() {
        assert!(obb_obb_3d(
            DVec3::ZERO, DQuat::IDENTITY, DVec3::ONE,
            DVec3::new(5.0, 0.0, 0.0), DQuat::IDENTITY, DVec3::ONE,
        ).is_none());
    }

    #[test]
    fn obb_obb_rotated_overlap() {
        // One box rotated 45° around Z
        let rot = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_4);
        let r = obb_obb_3d(
            DVec3::ZERO, DQuat::IDENTITY, DVec3::ONE,
            DVec3::new(1.5, 0.0, 0.0), rot, DVec3::ONE,
        );
        assert!(r.is_some());
    }

    #[test]
    fn obb_obb_rotated_separated() {
        // One box rotated, far enough apart to not overlap
        let rot = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_4);
        assert!(obb_obb_3d(
            DVec3::ZERO, DQuat::IDENTITY, DVec3::new(0.5, 0.5, 0.5),
            DVec3::new(3.0, 0.0, 0.0), rot, DVec3::new(0.5, 0.5, 0.5),
        ).is_none());
    }

    // -----------------------------------------------------------------------
    // Capsule-Capsule tests
    // -----------------------------------------------------------------------

    #[test]
    fn capsule_capsule_overlap() {
        let r = capsule_capsule_3d(
            DVec3::ZERO, DQuat::IDENTITY, 1.0, 0.5,
            DVec3::new(0.8, 0.0, 0.0), DQuat::IDENTITY, 1.0, 0.5,
        );
        assert!(r.is_some());
    }

    #[test]
    fn capsule_capsule_miss() {
        assert!(capsule_capsule_3d(
            DVec3::ZERO, DQuat::IDENTITY, 1.0, 0.5,
            DVec3::new(5.0, 0.0, 0.0), DQuat::IDENTITY, 1.0, 0.5,
        ).is_none());
    }

    #[test]
    fn capsule_capsule_crossed() {
        // Two capsules crossing at right angles
        let rot_x = DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2);
        let r = capsule_capsule_3d(
            DVec3::ZERO, DQuat::IDENTITY, 2.0, 0.3,
            DVec3::new(0.0, 0.0, 0.0), rot_x, 2.0, 0.3,
        );
        assert!(r.is_some());
    }

    // -----------------------------------------------------------------------
    // Capsule-Box tests
    // -----------------------------------------------------------------------

    #[test]
    fn capsule_box_overlap() {
        let r = capsule_box_3d(
            DVec3::new(0.0, 0.0, 0.0), DQuat::IDENTITY, 1.0, 0.5,
            DVec3::new(1.0, 0.0, 0.0), DQuat::IDENTITY, DVec3::ONE,
        );
        assert!(r.is_some());
    }

    #[test]
    fn capsule_box_miss() {
        assert!(capsule_box_3d(
            DVec3::new(0.0, 0.0, 0.0), DQuat::IDENTITY, 1.0, 0.5,
            DVec3::new(5.0, 0.0, 0.0), DQuat::IDENTITY, DVec3::ONE,
        ).is_none());
    }

    // -----------------------------------------------------------------------
    // Segment tests
    // -----------------------------------------------------------------------

    #[test]
    fn segment_sphere_overlap() {
        let r = segment_sphere_3d(
            DVec3::ZERO, DQuat::IDENTITY,
            DVec3::new(-2.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 0.0),
            DVec3::new(0.0, 0.3, 0.0), 0.5,
        );
        assert!(r.is_some());
    }

    #[test]
    fn segment_sphere_miss() {
        assert!(segment_sphere_3d(
            DVec3::ZERO, DQuat::IDENTITY,
            DVec3::new(-2.0, 0.0, 0.0), DVec3::new(2.0, 0.0, 0.0),
            DVec3::new(0.0, 5.0, 0.0), 0.5,
        ).is_none());
    }

    #[test]
    fn segment_box_overlap() {
        // Segment passing through the center of a box
        let r = segment_box_3d(
            DVec3::ZERO, DQuat::IDENTITY,
            DVec3::new(-0.5, 0.0, 0.0), DVec3::new(0.5, 0.0, 0.0),
            DVec3::ZERO, DQuat::IDENTITY, DVec3::ONE,
        );
        assert!(r.is_some());
    }

    #[test]
    fn segment_box_miss() {
        assert!(segment_box_3d(
            DVec3::ZERO, DQuat::IDENTITY,
            DVec3::new(-0.5, 0.0, 0.0), DVec3::new(0.5, 0.0, 0.0),
            DVec3::new(5.0, 5.0, 5.0), DQuat::IDENTITY, DVec3::ONE,
        ).is_none());
    }

    // -----------------------------------------------------------------------
    // ConvexHull vs Sphere tests
    // -----------------------------------------------------------------------

    #[test]
    fn convex_hull_sphere_overlap() {
        // Triangle hull near sphere
        let points = vec![
            [-1.0, -1.0, 0.0],
            [1.0, -1.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        let r = convex_hull_sphere_3d(
            &points,
            DVec3::ZERO, DQuat::IDENTITY,
            DVec3::new(0.0, 1.2, 0.0), 0.5,
        );
        assert!(r.is_some());
    }

    #[test]
    fn convex_hull_sphere_miss() {
        let points = vec![
            [-1.0, -1.0, 0.0],
            [1.0, -1.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        assert!(convex_hull_sphere_3d(
            &points,
            DVec3::ZERO, DQuat::IDENTITY,
            DVec3::new(0.0, 5.0, 0.0), 0.5,
        ).is_none());
    }

    // -----------------------------------------------------------------------
    // Closest points between segments
    // -----------------------------------------------------------------------

    #[test]
    fn closest_points_parallel_segments() {
        let (p1, p2) = closest_points_segments_3d(
            DVec3::new(0.0, 0.0, 0.0), DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(0.0, 1.0, 0.0), DVec3::new(1.0, 1.0, 0.0),
        );
        // Closest points should be on the same x coordinate
        assert!((p1.y).abs() < EPS);
        assert!((p2.y - 1.0).abs() < EPS);
    }

    #[test]
    fn closest_points_crossing_segments() {
        let (p1, p2) = closest_points_segments_3d(
            DVec3::new(-1.0, 0.0, 0.0), DVec3::new(1.0, 0.0, 0.0),
            DVec3::new(0.0, -1.0, 1.0), DVec3::new(0.0, 1.0, 1.0),
        );
        // p1 should be at origin (0,0,0) and p2 at (0,0,1)
        assert!((p1 - DVec3::new(0.0, 0.0, 0.0)).length() < EPS);
        assert!((p2 - DVec3::new(0.0, 0.0, 1.0)).length() < EPS);
    }
}
