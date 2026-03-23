//! Native 3D physics backend.
//!
//! Implements broadphase (spatial hash), narrowphase (shape-vs-shape contact
//! generation), and a sequential impulse constraint solver with friction and
//! angular response. All geometry uses f64 precision.

use std::collections::{HashMap, HashSet};

use hisab::{DQuat, DVec3};

use crate::body::{BodyDesc, BodyHandle, BodyState, BodyType};
use crate::collider::{ColliderDesc, ColliderHandle, ColliderShape};
use crate::event::CollisionEvent;
use crate::force::{Force, Impulse, Torque};
use crate::joint::{JointDesc, JointHandle, JointMotor, JointType};
use crate::material::PhysicsMaterial;
use crate::query::RayHit;
use crate::ImpetusError;

// ---------------------------------------------------------------------------
// Sleep / deactivation thresholds
// ---------------------------------------------------------------------------

/// Bodies with both linear and angular speed below this are candidates for sleep.
const SLEEP_VELOCITY_THRESHOLD_3D: f64 = 0.01;
/// How many seconds of low motion before a body is put to sleep.
const SLEEP_TIME_THRESHOLD_3D: f64 = 0.5;

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

    fn integrate_velocities(&mut self, gravity: DVec3, dt: f64) {
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
            return m.max(1e-6);
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
        (vol * self.material.density).max(1e-6)
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
                // Approximate as cylinder
                let r2 = radius * radius;
                let h = 2.0 * half_height;
                let ix = mass * (3.0 * r2 + h * h) / 12.0;
                let iy = ix;
                let iz = mass * r2 / 2.0;
                DVec3::new(ix, iy, iz)
            }
            _ => DVec3::splat(mass),
        };
        i.max(DVec3::splat(1e-10))
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
// Spatial hash (3D cells)
// ---------------------------------------------------------------------------

struct SpatialHash3d {
    inv_cell_size: f64,
    cells: HashMap<(i32, i32, i32), Vec<ColliderHandle>>,
}

impl SpatialHash3d {
    fn new(cell_size: f64) -> Self {
        Self {
            inv_cell_size: 1.0 / cell_size,
            cells: HashMap::new(),
        }
    }

    fn auto_cell_size(aabbs: &[(ColliderHandle, Aabb3d)]) -> f64 {
        if aabbs.is_empty() {
            return 1.0;
        }
        let total: f64 = aabbs
            .iter()
            .map(|(_, aabb)| {
                let size = aabb.max - aabb.min;
                size.x.max(size.y).max(size.z)
            })
            .sum();
        (total / aabbs.len() as f64 * 2.0).max(0.1)
    }

    fn cell(&self, x: f64, y: f64, z: f64) -> (i32, i32, i32) {
        (
            (x * self.inv_cell_size).floor() as i32,
            (y * self.inv_cell_size).floor() as i32,
            (z * self.inv_cell_size).floor() as i32,
        )
    }

    fn insert(&mut self, handle: ColliderHandle, aabb: &Aabb3d) {
        let (min_cx, min_cy, min_cz) = self.cell(aabb.min.x, aabb.min.y, aabb.min.z);
        let (max_cx, max_cy, max_cz) = self.cell(aabb.max.x, aabb.max.y, aabb.max.z);

        for cx in min_cx..=max_cx {
            for cy in min_cy..=max_cy {
                for cz in min_cz..=max_cz {
                    self.cells.entry((cx, cy, cz)).or_default().push(handle);
                }
            }
        }
    }

    fn query_pairs(&self) -> HashSet<(ColliderHandle, ColliderHandle)> {
        let mut pairs = HashSet::new();
        for cell in self.cells.values() {
            for i in 0..cell.len() {
                for j in (i + 1)..cell.len() {
                    let a = cell[i];
                    let b = cell[j];
                    if a.0 < b.0 {
                        pairs.insert((a, b));
                    } else {
                        pairs.insert((b, a));
                    }
                }
            }
        }
        pairs
    }
}

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
    pub bodies: HashMap<BodyHandle, RigidBody3d>,
    pub colliders: HashMap<ColliderHandle, Collider3d>,
    pub joints: HashMap<JointHandle, Joint3d>,
    pub body_colliders: HashMap<BodyHandle, Vec<ColliderHandle>>,
    prev_collision_pairs: HashSet<(ColliderHandle, ColliderHandle)>,
}

impl PhysicsState3d {
    pub fn new() -> Self {
        Self {
            bodies: HashMap::new(),
            colliders: HashMap::new(),
            joints: HashMap::new(),
            body_colliders: HashMap::new(),
            prev_collision_pairs: HashSet::new(),
        }
    }

    pub fn add_body(&mut self, handle: BodyHandle, desc: &BodyDesc) {
        self.bodies
            .insert(handle, RigidBody3d::from_desc(handle, desc));
        self.body_colliders.insert(handle, Vec::new());
    }

    pub fn add_collider(&mut self, handle: ColliderHandle, body: BodyHandle, desc: &ColliderDesc) {
        let collider = Collider3d::from_desc(handle, body, desc);

        if let Some(rb) = self.bodies.get_mut(&body)
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

        self.body_colliders.entry(body).or_default().push(handle);
        self.colliders.insert(handle, collider);
    }

    pub fn add_joint(&mut self, handle: JointHandle, desc: &JointDesc) {
        self.joints.insert(
            handle,
            Joint3d {
                body_a: desc.body_a,
                body_b: desc.body_b,
                joint_type: desc.joint_type.clone(),
                local_anchor_a: DVec3::new(desc.local_anchor_a[0], desc.local_anchor_a[1], 0.0),
                local_anchor_b: DVec3::new(desc.local_anchor_b[0], desc.local_anchor_b[1], 0.0),
                motor: desc.motor.clone(),
            },
        );
    }

    pub fn apply_force(&mut self, body: BodyHandle, force: &Force) {
        if let Some(rb) = self.bodies.get_mut(&body) {
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
        if let Some(rb) = self.bodies.get_mut(&body)
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
        if let Some(rb) = self.bodies.get_mut(&body) {
            // Wake the body
            rb.is_sleeping = false;
            rb.sleep_timer = 0.0;
            rb.torque_accumulator.z += torque.value;
        }
    }

    pub fn remove_body(&mut self, handle: BodyHandle) {
        self.bodies.remove(&handle);
        if let Some(collider_handles) = self.body_colliders.remove(&handle) {
            for ch in &collider_handles {
                self.colliders.remove(ch);
            }
            self.prev_collision_pairs
                .retain(|(a, b)| !collider_handles.contains(a) && !collider_handles.contains(b));
        }
        self.joints
            .retain(|_, j| j.body_a != handle && j.body_b != handle);
    }

    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    pub fn get_body_state(&self, handle: BodyHandle) -> Result<BodyState, ImpetusError> {
        let rb = self
            .bodies
            .get(&handle)
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

    // -----------------------------------------------------------------------
    // Step
    // -----------------------------------------------------------------------

    pub fn step(
        &mut self,
        gravity: [f64; 3],
        dt: f64,
        velocity_iterations: u32,
        position_iterations: u32,
    ) -> Vec<CollisionEvent> {
        let g = DVec3::from_array(gravity);
        // 1. Integrate velocities
        for rb in self.bodies.values_mut() {
            rb.integrate_velocities(g, dt);
        }

        // 2-3. Broadphase + narrowphase
        let broad_pairs = self.broadphase();
        let contacts = self.narrowphase(&broad_pairs);

        // 4. Wake sleeping bodies on contact with non-sleeping moving bodies
        for contact in &contacts {
            let a_sleeping = self
                .bodies
                .get(&contact.body_a)
                .is_some_and(|b| b.is_sleeping);
            let b_sleeping = self
                .bodies
                .get(&contact.body_b)
                .is_some_and(|b| b.is_sleeping);
            let a_moving = self.bodies.get(&contact.body_a).is_some_and(|b| {
                !b.is_sleeping
                    && b.is_dynamic()
                    && (b.linear_velocity.length() > SLEEP_VELOCITY_THRESHOLD_3D
                        || b.angular_velocity.length() > SLEEP_VELOCITY_THRESHOLD_3D)
            });
            let b_moving = self.bodies.get(&contact.body_b).is_some_and(|b| {
                !b.is_sleeping
                    && b.is_dynamic()
                    && (b.linear_velocity.length() > SLEEP_VELOCITY_THRESHOLD_3D
                        || b.angular_velocity.length() > SLEEP_VELOCITY_THRESHOLD_3D)
            });
            if a_sleeping
                && b_moving
                && let Some(ba) = self.bodies.get_mut(&contact.body_a)
            {
                ba.is_sleeping = false;
                ba.sleep_timer = 0.0;
            }
            if b_sleeping
                && a_moving
                && let Some(bb) = self.bodies.get_mut(&contact.body_b)
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
        self.solve_positions(&contacts, position_iterations);

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
                let rb = self.bodies.get(&c.body)?;
                Some((c.handle, c.world_aabb(rb.position, rb.rotation)))
            })
            .collect();

        let cell_size = SpatialHash3d::auto_cell_size(&collider_aabbs);
        let mut grid = SpatialHash3d::new(cell_size);
        for (handle, aabb) in &collider_aabbs {
            grid.insert(*handle, aabb);
        }

        let candidates = grid.query_pairs();
        let aabb_map: HashMap<ColliderHandle, Aabb3d> = collider_aabbs.into_iter().collect();

        let mut pairs = Vec::with_capacity(candidates.len());
        for (ha, hb) in candidates {
            let ca = match self.colliders.get(&ha) {
                Some(c) => c,
                None => continue,
            };
            let cb = match self.colliders.get(&hb) {
                Some(c) => c,
                None => continue,
            };
            if ca.body == cb.body {
                continue;
            }
            if let (Some(ba), Some(bb)) = (self.bodies.get(&ca.body), self.bodies.get(&cb.body))
                && ba.is_static() && bb.is_static()
            {
                continue;
            }
            if ca.is_sensor && cb.is_sensor {
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
            let ca = match self.colliders.get(ha) {
                Some(c) => c,
                None => continue,
            };
            let cb = match self.colliders.get(hb) {
                Some(c) => c,
                None => continue,
            };
            let ba = match self.bodies.get(&ca.body) {
                Some(b) => b,
                None => continue,
            };
            let bb = match self.bodies.get(&cb.body) {
                Some(b) => b,
                None => continue,
            };

            let pos_a = ba.position + ba.rotation * ca.offset;
            let pos_b = bb.position + bb.rotation * cb.offset;

            if let Some((normal, depth, point)) =
                generate_contact_3d(&ca.shape, pos_a, &cb.shape, pos_b)
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
                    self.colliders.get(&c.collider_a),
                    self.colliders.get(&c.collider_b),
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
                    let ba = match self.bodies.get(&contact.body_a) {
                        Some(b) => b,
                        None => continue,
                    };
                    (ba.inv_mass, ba.inv_inertia, ba.linear_velocity, ba.angular_velocity, ba.position)
                };
                let (inv_mass_b, inv_inertia_b, vel_b, angvel_b, pos_b) = {
                    let bb = match self.bodies.get(&contact.body_b) {
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

                if let Some(ba) = self.bodies.get_mut(&contact.body_a)
                    && ba.is_dynamic()
                {
                    ba.linear_velocity -= impulse_n * ba.inv_mass;
                    let ang_imp = ra.cross(impulse_n);
                    ba.angular_velocity -= ang_imp * ba.inv_inertia;
                }
                if let Some(bb) = self.bodies.get_mut(&contact.body_b)
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
                    if tangent_speed > 1e-10 {
                        let tangent = tangent_vel / tangent_speed;
                        let jt = (-tangent_speed / inv_mass_sum)
                            .clamp(-j.abs() * friction, j.abs() * friction);
                        let impulse_t = tangent * jt;

                        if let Some(ba) = self.bodies.get_mut(&contact.body_a)
                            && ba.is_dynamic()
                        {
                            ba.linear_velocity -= impulse_t * ba.inv_mass;
                            let ang_t = ra.cross(impulse_t);
                            ba.angular_velocity -= ang_t * ba.inv_inertia;
                        }
                        if let Some(bb) = self.bodies.get_mut(&contact.body_b)
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

    fn solve_positions(&mut self, contacts: &[Contact3d], iterations: u32) {
        let slop = 0.01;
        let percent = 0.2;

        for _ in 0..iterations {
            for contact in contacts {
                let is_sensor = match (
                    self.colliders.get(&contact.collider_a),
                    self.colliders.get(&contact.collider_b),
                ) {
                    (Some(a), Some(b)) => a.is_sensor || b.is_sensor,
                    _ => false,
                };
                if is_sensor {
                    continue;
                }

                let inv_mass_a = self.bodies.get(&contact.body_a).map(|b| b.inv_mass).unwrap_or(0.0);
                let inv_mass_b = self.bodies.get(&contact.body_b).map(|b| b.inv_mass).unwrap_or(0.0);
                let inv_mass_sum = inv_mass_a + inv_mass_b;
                if inv_mass_sum == 0.0 {
                    continue;
                }

                let correction_mag = (contact.depth - slop).max(0.0) / inv_mass_sum * percent;
                let correction = contact.normal * correction_mag;

                if let Some(ba) = self.bodies.get_mut(&contact.body_a)
                    && ba.is_dynamic()
                {
                    ba.position -= correction * ba.inv_mass;
                }
                if let Some(bb) = self.bodies.get_mut(&contact.body_b)
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
            }
        }
    }

    fn world_anchor_3d(&self, body: BodyHandle, local: DVec3) -> DVec3 {
        let rb = match self.bodies.get(&body) {
            Some(b) => b,
            None => return local,
        };
        rb.position + rb.rotation * local
    }

    fn solve_fixed_joint_3d(&mut self, joint: &Joint3d) {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);
        let diff = anchor_b - anchor_a;

        if let Some(ba) = self.bodies.get_mut(&joint.body_a)
            && ba.is_dynamic()
        {
            ba.position += diff * 0.5;
        }
        if let Some(bb) = self.bodies.get_mut(&joint.body_b)
            && bb.is_dynamic()
        {
            bb.position -= diff * 0.5;
        }
    }

    fn solve_distance_joint_3d(&mut self, joint: &Joint3d, length: f64) {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);
        let diff = anchor_b - anchor_a;
        let dist = diff.length();

        if dist < 1e-10 {
            return;
        }

        let n = diff / dist;
        let correction = (dist - length) * 0.5;

        if let Some(ba) = self.bodies.get_mut(&joint.body_a)
            && ba.is_dynamic()
        {
            ba.position += n * correction;
        }
        if let Some(bb) = self.bodies.get_mut(&joint.body_b)
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
        let dist = diff.length();

        if dist < 1e-10 {
            return;
        }

        let n = diff / dist;
        let spring_force = stiffness * (dist - rest_length);

        let vel_a = self
            .bodies
            .get(&joint.body_a)
            .map(|b| b.linear_velocity)
            .unwrap_or(DVec3::ZERO);
        let vel_b = self
            .bodies
            .get(&joint.body_b)
            .map(|b| b.linear_velocity)
            .unwrap_or(DVec3::ZERO);
        let rel_vel = vel_b - vel_a;
        let damping_force = damping * rel_vel.dot(n);

        let total_force = spring_force + damping_force;
        let force = n * (total_force * dt);

        if let Some(ba) = self.bodies.get_mut(&joint.body_a)
            && ba.is_dynamic()
        {
            ba.linear_velocity += force * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(&joint.body_b)
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
        let current_pairs: HashSet<(ColliderHandle, ColliderHandle)> = contacts
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
            let rb = match self.bodies.get(&collider.body) {
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

fn generate_contact_3d(
    shape_a: &ColliderShape,
    pos_a: DVec3,
    shape_b: &ColliderShape,
    pos_b: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    match (shape_a, shape_b) {
        (ColliderShape::Ball { radius: ra }, ColliderShape::Ball { radius: rb }) => {
            sphere_sphere(pos_a, *ra, pos_b, *rb)
        }
        (ColliderShape::Ball { radius }, ColliderShape::Box { half_extents }) => {
            sphere_aabb(pos_a, *radius, pos_b, DVec3::from_array(*half_extents))
        }
        (ColliderShape::Box { half_extents }, ColliderShape::Ball { radius }) => {
            sphere_aabb(pos_b, *radius, pos_a, DVec3::from_array(*half_extents))
                .map(|(n, d, p)| (-n, d, p))
        }
        (
            ColliderShape::Box { half_extents: he_a },
            ColliderShape::Box { half_extents: he_b },
        ) => aabb_aabb_3d(pos_a, DVec3::from_array(*he_a), pos_b, DVec3::from_array(*he_b)),
        // Capsule vs Sphere
        (
            ColliderShape::Capsule {
                half_height,
                radius: cr,
            },
            ColliderShape::Ball { radius: br },
        ) => capsule_sphere_3d(pos_a, *half_height, *cr, pos_b, *br),
        (
            ColliderShape::Ball { radius: br },
            ColliderShape::Capsule {
                half_height,
                radius: cr,
            },
        ) => {
            capsule_sphere_3d(pos_b, *half_height, *cr, pos_a, *br)
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
    let (normal, depth) = if dist < 1e-10 {
        (DVec3::Y, sum_r)
    } else {
        (d / dist, sum_r - dist)
    };

    let point = pos_a + normal * ra;
    Some((normal, depth, point))
}

fn sphere_aabb(
    sphere_pos: DVec3,
    radius: f64,
    box_pos: DVec3,
    half_extents: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    let d = sphere_pos - box_pos;
    let closest = d.clamp(-half_extents, half_extents);
    let diff = d - closest;
    let dist_sq = diff.dot(diff);

    if dist_sq >= radius * radius {
        return None;
    }

    let dist = dist_sq.sqrt();
    let (normal, depth) = if dist < 1e-10 {
        let face_dists = DVec3::new(
            half_extents.x - d.x.abs(),
            half_extents.y - d.y.abs(),
            half_extents.z - d.z.abs(),
        );
        let min_axis = if face_dists.x <= face_dists.y && face_dists.x <= face_dists.z {
            0
        } else if face_dists.y <= face_dists.z {
            1
        } else {
            2
        };
        let mut n = DVec3::ZERO;
        n[min_axis] = if d[min_axis] >= 0.0 { 1.0 } else { -1.0 };
        (n, face_dists[min_axis] + radius)
    } else {
        (diff / dist, radius - dist)
    };

    let point = box_pos + closest;
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
    if len_sq < 1e-20 {
        return a;
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    a + ab * t
}

fn capsule_sphere_3d(
    cap_pos: DVec3,
    half_height: f64,
    cap_radius: f64,
    sphere_pos: DVec3,
    sphere_radius: f64,
) -> Option<(DVec3, f64, DVec3)> {
    // Capsule axis along Y in local space (no rotation transform here — pos is world center)
    let ep_a = cap_pos + DVec3::new(0.0, -half_height, 0.0);
    let ep_b = cap_pos + DVec3::new(0.0, half_height, 0.0);
    let closest = closest_point_on_segment_3d(ep_a, ep_b, sphere_pos);
    sphere_sphere(closest, cap_radius, sphere_pos, sphere_radius)
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
        if dir[i].abs() < 1e-10 {
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
    fn sphere_aabb_overlap() {
        let r = sphere_aabb(
            DVec3::new(1.8, 0.0, 0.0),
            0.5,
            DVec3::ZERO,
            DVec3::new(1.5, 1.0, 1.0),
        );
        assert!(r.is_some());
    }

    #[test]
    fn sphere_aabb_miss() {
        assert!(sphere_aabb(
            DVec3::new(5.0, 0.0, 0.0),
            0.5,
            DVec3::ZERO,
            DVec3::new(1.0, 1.0, 1.0),
        )
        .is_none());
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
        let bh = BodyHandle(0);
        state.add_body(
            bh,
            &BodyDesc {
                body_type: BodyType::Dynamic,
                position: [0.0, 10.0, 0.0],
                ..BodyDesc::default()
            },
        );
        state.add_collider(
            ColliderHandle(0),
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
            state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1);
        }

        assert!(state.bodies[&bh].position.y < 10.0, "body should fall");
    }

    #[test]
    fn sphere_collision_3d() {
        let mut state = PhysicsState3d::new();

        let floor = BodyHandle(0);
        state.add_body(
            floor,
            &BodyDesc {
                body_type: BodyType::Static,
                position: [0.0, 0.0, 0.0],
                ..BodyDesc::default()
            },
        );
        state.add_collider(
            ColliderHandle(0),
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

        let ball = BodyHandle(1);
        state.add_body(
            ball,
            &BodyDesc {
                body_type: BodyType::Dynamic,
                position: [0.0, 2.0, 0.0],
                ..BodyDesc::default()
            },
        );
        state.add_collider(
            ColliderHandle(1),
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
            let events = state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1);
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
        let bh = BodyHandle(0);
        state.add_body(
            bh,
            &BodyDesc {
                body_type: BodyType::Static,
                position: [5.0, 0.0, 0.0],
                ..BodyDesc::default()
            },
        );
        state.add_collider(
            ColliderHandle(0),
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
        state.add_body(BodyHandle(0), &BodyDesc::default());
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
        let bh = BodyHandle(0);
        state.add_body(bh, &BodyDesc::default());

        state.add_collider(ColliderHandle(0), bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        let mass_first = state.bodies[&bh].mass;

        state.add_collider(ColliderHandle(1), bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [2.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        assert!((state.bodies[&bh].mass - 2.0 * mass_first).abs() < EPS);
    }

    #[test]
    fn capsule_sphere_3d_overlap() {
        let r = capsule_sphere_3d(DVec3::ZERO, 1.0, 0.5, DVec3::new(0.8, 0.0, 0.0), 0.5);
        assert!(r.is_some());
    }

    #[test]
    fn capsule_sphere_3d_miss() {
        assert!(capsule_sphere_3d(DVec3::ZERO, 1.0, 0.5, DVec3::new(5.0, 0.0, 0.0), 0.5).is_none());
    }

    #[test]
    fn impulse_changes_velocity_3d() {
        let mut state = PhysicsState3d::new();
        let bh = BodyHandle(0);
        state.add_body(bh, &BodyDesc::default());
        state.add_collider(ColliderHandle(0), bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        state.apply_impulse(bh, &Impulse::new(10.0, 0.0, 0.0));
        assert!(state.bodies[&bh].linear_velocity.x > 0.0);
    }

    #[test]
    fn remove_cleans_collision_pairs_3d() {
        let mut state = PhysicsState3d::new();

        let a = BodyHandle(0);
        state.add_body(a, &BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(ColliderHandle(0), a, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        let b = BodyHandle(1);
        state.add_body(b, &BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.5, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(ColliderHandle(1), b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });

        state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1);
        assert!(!state.prev_collision_pairs.is_empty());

        state.remove_body(b);
        assert!(state.prev_collision_pairs.is_empty());
    }

    #[test]
    fn fixed_joint_3d() {
        let mut state = PhysicsState3d::new();
        let a = BodyHandle(0);
        let b = BodyHandle(1);
        state.add_body(a, &BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 5.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_body(b, &BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 3.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(ColliderHandle(0), b, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        });
        state.add_joint(JointHandle(0), &JointDesc {
            body_a: a,
            body_b: b,
            joint_type: JointType::Fixed,
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
        });

        for _ in 0..10 {
            state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1);
        }
        // Joint should prevent body from falling far
        assert!(state.bodies[&b].position.y > 2.0);
    }

    #[test]
    fn spatial_hash_3d_finds_pair() {
        let mut grid = SpatialHash3d::new(2.0);
        grid.insert(ColliderHandle(0), &Aabb3d {
            min: DVec3::ZERO,
            max: DVec3::ONE,
        });
        grid.insert(ColliderHandle(1), &Aabb3d {
            min: DVec3::splat(0.5),
            max: DVec3::splat(1.5),
        });
        let pairs = grid.query_pairs();
        assert!(pairs.contains(&(ColliderHandle(0), ColliderHandle(1))));
    }

    #[test]
    fn spatial_hash_3d_no_false_pair() {
        let mut grid = SpatialHash3d::new(1.0);
        grid.insert(ColliderHandle(0), &Aabb3d {
            min: DVec3::ZERO,
            max: DVec3::splat(0.5),
        });
        grid.insert(ColliderHandle(1), &Aabb3d {
            min: DVec3::splat(10.0),
            max: DVec3::splat(10.5),
        });
        assert!(grid.query_pairs().is_empty());
    }
}
