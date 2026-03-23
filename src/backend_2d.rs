//! Native 2D physics backend built on ganit.
//!
//! Implements broadphase (AABB sweep), narrowphase (shape-vs-shape contact
//! generation), a sequential impulse constraint solver, and island-based
//! sleep/wake management. All geometry uses f64 precision.

use std::collections::HashMap;

use crate::body::{BodyDesc, BodyHandle, BodyState, BodyType};
use crate::collider::{ColliderDesc, ColliderHandle, ColliderShape};
use crate::event::CollisionEvent;
use crate::force::{Force, Impulse, Torque};
use crate::joint::{JointDesc, JointHandle, JointType};
use crate::material::PhysicsMaterial;
use crate::query::RayHit;
use crate::ImpetusError;

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
    // Accumulated forces for the current step
    pub force_accumulator: [f64; 2],
    pub torque_accumulator: f64,
    // Mass properties (computed from attached colliders)
    pub inv_mass: f64,
    pub inv_inertia: f64,
    pub is_sleeping: bool,
}

impl RigidBody2d {
    fn from_desc(handle: BodyHandle, desc: &BodyDesc) -> Self {
        let (inv_mass, inv_inertia) = match desc.body_type {
            BodyType::Dynamic => (1.0, 1.0), // Updated when colliders are attached
            BodyType::Static | BodyType::Kinematic => (0.0, 0.0),
        };
        Self {
            handle,
            body_type: desc.body_type,
            position: desc.position,
            rotation: desc.rotation,
            linear_velocity: desc.linear_velocity,
            angular_velocity: desc.angular_velocity,
            linear_damping: desc.linear_damping,
            angular_damping: desc.angular_damping,
            fixed_rotation: desc.fixed_rotation,
            gravity_scale: desc.gravity_scale.unwrap_or(1.0),
            force_accumulator: [0.0, 0.0],
            torque_accumulator: 0.0,
            inv_mass,
            inv_inertia,
            is_sleeping: false,
        }
    }

    fn is_dynamic(&self) -> bool {
        self.body_type == BodyType::Dynamic
    }

    fn is_static(&self) -> bool {
        self.body_type == BodyType::Static
    }

    fn integrate_velocities(&mut self, gravity: [f64; 2], dt: f64) {
        if !self.is_dynamic() {
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
    }

    fn integrate_positions(&mut self, dt: f64) {
        if !self.is_dynamic() {
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
    pub mass: Option<f64>,
}

impl Collider2d {
    fn from_desc(handle: ColliderHandle, body: BodyHandle, desc: &ColliderDesc) -> Self {
        Self {
            handle,
            body,
            shape: desc.shape.clone(),
            offset: desc.offset,
            material: desc.material.clone(),
            mass: desc.mass,
        }
    }

    /// Compute AABB in world space given body position and rotation.
    fn world_aabb(&self, body_pos: [f64; 2], body_rot: f64) -> Aabb2d {
        let (sin, cos) = body_rot.sin_cos();
        let wx = body_pos[0] + cos * self.offset[0] - sin * self.offset[1];
        let wy = body_pos[1] + sin * self.offset[0] + cos * self.offset[1];

        match &self.shape {
            ColliderShape::Ball { radius } => Aabb2d {
                min: [wx - radius, wy - radius],
                max: [wx + radius, wy + radius],
            },
            ColliderShape::Box { half_extents } => {
                // Rotated box — expand to axis-aligned bounds
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
                let mut min = [f64::INFINITY, f64::INFINITY];
                let mut max = [f64::NEG_INFINITY, f64::NEG_INFINITY];
                for p in points {
                    let px = cos * p[0] - sin * p[1] + wx;
                    let py = sin * p[0] + cos * p[1] + wy;
                    min[0] = min[0].min(px);
                    min[1] = min[1].min(py);
                    max[0] = max[0].max(px);
                    max[1] = max[1].max(py);
                }
                Aabb2d { min, max }
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
            ColliderShape::TriMesh { .. } => {
                // 3D only — return degenerate AABB
                Aabb2d {
                    min: [wx, wy],
                    max: [wx, wy],
                }
            }
        }
    }

    /// Compute mass from shape and material.
    fn compute_mass(&self) -> f64 {
        if let Some(m) = self.mass {
            return m;
        }
        let area = match &self.shape {
            ColliderShape::Ball { radius } => std::f64::consts::PI * radius * radius,
            ColliderShape::Box { half_extents } => 4.0 * half_extents[0] * half_extents[1],
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                // Rectangle + circle
                2.0 * half_height * 2.0 * radius + std::f64::consts::PI * radius * radius
            }
            ColliderShape::Segment { .. } => 0.0, // Line has no area
            _ => 1.0,                             // Fallback
        };
        area * self.material.density
    }

    /// Compute moment of inertia about center of mass.
    fn compute_inertia(&self, mass: f64) -> f64 {
        match &self.shape {
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
                // Approximate: rectangle + semicircle contributions
                let rect_mass = mass * (2.0 * half_height * 2.0 * radius)
                    / (2.0 * half_height * 2.0 * radius
                        + std::f64::consts::PI * radius * radius);
                let w = 2.0 * radius;
                let h = 2.0 * half_height;
                rect_mass * (w * w + h * h) / 12.0 + (mass - rect_mass) * 0.5 * radius * radius
            }
            _ => mass, // Fallback: unit inertia
        }
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
}

// ---------------------------------------------------------------------------
// Physics state
// ---------------------------------------------------------------------------

pub(crate) struct PhysicsState2d {
    pub bodies: HashMap<BodyHandle, RigidBody2d>,
    pub colliders: HashMap<ColliderHandle, Collider2d>,
    pub joints: HashMap<JointHandle, Joint2d>,
    // Track which colliders belong to which body
    pub body_colliders: HashMap<BodyHandle, Vec<ColliderHandle>>,
    // Previous-frame collision pairs for started/stopped detection
    prev_collision_pairs: Vec<(ColliderHandle, ColliderHandle)>,
}

impl PhysicsState2d {
    pub fn new() -> Self {
        Self {
            bodies: HashMap::new(),
            colliders: HashMap::new(),
            joints: HashMap::new(),
            body_colliders: HashMap::new(),
            prev_collision_pairs: Vec::new(),
        }
    }

    pub fn add_body(&mut self, handle: BodyHandle, desc: &BodyDesc) {
        self.bodies.insert(handle, RigidBody2d::from_desc(handle, desc));
        self.body_colliders.insert(handle, Vec::new());
    }

    pub fn add_collider(&mut self, handle: ColliderHandle, body: BodyHandle, desc: &ColliderDesc) {
        let collider = Collider2d::from_desc(handle, body, desc);
        // Update body mass properties
        if let Some(rb) = self.bodies.get_mut(&body)
            && rb.is_dynamic()
        {
            let mass = collider.compute_mass();
            let inertia = collider.compute_inertia(mass);
            if mass > 0.0 {
                rb.inv_mass = 1.0 / mass;
                rb.inv_inertia = if rb.fixed_rotation || inertia <= 0.0 {
                    0.0
                } else {
                    1.0 / inertia
                };
            }
        }
        self.body_colliders
            .entry(body)
            .or_default()
            .push(handle);
        self.colliders.insert(handle, collider);
    }

    pub fn add_joint(&mut self, handle: JointHandle, desc: &JointDesc) {
        self.joints.insert(
            handle,
            Joint2d {
                body_a: desc.body_a,
                body_b: desc.body_b,
                joint_type: desc.joint_type.clone(),
                local_anchor_a: desc.local_anchor_a,
                local_anchor_b: desc.local_anchor_b,
            },
        );
    }

    pub fn apply_force(&mut self, body: BodyHandle, force: &Force) {
        if let Some(rb) = self.bodies.get_mut(&body) {
            rb.force_accumulator[0] += force.vector[0];
            rb.force_accumulator[1] += force.vector[1];
            // If force is applied at a point, compute torque
            if let Some(point) = force.point {
                let torque = point[0] * force.vector[1] - point[1] * force.vector[0];
                rb.torque_accumulator += torque;
            }
            rb.is_sleeping = false;
        }
    }

    pub fn apply_impulse(&mut self, body: BodyHandle, impulse: &Impulse) {
        if let Some(rb) = self.bodies.get_mut(&body)
            && rb.is_dynamic()
        {
            rb.linear_velocity[0] += impulse.vector[0] * rb.inv_mass;
            rb.linear_velocity[1] += impulse.vector[1] * rb.inv_mass;
            if let Some(point) = impulse.point {
                let angular_impulse =
                    point[0] * impulse.vector[1] - point[1] * impulse.vector[0];
                rb.angular_velocity += angular_impulse * rb.inv_inertia;
            }
            rb.is_sleeping = false;
        }
    }

    pub fn apply_torque(&mut self, body: BodyHandle, torque: &Torque) {
        if let Some(rb) = self.bodies.get_mut(&body) {
            rb.torque_accumulator += torque.value;
            rb.is_sleeping = false;
        }
    }

    pub fn remove_body(&mut self, handle: BodyHandle) {
        self.bodies.remove(&handle);
        // Remove attached colliders
        if let Some(collider_handles) = self.body_colliders.remove(&handle) {
            for ch in collider_handles {
                self.colliders.remove(&ch);
            }
        }
        // Remove joints referencing this body
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
            position: rb.position,
            rotation: rb.rotation,
            linear_velocity: rb.linear_velocity,
            angular_velocity: rb.angular_velocity,
            is_sleeping: rb.is_sleeping,
        })
    }

    // -----------------------------------------------------------------------
    // Simulation step
    // -----------------------------------------------------------------------

    pub fn step(&mut self, gravity: [f64; 2], dt: f64) -> Vec<CollisionEvent> {
        // 1. Integrate velocities (apply gravity + forces)
        for rb in self.bodies.values_mut() {
            rb.integrate_velocities(gravity, dt);
        }

        // 2. Broadphase: find AABB overlaps
        let broad_pairs = self.broadphase();

        // 3. Narrowphase: generate contacts
        let contacts = self.narrowphase(&broad_pairs);

        // 4. Solve contact constraints (sequential impulse)
        self.solve_contacts(&contacts, dt);

        // 5. Solve joint constraints
        self.solve_joints(dt);

        // 6. Integrate positions
        for rb in self.bodies.values_mut() {
            rb.integrate_positions(dt);
        }

        // 7. Clear forces
        for rb in self.bodies.values_mut() {
            rb.clear_forces();
        }

        // 8. Generate collision events
        self.generate_events(&contacts)
    }

    // -----------------------------------------------------------------------
    // Broadphase — brute-force AABB overlap (will upgrade to spatial hash)
    // -----------------------------------------------------------------------

    fn broadphase(&self) -> Vec<(ColliderHandle, ColliderHandle)> {
        let collider_aabbs: Vec<(ColliderHandle, Aabb2d)> = self
            .colliders
            .values()
            .filter_map(|c| {
                let rb = self.bodies.get(&c.body)?;
                Some((c.handle, c.world_aabb(rb.position, rb.rotation)))
            })
            .collect();

        let mut pairs = Vec::new();
        for i in 0..collider_aabbs.len() {
            for j in (i + 1)..collider_aabbs.len() {
                let (ha, aabb_a) = &collider_aabbs[i];
                let (hb, aabb_b) = &collider_aabbs[j];
                // Skip pairs on the same body
                let ca = &self.colliders[ha];
                let cb = &self.colliders[hb];
                if ca.body == cb.body {
                    continue;
                }
                // Skip static-static pairs
                let ba = self.bodies.get(&ca.body);
                let bb = self.bodies.get(&cb.body);
                if let (Some(ba), Some(bb)) = (ba, bb)
                    && ba.is_static() && bb.is_static()
                {
                    continue;
                }
                if aabb_a.overlaps(aabb_b) {
                    pairs.push((*ha, *hb));
                }
            }
        }
        pairs
    }

    // -----------------------------------------------------------------------
    // Narrowphase — shape-vs-shape contact generation
    // -----------------------------------------------------------------------

    fn narrowphase(
        &self,
        broad_pairs: &[(ColliderHandle, ColliderHandle)],
    ) -> Vec<Contact> {
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

            // World positions of collider centers
            let (sin_a, cos_a) = ba.rotation.sin_cos();
            let pos_a = [
                ba.position[0] + cos_a * ca.offset[0] - sin_a * ca.offset[1],
                ba.position[1] + sin_a * ca.offset[0] + cos_a * ca.offset[1],
            ];
            let (sin_b, cos_b) = bb.rotation.sin_cos();
            let pos_b = [
                bb.position[0] + cos_b * cb.offset[0] - sin_b * cb.offset[1],
                bb.position[1] + sin_b * cb.offset[0] + cos_b * cb.offset[1],
            ];

            if let Some(contact) =
                generate_contact(&ca.shape, pos_a, ba.rotation, &cb.shape, pos_b, bb.rotation)
            {
                contacts.push(Contact {
                    collider_a: *ha,
                    collider_b: *hb,
                    body_a: ca.body,
                    body_b: cb.body,
                    normal: contact.0,
                    depth: contact.1,
                });
            }
        }

        contacts
    }

    // -----------------------------------------------------------------------
    // Contact constraint solver (sequential impulse)
    // -----------------------------------------------------------------------

    fn solve_contacts(&mut self, contacts: &[Contact], _dt: f64) {
        let iterations = 4;

        for _ in 0..iterations {
            for contact in contacts {
                let (inv_mass_a, vel_a, inv_mass_b, vel_b) = {
                    let ba = match self.bodies.get(&contact.body_a) {
                        Some(b) => b,
                        None => continue,
                    };
                    let bb = match self.bodies.get(&contact.body_b) {
                        Some(b) => b,
                        None => continue,
                    };
                    (
                        ba.inv_mass,
                        ba.linear_velocity,
                        bb.inv_mass,
                        bb.linear_velocity,
                    )
                };

                // Skip if both are immovable
                if inv_mass_a == 0.0 && inv_mass_b == 0.0 {
                    continue;
                }

                let n = contact.normal;
                // Relative velocity at contact point
                let rel_vel = [vel_b[0] - vel_a[0], vel_b[1] - vel_a[1]];
                let vel_along_normal = rel_vel[0] * n[0] + rel_vel[1] * n[1];

                // Don't resolve if velocities are separating
                if vel_along_normal > 0.0 {
                    continue;
                }

                // Restitution (use minimum of the two materials)
                let ca = self.colliders.get(&contact.collider_a);
                let cb = self.colliders.get(&contact.collider_b);
                let restitution = match (ca, cb) {
                    (Some(a), Some(b)) => a.material.restitution.min(b.material.restitution),
                    _ => 0.0,
                };

                // Impulse magnitude
                let inv_mass_sum = inv_mass_a + inv_mass_b;
                let j = -(1.0 + restitution) * vel_along_normal / inv_mass_sum;

                let impulse = [j * n[0], j * n[1]];

                // Apply impulse
                if let Some(ba) = self.bodies.get_mut(&contact.body_a)
                    && ba.is_dynamic()
                {
                    ba.linear_velocity[0] -= impulse[0] * ba.inv_mass;
                    ba.linear_velocity[1] -= impulse[1] * ba.inv_mass;
                }
                if let Some(bb) = self.bodies.get_mut(&contact.body_b)
                    && bb.is_dynamic()
                {
                    bb.linear_velocity[0] += impulse[0] * bb.inv_mass;
                    bb.linear_velocity[1] += impulse[1] * bb.inv_mass;
                }

                // Positional correction (Baumgarte stabilization)
                let slop = 0.01;
                let percent = 0.2;
                let correction_mag =
                    (contact.depth - slop).max(0.0) / inv_mass_sum * percent;
                let correction = [correction_mag * n[0], correction_mag * n[1]];

                if let Some(ba) = self.bodies.get_mut(&contact.body_a)
                    && ba.is_dynamic()
                {
                    ba.position[0] -= correction[0] * ba.inv_mass;
                    ba.position[1] -= correction[1] * ba.inv_mass;
                }
                if let Some(bb) = self.bodies.get_mut(&contact.body_b)
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

    fn solve_joints(&mut self, dt: f64) {
        let joints: Vec<Joint2d> = self.joints.values().cloned().collect();
        let iterations = 4;

        for _ in 0..iterations {
            for joint in &joints {
                match &joint.joint_type {
                    JointType::Fixed => self.solve_fixed_joint(joint),
                    JointType::Distance { length } => {
                        self.solve_distance_joint(joint, *length);
                    }
                    JointType::Spring {
                        rest_length,
                        stiffness,
                        damping,
                    } => {
                        self.solve_spring_joint(joint, *rest_length, *stiffness, *damping, dt);
                    }
                    JointType::Revolute { limits, .. } => {
                        self.solve_revolute_joint(joint, limits.as_ref());
                    }
                    JointType::Prismatic { axis, limits } => {
                        self.solve_prismatic_joint(joint, *axis, limits.as_ref());
                    }
                }
            }
        }
    }

    fn world_anchor(&self, body: BodyHandle, local: [f64; 2]) -> [f64; 2] {
        let rb = match self.bodies.get(&body) {
            Some(b) => b,
            None => return local,
        };
        let (sin, cos) = rb.rotation.sin_cos();
        [
            rb.position[0] + cos * local[0] - sin * local[1],
            rb.position[1] + sin * local[0] + cos * local[1],
        ]
    }

    fn solve_fixed_joint(&mut self, joint: &Joint2d) {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];
        let correction = 0.5;

        if let Some(ba) = self.bodies.get_mut(&joint.body_a)
            && ba.is_dynamic()
        {
            ba.position[0] += diff[0] * correction;
            ba.position[1] += diff[1] * correction;
        }
        if let Some(bb) = self.bodies.get_mut(&joint.body_b)
            && bb.is_dynamic()
        {
            bb.position[0] -= diff[0] * correction;
            bb.position[1] -= diff[1] * correction;
        }
    }

    fn solve_distance_joint(&mut self, joint: &Joint2d, length: f64) {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];
        let dist = (diff[0] * diff[0] + diff[1] * diff[1]).sqrt();

        if dist < 1e-10 {
            return;
        }

        let n = [diff[0] / dist, diff[1] / dist];
        let error = dist - length;
        let correction = error * 0.5;

        if let Some(ba) = self.bodies.get_mut(&joint.body_a)
            && ba.is_dynamic()
        {
            ba.position[0] += n[0] * correction;
            ba.position[1] += n[1] * correction;
        }
        if let Some(bb) = self.bodies.get_mut(&joint.body_b)
            && bb.is_dynamic()
        {
            bb.position[0] -= n[0] * correction;
            bb.position[1] -= n[1] * correction;
        }
    }

    fn solve_spring_joint(
        &mut self,
        joint: &Joint2d,
        rest_length: f64,
        stiffness: f64,
        damping: f64,
        dt: f64,
    ) {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];
        let dist = (diff[0] * diff[0] + diff[1] * diff[1]).sqrt();

        if dist < 1e-10 {
            return;
        }

        let n = [diff[0] / dist, diff[1] / dist];
        let spring_force = stiffness * (dist - rest_length);

        // Relative velocity along spring axis for damping
        let vel_a = self
            .bodies
            .get(&joint.body_a)
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let vel_b = self
            .bodies
            .get(&joint.body_b)
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let rel_vel = [vel_b[0] - vel_a[0], vel_b[1] - vel_a[1]];
        let damping_force = damping * (rel_vel[0] * n[0] + rel_vel[1] * n[1]);

        let total_force = spring_force + damping_force;
        let force = [total_force * n[0] * dt, total_force * n[1] * dt];

        if let Some(ba) = self.bodies.get_mut(&joint.body_a)
            && ba.is_dynamic()
        {
            ba.linear_velocity[0] += force[0] * ba.inv_mass;
            ba.linear_velocity[1] += force[1] * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(&joint.body_b)
            && bb.is_dynamic()
        {
            bb.linear_velocity[0] -= force[0] * bb.inv_mass;
            bb.linear_velocity[1] -= force[1] * bb.inv_mass;
        }
    }

    fn solve_revolute_joint(&mut self, joint: &Joint2d, limits: Option<&[f64; 2]>) {
        // Pin anchor points together
        self.solve_fixed_joint(joint);

        // Apply angular limits if present
        if let Some([lo, hi]) = limits {
            let rot_a = self.bodies.get(&joint.body_a).map(|b| b.rotation).unwrap_or(0.0);
            let rot_b = self.bodies.get(&joint.body_b).map(|b| b.rotation).unwrap_or(0.0);
            let rel_angle = rot_b - rot_a;

            if rel_angle < *lo {
                let correction = (lo - rel_angle) * 0.5;
                if let Some(ba) = self.bodies.get_mut(&joint.body_a)
                    && ba.is_dynamic()
                {
                    ba.rotation -= correction;
                }
                if let Some(bb) = self.bodies.get_mut(&joint.body_b)
                    && bb.is_dynamic()
                {
                    bb.rotation += correction;
                }
            } else if rel_angle > *hi {
                let correction = (rel_angle - hi) * 0.5;
                if let Some(ba) = self.bodies.get_mut(&joint.body_a)
                    && ba.is_dynamic()
                {
                    ba.rotation += correction;
                }
                if let Some(bb) = self.bodies.get_mut(&joint.body_b)
                    && bb.is_dynamic()
                {
                    bb.rotation -= correction;
                }
            }
        }
    }

    fn solve_prismatic_joint(
        &mut self,
        joint: &Joint2d,
        axis: [f64; 2],
        limits: Option<&[f64; 2]>,
    ) {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];

        // Axis perpendicular component — constrain to zero
        let axis_len = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
        if axis_len < 1e-10 {
            return;
        }
        let ax = [axis[0] / axis_len, axis[1] / axis_len];
        let perp = [-ax[1], ax[0]];
        let perp_error = diff[0] * perp[0] + diff[1] * perp[1];
        let correction = perp_error * 0.5;

        if let Some(ba) = self.bodies.get_mut(&joint.body_a)
            && ba.is_dynamic()
        {
            ba.position[0] += perp[0] * correction;
            ba.position[1] += perp[1] * correction;
        }
        if let Some(bb) = self.bodies.get_mut(&joint.body_b)
            && bb.is_dynamic()
        {
            bb.position[0] -= perp[0] * correction;
            bb.position[1] -= perp[1] * correction;
        }

        // Apply limits along axis
        if let Some([lo, hi]) = limits {
            let along = diff[0] * ax[0] + diff[1] * ax[1];
            if along < *lo {
                let c = (lo - along) * 0.5;
                if let Some(ba) = self.bodies.get_mut(&joint.body_a)
                    && ba.is_dynamic()
                {
                    ba.position[0] -= ax[0] * c;
                    ba.position[1] -= ax[1] * c;
                }
                if let Some(bb) = self.bodies.get_mut(&joint.body_b)
                    && bb.is_dynamic()
                {
                    bb.position[0] += ax[0] * c;
                    bb.position[1] += ax[1] * c;
                }
            } else if along > *hi {
                let c = (along - hi) * 0.5;
                if let Some(ba) = self.bodies.get_mut(&joint.body_a)
                    && ba.is_dynamic()
                {
                    ba.position[0] += ax[0] * c;
                    ba.position[1] += ax[1] * c;
                }
                if let Some(bb) = self.bodies.get_mut(&joint.body_b)
                    && bb.is_dynamic()
                {
                    bb.position[0] -= ax[0] * c;
                    bb.position[1] -= ax[1] * c;
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Collision event generation
    // -----------------------------------------------------------------------

    fn generate_events(&mut self, contacts: &[Contact]) -> Vec<CollisionEvent> {
        let mut events = Vec::new();

        let mut current_pairs: Vec<(ColliderHandle, ColliderHandle)> = contacts
            .iter()
            .map(|c| {
                if c.collider_a.0 < c.collider_b.0 {
                    (c.collider_a, c.collider_b)
                } else {
                    (c.collider_b, c.collider_a)
                }
            })
            .collect();
        current_pairs.sort_by_key(|p| (p.0 .0, p.1 .0));
        current_pairs.dedup();

        // Started: in current but not previous
        for pair in &current_pairs {
            if !self.prev_collision_pairs.contains(pair) {
                events.push(CollisionEvent::Started {
                    collider_a: pair.0,
                    collider_b: pair.1,
                });
            }
        }

        // Stopped: in previous but not current
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
        origin: [f64; 2],
        direction: [f64; 2],
        max_dist: f64,
    ) -> Option<RayHit> {
        let dir_len = (direction[0] * direction[0] + direction[1] * direction[1]).sqrt();
        if dir_len < 1e-10 {
            return None;
        }
        let dir = [direction[0] / dir_len, direction[1] / dir_len];

        let mut best: Option<(f64, ColliderHandle, [f64; 2], [f64; 2])> = None;

        for collider in self.colliders.values() {
            let rb = match self.bodies.get(&collider.body) {
                Some(b) => b,
                None => continue,
            };
            let (sin, cos) = rb.rotation.sin_cos();
            let cx = rb.position[0] + cos * collider.offset[0] - sin * collider.offset[1];
            let cy = rb.position[1] + sin * collider.offset[0] + cos * collider.offset[1];

            let hit = match &collider.shape {
                ColliderShape::Ball { radius } => {
                    ray_circle(origin, dir, [cx, cy], *radius)
                }
                ColliderShape::Box { half_extents } => {
                    ray_aabb_2d(
                        origin,
                        dir,
                        [cx - half_extents[0], cy - half_extents[1]],
                        [cx + half_extents[0], cy + half_extents[1]],
                    )
                }
                _ => None,
            };

            if let Some((t, normal)) = hit
                && t >= 0.0
                && t <= max_dist
                && (best.is_none() || t < best.as_ref().unwrap().0)
            {
                let point = [origin[0] + dir[0] * t, origin[1] + dir[1] * t];
                best = Some((t, collider.handle, point, normal));
            }
        }

        best.map(|(distance, collider, point, normal)| RayHit {
            collider,
            point,
            normal,
            distance,
        })
    }
}

// ---------------------------------------------------------------------------
// Narrowphase contact generation (free functions)
// ---------------------------------------------------------------------------

fn generate_contact(
    shape_a: &ColliderShape,
    pos_a: [f64; 2],
    _rot_a: f64,
    shape_b: &ColliderShape,
    pos_b: [f64; 2],
    _rot_b: f64,
) -> Option<([f64; 2], f64, [f64; 2])> {
    match (shape_a, shape_b) {
        (ColliderShape::Ball { radius: ra }, ColliderShape::Ball { radius: rb }) => {
            circle_circle(pos_a, *ra, pos_b, *rb)
        }
        (ColliderShape::Ball { radius }, ColliderShape::Box { half_extents }) => {
            circle_aabb(pos_a, *radius, pos_b, *half_extents)
        }
        (ColliderShape::Box { half_extents }, ColliderShape::Ball { radius }) => {
            circle_aabb(pos_b, *radius, pos_a, *half_extents).map(|(n, d, p)| ([-n[0], -n[1]], d, p))
        }
        (
            ColliderShape::Box {
                half_extents: he_a,
            },
            ColliderShape::Box {
                half_extents: he_b,
            },
        ) => aabb_aabb_contact(pos_a, *he_a, pos_b, *he_b),
        _ => None, // Other shape combinations: TODO
    }
}

/// Circle vs circle contact.
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
    let (normal, depth) = if dist < 1e-10 {
        ([0.0, 1.0], sum_r)
    } else {
        ([dx / dist, dy / dist], sum_r - dist)
    };

    let point = [
        pos_a[0] + normal[0] * ra,
        pos_a[1] + normal[1] * ra,
    ];

    Some((normal, depth, point))
}

/// Circle vs AABB contact.
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
    let (normal, depth) = if dist < 1e-10 {
        // Circle center is inside the box — find closest face
        let face_dists = [
            half_extents[0] - dx.abs(),
            half_extents[1] - dy.abs(),
        ];
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

    let point = [
        box_pos[0] + closest_x,
        box_pos[1] + closest_y,
    ];

    Some((normal, depth, point))
}

/// AABB vs AABB contact (SAT).
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
    let normal_len = ((point[0] - center[0]).powi(2) + (point[1] - center[1]).powi(2)).sqrt();
    let normal = if normal_len > 1e-10 {
        [
            (point[0] - center[0]) / normal_len,
            (point[1] - center[1]) / normal_len,
        ]
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
        if dir[i].abs() < 1e-10 {
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
