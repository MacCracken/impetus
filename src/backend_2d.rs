//! Native 2D physics backend.
//!
//! Implements broadphase (AABB overlap), narrowphase (shape-vs-shape contact
//! generation), and a sequential impulse constraint solver with friction and
//! angular response. All geometry uses f64 precision.

use std::collections::{HashMap, HashSet};

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
    pub force_accumulator: [f64; 2],
    pub torque_accumulator: f64,
    // Mass properties (accumulated from attached colliders)
    pub mass: f64,
    pub inv_mass: f64,
    pub inertia: f64,
    pub inv_inertia: f64,
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
        }
    }

    fn is_dynamic(&self) -> bool {
        self.body_type == BodyType::Dynamic
    }

    fn is_static(&self) -> bool {
        self.body_type == BodyType::Static
    }

    fn integrate_velocities(&mut self, gravity: [f64; 2], dt: f64) {
        if !self.is_dynamic() || self.inv_mass == 0.0 {
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
            return m.max(1e-6);
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
        (area * self.material.density).max(1e-6)
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
        i.max(1e-10)
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
// Spatial hash broadphase
// ---------------------------------------------------------------------------

struct SpatialHash {
    inv_cell_size: f64,
    cells: HashMap<(i32, i32), Vec<ColliderHandle>>,
}

impl SpatialHash {
    fn new(cell_size: f64) -> Self {
        Self {
            inv_cell_size: 1.0 / cell_size,
            cells: HashMap::new(),
        }
    }

    /// Auto-compute cell size from collider AABBs. Uses 2x the average AABB max dimension.
    fn auto_cell_size(aabbs: &[(ColliderHandle, Aabb2d)]) -> f64 {
        if aabbs.is_empty() {
            return 1.0;
        }
        let total: f64 = aabbs
            .iter()
            .map(|(_, aabb)| {
                let w = aabb.max[0] - aabb.min[0];
                let h = aabb.max[1] - aabb.min[1];
                w.max(h)
            })
            .sum();
        let avg = total / aabbs.len() as f64;
        (avg * 2.0).max(0.1) // at least 0.1 to avoid degenerate cells
    }

    fn cell(&self, x: f64, y: f64) -> (i32, i32) {
        (
            (x * self.inv_cell_size).floor() as i32,
            (y * self.inv_cell_size).floor() as i32,
        )
    }

    fn insert(&mut self, handle: ColliderHandle, aabb: &Aabb2d) {
        let (min_cx, min_cy) = self.cell(aabb.min[0], aabb.min[1]);
        let (max_cx, max_cy) = self.cell(aabb.max[0], aabb.max[1]);

        for cx in min_cx..=max_cx {
            for cy in min_cy..=max_cy {
                self.cells.entry((cx, cy)).or_default().push(handle);
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
                    // Canonical ordering for dedup
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
// Physics state
// ---------------------------------------------------------------------------

pub(crate) struct PhysicsState2d {
    pub bodies: HashMap<BodyHandle, RigidBody2d>,
    pub colliders: HashMap<ColliderHandle, Collider2d>,
    pub joints: HashMap<JointHandle, Joint2d>,
    pub body_colliders: HashMap<BodyHandle, Vec<ColliderHandle>>,
    prev_collision_pairs: HashSet<(ColliderHandle, ColliderHandle)>,
}

impl PhysicsState2d {
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
            .insert(handle, RigidBody2d::from_desc(handle, desc));
        self.body_colliders.insert(handle, Vec::new());
    }

    pub fn add_collider(
        &mut self,
        handle: ColliderHandle,
        body: BodyHandle,
        desc: &ColliderDesc,
    ) {
        let collider = Collider2d::from_desc(handle, body, desc);

        // Accumulate mass properties onto the body
        if let Some(rb) = self.bodies.get_mut(&body)
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

        self.body_colliders.entry(body).or_default().push(handle);
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
            if let Some(point) = force.point {
                rb.torque_accumulator += point[0] * force.vector[1] - point[1] * force.vector[0];
            }
        }
    }

    pub fn apply_impulse(&mut self, body: BodyHandle, impulse: &Impulse) {
        if let Some(rb) = self.bodies.get_mut(&body)
            && rb.is_dynamic()
            && rb.inv_mass > 0.0
        {
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
        if let Some(rb) = self.bodies.get_mut(&body) {
            rb.torque_accumulator += torque.value;
        }
    }

    pub fn remove_body(&mut self, handle: BodyHandle) {
        self.bodies.remove(&handle);
        if let Some(collider_handles) = self.body_colliders.remove(&handle) {
            for ch in &collider_handles {
                self.colliders.remove(ch);
            }
            // Clean stale collision pairs referencing removed colliders
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
            position: [rb.position[0], rb.position[1], 0.0],
            rotation: rb.rotation,
            linear_velocity: [rb.linear_velocity[0], rb.linear_velocity[1], 0.0],
            angular_velocity: rb.angular_velocity,
            is_sleeping: false,
        })
    }

    // -----------------------------------------------------------------------
    // Simulation step
    // -----------------------------------------------------------------------

    pub fn step(
        &mut self,
        gravity: [f64; 3],
        dt: f64,
        velocity_iterations: u32,
        position_iterations: u32,
    ) -> Vec<CollisionEvent> {
        let gravity_2d = [gravity[0], gravity[1]];
        // 1. Integrate velocities
        for rb in self.bodies.values_mut() {
            rb.integrate_velocities(gravity_2d, dt);
        }

        // 2. Broadphase
        let broad_pairs = self.broadphase();

        // 3. Narrowphase
        let contacts = self.narrowphase(&broad_pairs);

        // 4. Solve velocity constraints
        self.solve_contacts(&contacts, velocity_iterations);

        // 5. Solve joint constraints
        self.solve_joints(dt, velocity_iterations);

        // 6. Positional correction
        self.solve_positions(&contacts, position_iterations);

        // 7. Integrate positions
        for rb in self.bodies.values_mut() {
            rb.integrate_positions(dt);
        }

        // 8. Clear forces
        for rb in self.bodies.values_mut() {
            rb.clear_forces();
        }

        // 9. Generate collision events
        self.generate_events(&contacts)
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
                let rb = self.bodies.get(&c.body)?;
                Some((c.handle, c.world_aabb(rb.position, rb.rotation)))
            })
            .collect();

        // Build spatial hash
        let cell_size = SpatialHash::auto_cell_size(&collider_aabbs);
        let mut grid = SpatialHash::new(cell_size);
        for (handle, aabb) in &collider_aabbs {
            grid.insert(*handle, aabb);
        }

        // Collect candidate pairs from shared cells
        let candidates = grid.query_pairs();

        // Build AABB lookup for overlap verification
        let aabb_map: HashMap<ColliderHandle, Aabb2d> =
            collider_aabbs.into_iter().collect();

        // Filter candidates
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
            // Skip same-body
            if ca.body == cb.body {
                continue;
            }
            // Skip static-static
            if let (Some(ba), Some(bb)) = (self.bodies.get(&ca.body), self.bodies.get(&cb.body))
                && ba.is_static() && bb.is_static()
            {
                continue;
            }
            // Skip sensor-sensor
            if ca.is_sensor && cb.is_sensor {
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
    // Contact constraint solver with friction and angular response
    // -----------------------------------------------------------------------

    fn solve_contacts(&mut self, contacts: &[Contact], iterations: u32) {
        // Pre-extract material properties to avoid repeated HashMap lookups
        struct ContactMaterial {
            restitution: f64,
            friction: f64,
            is_sensor: bool,
        }
        let materials: Vec<ContactMaterial> = contacts
            .iter()
            .map(|c| {
                let (rest, fric, sensor) = match (
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
                ContactMaterial {
                    restitution: rest,
                    friction: fric,
                    is_sensor: sensor,
                }
            })
            .collect();

        for _ in 0..iterations {
            for (ci, contact) in contacts.iter().enumerate() {
                // Sensors generate events but no physical response
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
                let ra = [cp[0] - pos_a[0], cp[1] - pos_a[1]];
                let rb = [cp[0] - pos_b[0], cp[1] - pos_b[1]];

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

                if vel_along_normal > 0.0 {
                    continue;
                }

                // Angular effective mass
                let ra_cross_n = ra[0] * n[1] - ra[1] * n[0];
                let rb_cross_n = rb[0] * n[1] - rb[1] * n[0];
                let inv_mass_sum = inv_mass_a + inv_mass_b
                    + ra_cross_n * ra_cross_n * inv_inertia_a
                    + rb_cross_n * rb_cross_n * inv_inertia_b;

                // Normal impulse
                let j = -(1.0 + materials[ci].restitution) * vel_along_normal / inv_mass_sum;
                let impulse_n = [j * n[0], j * n[1]];

                if let Some(ba) = self.bodies.get_mut(&contact.body_a)
                    && ba.is_dynamic()
                {
                    ba.linear_velocity[0] -= impulse_n[0] * ba.inv_mass;
                    ba.linear_velocity[1] -= impulse_n[1] * ba.inv_mass;
                    ba.angular_velocity -= ra_cross_n * j * ba.inv_inertia;
                }
                if let Some(bb) = self.bodies.get_mut(&contact.body_b)
                    && bb.is_dynamic()
                {
                    bb.linear_velocity[0] += impulse_n[0] * bb.inv_mass;
                    bb.linear_velocity[1] += impulse_n[1] * bb.inv_mass;
                    bb.angular_velocity += rb_cross_n * j * bb.inv_inertia;
                }

                // Friction impulse
                let friction = materials[ci].friction;
                if friction > 0.0 {
                    let tangent = [-n[1], n[0]];
                    let vel_along_tangent = rel_vel[0] * tangent[0] + rel_vel[1] * tangent[1];

                    let ra_cross_t = ra[0] * tangent[1] - ra[1] * tangent[0];
                    let rb_cross_t = rb[0] * tangent[1] - rb[1] * tangent[0];
                    let inv_mass_sum_t = inv_mass_a + inv_mass_b
                        + ra_cross_t * ra_cross_t * inv_inertia_a
                        + rb_cross_t * rb_cross_t * inv_inertia_b;

                    let jt = (-vel_along_tangent / inv_mass_sum_t)
                        .clamp(-j.abs() * friction, j.abs() * friction);
                    let impulse_t = [jt * tangent[0], jt * tangent[1]];

                    if let Some(ba) = self.bodies.get_mut(&contact.body_a)
                        && ba.is_dynamic()
                    {
                        ba.linear_velocity[0] -= impulse_t[0] * ba.inv_mass;
                        ba.linear_velocity[1] -= impulse_t[1] * ba.inv_mass;
                        ba.angular_velocity -= ra_cross_t * jt * ba.inv_inertia;
                    }
                    if let Some(bb) = self.bodies.get_mut(&contact.body_b)
                        && bb.is_dynamic()
                    {
                        bb.linear_velocity[0] += impulse_t[0] * bb.inv_mass;
                        bb.linear_velocity[1] += impulse_t[1] * bb.inv_mass;
                        bb.angular_velocity += rb_cross_t * jt * bb.inv_inertia;
                    }
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Positional correction (Baumgarte stabilization)
    // -----------------------------------------------------------------------

    fn solve_positions(&mut self, contacts: &[Contact], iterations: u32) {
        let slop = 0.01;
        let percent = 0.2;

        for _ in 0..iterations {
            for contact in contacts {
                // Skip sensors
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

                let n = contact.normal;
                let correction_mag = (contact.depth - slop).max(0.0) / inv_mass_sum * percent;
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

    fn solve_joints(&mut self, dt: f64, iterations: u32) {
        let joints: Vec<Joint2d> = self.joints.values().cloned().collect();

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

        if let Some(ba) = self.bodies.get_mut(&joint.body_a)
            && ba.is_dynamic()
        {
            ba.position[0] += diff[0] * 0.5;
            ba.position[1] += diff[1] * 0.5;
        }
        if let Some(bb) = self.bodies.get_mut(&joint.body_b)
            && bb.is_dynamic()
        {
            bb.position[0] -= diff[0] * 0.5;
            bb.position[1] -= diff[1] * 0.5;
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
        let correction = (dist - length) * 0.5;

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
        self.solve_fixed_joint(joint);

        if let Some([lo, hi]) = limits {
            let rot_a = self
                .bodies
                .get(&joint.body_a)
                .map(|b| b.rotation)
                .unwrap_or(0.0);
            let rot_b = self
                .bodies
                .get(&joint.body_b)
                .map(|b| b.rotation)
                .unwrap_or(0.0);
            let rel_angle = rot_b - rot_a;

            if rel_angle < *lo {
                let c = (lo - rel_angle) * 0.5;
                if let Some(ba) = self.bodies.get_mut(&joint.body_a)
                    && ba.is_dynamic()
                {
                    ba.rotation -= c;
                }
                if let Some(bb) = self.bodies.get_mut(&joint.body_b)
                    && bb.is_dynamic()
                {
                    bb.rotation += c;
                }
            } else if rel_angle > *hi {
                let c = (rel_angle - hi) * 0.5;
                if let Some(ba) = self.bodies.get_mut(&joint.body_a)
                    && ba.is_dynamic()
                {
                    ba.rotation += c;
                }
                if let Some(bb) = self.bodies.get_mut(&joint.body_b)
                    && bb.is_dynamic()
                {
                    bb.rotation -= c;
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
        let origin_2d = [origin[0], origin[1]];
        let direction_2d = [direction[0], direction[1]];
        let dir_len = (direction_2d[0] * direction_2d[0] + direction_2d[1] * direction_2d[1]).sqrt();
        if dir_len < 1e-10 {
            return None;
        }
        let dir = [direction_2d[0] / dir_len, direction_2d[1] / dir_len];

        let mut best: Option<(f64, ColliderHandle, [f64; 2], [f64; 2])> = None;

        for collider in self.colliders.values() {
            let rb = match self.bodies.get(&collider.body) {
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
        // Box vs Box
        (
            ColliderShape::Box { half_extents: he_a },
            ColliderShape::Box { half_extents: he_b },
        ) => aabb_aabb_contact(pos_a, [he_a[0], he_a[1]], pos_b, [he_b[0], he_b[1]]),
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
        _ => None,
    }
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
    let (normal, depth) = if dist < 1e-10 {
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
    let (normal, depth) = if dist < 1e-10 {
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
    if len_sq < 1e-20 {
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
    if d1 > 1e-20 && d2 > 1e-20 {
        let r = [a[0] - c[0], a[1] - c[1]];
        let d3 = ab[0] * cd[0] + ab[1] * cd[1]; // AB·CD
        let d4 = ab[0] * r[0] + ab[1] * r[1]; // AB·r
        let d5 = cd[0] * r[0] + cd[1] * r[1]; // CD·r
        let denom = d1 * d2 - d3 * d3;

        if denom.abs() > 1e-20 {
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
    let normal = if nl > 1e-10 {
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
    if axis_len > 1e-10 {
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
            },
        );
        assert!((c.compute_mass() - 42.0).abs() < EPS);
    }

    #[test]
    fn multiple_colliders_accumulate_mass() {
        let mut state = PhysicsState2d::new();
        let bh = BodyHandle(0);
        state.add_body(bh, &BodyDesc::default());

        state.add_collider(ColliderHandle(0), bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
        });
        let mass_after_first = state.bodies[&bh].mass;

        state.add_collider(ColliderHandle(1), bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [1.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
        });
        let mass_after_second = state.bodies[&bh].mass;

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
        let floor = BodyHandle(0);
        state.add_body(floor, &BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(ColliderHandle(0), floor, &ColliderDesc {
            shape: ColliderShape::Box { half_extents: [10.0, 0.5, 0.0] },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: true, // Sensor!
            mass: None,
        });

        // Dynamic ball overlapping the sensor
        let ball = BodyHandle(1);
        state.add_body(ball, &BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        state.add_collider(ColliderHandle(1), ball, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
        });

        let vel_before = state.bodies[&ball].linear_velocity;
        let events = state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1);

        // Should generate events
        assert!(!events.is_empty());
        // But sensor should not affect velocity (no physical response)
        let vel_after = state.bodies[&ball].linear_velocity;
        assert!((vel_after[0] - vel_before[0]).abs() < EPS);
        assert!((vel_after[1] - vel_before[1]).abs() < EPS);
    }

    // -- Kinematic body tests --

    #[test]
    fn kinematic_body_moves_from_velocity() {
        let mut state = PhysicsState2d::new();
        let bh = BodyHandle(0);
        state.add_body(bh, &BodyDesc {
            body_type: BodyType::Kinematic,
            position: [0.0, 0.0, 0.0],
            linear_velocity: [10.0, 0.0, 0.0],
            ..BodyDesc::default()
        });

        let dt = 1.0 / 60.0;
        state.step([0.0, -9.81, 0.0], dt, 4, 1);

        let rb = &state.bodies[&bh];
        // Should have moved from velocity
        assert!(rb.position[0] > 0.0);
        // Should NOT have been affected by gravity
        assert!((rb.linear_velocity[1]).abs() < EPS);
    }

    // -- Stale collision pair cleanup --

    #[test]
    fn remove_body_cleans_collision_pairs() {
        let mut state = PhysicsState2d::new();

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
        });

        // Step to generate collision pairs
        state.step([0.0, 0.0, 0.0], 1.0 / 60.0, 4, 1);
        assert!(!state.prev_collision_pairs.is_empty());

        // Remove body b — should clean pairs
        state.remove_body(b);
        assert!(state.prev_collision_pairs.is_empty());
    }

    // -- Spatial hash tests --

    #[test]
    fn spatial_hash_single_insert() {
        let mut grid = SpatialHash::new(1.0);
        grid.insert(ColliderHandle(0), &Aabb2d { min: [0.0, 0.0], max: [0.5, 0.5] });
        assert!(grid.cells.contains_key(&(0, 0)));
        assert_eq!(grid.cells[&(0, 0)].len(), 1);
    }

    #[test]
    fn spatial_hash_spanning_cells() {
        let mut grid = SpatialHash::new(1.0);
        // AABB spans 4 cells: (0,0), (0,1), (1,0), (1,1)
        grid.insert(ColliderHandle(0), &Aabb2d { min: [0.5, 0.5], max: [1.5, 1.5] });
        assert!(grid.cells.len() >= 4);
    }

    #[test]
    fn spatial_hash_finds_nearby_pair() {
        let mut grid = SpatialHash::new(2.0);
        grid.insert(ColliderHandle(0), &Aabb2d { min: [0.0, 0.0], max: [1.0, 1.0] });
        grid.insert(ColliderHandle(1), &Aabb2d { min: [0.5, 0.5], max: [1.5, 1.5] });
        let pairs = grid.query_pairs();
        assert!(pairs.contains(&(ColliderHandle(0), ColliderHandle(1))));
    }

    #[test]
    fn spatial_hash_no_false_pair() {
        let mut grid = SpatialHash::new(1.0);
        grid.insert(ColliderHandle(0), &Aabb2d { min: [0.0, 0.0], max: [0.5, 0.5] });
        grid.insert(ColliderHandle(1), &Aabb2d { min: [10.0, 10.0], max: [10.5, 10.5] });
        let pairs = grid.query_pairs();
        assert!(pairs.is_empty());
    }

    #[test]
    fn spatial_hash_auto_cell_size() {
        let aabbs = vec![
            (ColliderHandle(0), Aabb2d { min: [0.0, 0.0], max: [1.0, 1.0] }),
            (ColliderHandle(1), Aabb2d { min: [0.0, 0.0], max: [2.0, 2.0] }),
        ];
        let size = SpatialHash::auto_cell_size(&aabbs);
        // avg max dimension = (1+2)/2 = 1.5, *2 = 3.0
        assert!((size - 3.0).abs() < EPS);
    }

    #[test]
    fn spatial_hash_empty() {
        let size = SpatialHash::auto_cell_size(&[]);
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
}
