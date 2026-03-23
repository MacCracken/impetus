//! Native 3D physics backend.
//!
//! Implements broadphase (spatial hash), narrowphase (shape-vs-shape contact
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
// 3D vector math helpers
// ---------------------------------------------------------------------------

fn v3_add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn v3_sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn v3_scale(v: [f64; 3], s: f64) -> [f64; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

fn v3_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn v3_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn v3_len(v: [f64; 3]) -> f64 {
    v3_dot(v, v).sqrt()
}

fn v3_normalize(v: [f64; 3]) -> [f64; 3] {
    let l = v3_len(v);
    if l < 1e-10 {
        [0.0, 1.0, 0.0]
    } else {
        v3_scale(v, 1.0 / l)
    }
}

// Quaternion: [x, y, z, w]
fn q_identity() -> [f64; 4] {
    [0.0, 0.0, 0.0, 1.0]
}

fn q_from_z_rotation(angle: f64) -> [f64; 4] {
    let (s, c) = (angle * 0.5).sin_cos();
    [0.0, 0.0, s, c]
}

fn q_rotate_vec(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let qv = [q[0], q[1], q[2]];
    let w = q[3];
    let t = v3_scale(v3_cross(qv, v), 2.0);
    v3_add(v3_add(v, v3_scale(t, w)), v3_cross(qv, t))
}

fn q_multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

fn q_normalize(q: [f64; 4]) -> [f64; 4] {
    let l = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if l < 1e-10 {
        q_identity()
    } else {
        [q[0] / l, q[1] / l, q[2] / l, q[3] / l]
    }
}

// ---------------------------------------------------------------------------
// Internal body representation
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub(crate) struct RigidBody3d {
    pub handle: BodyHandle,
    pub body_type: BodyType,
    pub position: [f64; 3],
    pub rotation: [f64; 4], // quaternion [x, y, z, w]
    pub linear_velocity: [f64; 3],
    pub angular_velocity: [f64; 3],
    pub linear_damping: f64,
    pub angular_damping: f64,
    pub fixed_rotation: bool,
    pub gravity_scale: f64,
    pub force_accumulator: [f64; 3],
    pub torque_accumulator: [f64; 3],
    pub mass: f64,
    pub inv_mass: f64,
    pub inertia: [f64; 3], // diagonal inertia tensor
    pub inv_inertia: [f64; 3],
}

impl RigidBody3d {
    fn from_desc(handle: BodyHandle, desc: &BodyDesc) -> Self {
        Self {
            handle,
            body_type: desc.body_type,
            position: desc.position,
            rotation: q_from_z_rotation(desc.rotation),
            linear_velocity: desc.linear_velocity,
            angular_velocity: [0.0, 0.0, desc.angular_velocity],
            linear_damping: desc.linear_damping,
            angular_damping: desc.angular_damping,
            fixed_rotation: desc.fixed_rotation,
            gravity_scale: desc.gravity_scale.unwrap_or(1.0),
            force_accumulator: [0.0, 0.0, 0.0],
            torque_accumulator: [0.0, 0.0, 0.0],
            mass: 0.0,
            inv_mass: 0.0,
            inertia: [0.0, 0.0, 0.0],
            inv_inertia: [0.0, 0.0, 0.0],
        }
    }

    fn is_dynamic(&self) -> bool {
        self.body_type == BodyType::Dynamic
    }

    fn is_static(&self) -> bool {
        self.body_type == BodyType::Static
    }

    fn integrate_velocities(&mut self, gravity: [f64; 3], dt: f64) {
        if !self.is_dynamic() || self.inv_mass == 0.0 {
            return;
        }

        self.linear_velocity = v3_add(
            self.linear_velocity,
            v3_scale(gravity, self.gravity_scale * dt),
        );
        self.linear_velocity = v3_add(
            self.linear_velocity,
            v3_scale(self.force_accumulator, self.inv_mass * dt),
        );

        if !self.fixed_rotation {
            let torque_effect = [
                self.torque_accumulator[0] * self.inv_inertia[0] * dt,
                self.torque_accumulator[1] * self.inv_inertia[1] * dt,
                self.torque_accumulator[2] * self.inv_inertia[2] * dt,
            ];
            self.angular_velocity = v3_add(self.angular_velocity, torque_effect);
        }

        let damp = 1.0 / (1.0 + dt * self.linear_damping);
        self.linear_velocity = v3_scale(self.linear_velocity, damp);
        let adamp = 1.0 / (1.0 + dt * self.angular_damping);
        self.angular_velocity = v3_scale(self.angular_velocity, adamp);
    }

    fn integrate_positions(&mut self, dt: f64) {
        if self.is_static() {
            return;
        }
        if self.is_dynamic() && self.inv_mass == 0.0 {
            return;
        }

        self.position = v3_add(self.position, v3_scale(self.linear_velocity, dt));

        if !self.fixed_rotation {
            let w = self.angular_velocity;
            let half_dt = dt * 0.5;
            let dq = q_multiply([w[0] * half_dt, w[1] * half_dt, w[2] * half_dt, 0.0], self.rotation);
            self.rotation = q_normalize([
                self.rotation[0] + dq[0],
                self.rotation[1] + dq[1],
                self.rotation[2] + dq[2],
                self.rotation[3] + dq[3],
            ]);
        }
    }

    fn clear_forces(&mut self) {
        self.force_accumulator = [0.0, 0.0, 0.0];
        self.torque_accumulator = [0.0, 0.0, 0.0];
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
    pub offset: [f64; 3],
    pub material: PhysicsMaterial,
    pub is_sensor: bool,
    pub mass: Option<f64>,
}

impl Collider3d {
    fn from_desc(handle: ColliderHandle, body: BodyHandle, desc: &ColliderDesc) -> Self {
        Self {
            handle,
            body,
            shape: desc.shape.clone(),
            offset: desc.offset,
            material: desc.material.clone(),
            is_sensor: desc.is_sensor,
            mass: desc.mass,
        }
    }

    fn world_aabb(&self, body_pos: [f64; 3], body_rot: [f64; 4]) -> Aabb3d {
        let wp = v3_add(body_pos, q_rotate_vec(body_rot, self.offset));

        match &self.shape {
            ColliderShape::Ball { radius } => Aabb3d {
                min: v3_sub(wp, [*radius, *radius, *radius]),
                max: v3_add(wp, [*radius, *radius, *radius]),
            },
            ColliderShape::Box { half_extents } => {
                // Conservative AABB for rotated box
                let he = *half_extents;
                let corners = [
                    [-he[0], -he[1], -he[2]],
                    [he[0], -he[1], -he[2]],
                    [-he[0], he[1], -he[2]],
                    [he[0], he[1], -he[2]],
                    [-he[0], -he[1], he[2]],
                    [he[0], -he[1], he[2]],
                    [-he[0], he[1], he[2]],
                    [he[0], he[1], he[2]],
                ];
                let mut min = [f64::INFINITY; 3];
                let mut max = [f64::NEG_INFINITY; 3];
                for c in &corners {
                    let wc = v3_add(wp, q_rotate_vec(body_rot, *c));
                    for i in 0..3 {
                        min[i] = min[i].min(wc[i]);
                        max[i] = max[i].max(wc[i]);
                    }
                }
                Aabb3d { min, max }
            }
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                let axis = q_rotate_vec(body_rot, [0.0, *half_height, 0.0]);
                let mut min = [f64::INFINITY; 3];
                let mut max = [f64::NEG_INFINITY; 3];
                for i in 0..3 {
                    min[i] = wp[i] - axis[i].abs() - radius;
                    max[i] = wp[i] + axis[i].abs() + radius;
                }
                Aabb3d { min, max }
            }
            _ => Aabb3d {
                min: wp,
                max: wp,
            },
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

    fn compute_inertia(&self, mass: f64) -> [f64; 3] {
        let i = match &self.shape {
            ColliderShape::Ball { radius } => {
                let i = 0.4 * mass * radius * radius;
                [i, i, i]
            }
            ColliderShape::Box { half_extents } => {
                let w = 2.0 * half_extents[0];
                let h = 2.0 * half_extents[1];
                let d = 2.0 * half_extents[2];
                [
                    mass * (h * h + d * d) / 12.0,
                    mass * (w * w + d * d) / 12.0,
                    mass * (w * w + h * h) / 12.0,
                ]
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
                [ix, iy, iz]
            }
            _ => [mass, mass, mass],
        };
        [i[0].max(1e-10), i[1].max(1e-10), i[2].max(1e-10)]
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
    pub local_anchor_a: [f64; 3],
    pub local_anchor_b: [f64; 3],
}

// ---------------------------------------------------------------------------
// 3D AABB
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy)]
pub(crate) struct Aabb3d {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl Aabb3d {
    fn overlaps(&self, other: &Aabb3d) -> bool {
        self.min[0] <= other.max[0]
            && self.max[0] >= other.min[0]
            && self.min[1] <= other.max[1]
            && self.max[1] >= other.min[1]
            && self.min[2] <= other.max[2]
            && self.max[2] >= other.min[2]
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
                let w = aabb.max[0] - aabb.min[0];
                let h = aabb.max[1] - aabb.min[1];
                let d = aabb.max[2] - aabb.min[2];
                w.max(h).max(d)
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
        let (min_cx, min_cy, min_cz) = self.cell(aabb.min[0], aabb.min[1], aabb.min[2]);
        let (max_cx, max_cy, max_cz) = self.cell(aabb.max[0], aabb.max[1], aabb.max[2]);

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
    pub normal: [f64; 3],
    pub depth: f64,
    pub point: [f64; 3],
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
            for (ri, ci) in rb.inertia.iter_mut().zip(c_inertia.iter()) {
                *ri += ci;
            }
            rb.inv_mass = 1.0 / rb.mass;
            for (inv, ine) in rb.inv_inertia.iter_mut().zip(rb.inertia.iter()) {
                *inv = if rb.fixed_rotation { 0.0 } else { 1.0 / ine };
            }
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
                local_anchor_a: [desc.local_anchor_a[0], desc.local_anchor_a[1], 0.0],
                local_anchor_b: [desc.local_anchor_b[0], desc.local_anchor_b[1], 0.0],
            },
        );
    }

    pub fn apply_force(&mut self, body: BodyHandle, force: &Force) {
        if let Some(rb) = self.bodies.get_mut(&body) {
            rb.force_accumulator = v3_add(rb.force_accumulator, force.vector);
            if let Some(point) = force.point {
                rb.torque_accumulator =
                    v3_add(rb.torque_accumulator, v3_cross(point, force.vector));
            }
        }
    }

    pub fn apply_impulse(&mut self, body: BodyHandle, impulse: &Impulse) {
        if let Some(rb) = self.bodies.get_mut(&body)
            && rb.is_dynamic()
            && rb.inv_mass > 0.0
        {
            rb.linear_velocity = v3_add(
                rb.linear_velocity,
                v3_scale(impulse.vector, rb.inv_mass),
            );
            if let Some(point) = impulse.point {
                let ang = v3_cross(point, impulse.vector);
                rb.angular_velocity = v3_add(
                    rb.angular_velocity,
                    [
                        ang[0] * rb.inv_inertia[0],
                        ang[1] * rb.inv_inertia[1],
                        ang[2] * rb.inv_inertia[2],
                    ],
                );
            }
        }
    }

    pub fn apply_torque(&mut self, body: BodyHandle, torque: &Torque) {
        if let Some(rb) = self.bodies.get_mut(&body) {
            rb.torque_accumulator[2] += torque.value;
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
            position: rb.position,
            rotation: rb.rotation[2].atan2(rb.rotation[3]) * 2.0, // extract z-rotation
            linear_velocity: rb.linear_velocity,
            angular_velocity: rb.angular_velocity[2],
            is_sleeping: false,
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
        for rb in self.bodies.values_mut() {
            rb.integrate_velocities(gravity, dt);
        }

        let broad_pairs = self.broadphase();
        let contacts = self.narrowphase(&broad_pairs);
        self.solve_contacts(&contacts, velocity_iterations);
        self.solve_joints(dt, velocity_iterations);
        self.solve_positions(&contacts, position_iterations);

        for rb in self.bodies.values_mut() {
            rb.integrate_positions(dt);
        }
        for rb in self.bodies.values_mut() {
            rb.clear_forces();
        }

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

            let pos_a = v3_add(ba.position, q_rotate_vec(ba.rotation, ca.offset));
            let pos_b = v3_add(bb.position, q_rotate_vec(bb.rotation, cb.offset));

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
                let ra = v3_sub(cp, pos_a);
                let rb = v3_sub(cp, pos_b);

                let vel_a_at_cp = v3_add(vel_a, v3_cross(angvel_a, ra));
                let vel_b_at_cp = v3_add(vel_b, v3_cross(angvel_b, rb));
                let rel_vel = v3_sub(vel_b_at_cp, vel_a_at_cp);
                let vel_along_normal = v3_dot(rel_vel, n);

                if vel_along_normal > 0.0 {
                    continue;
                }

                let ra_cross_n = v3_cross(ra, n);
                let rb_cross_n = v3_cross(rb, n);
                let ang_eff_a = v3_dot(
                    ra_cross_n,
                    [
                        ra_cross_n[0] * inv_inertia_a[0],
                        ra_cross_n[1] * inv_inertia_a[1],
                        ra_cross_n[2] * inv_inertia_a[2],
                    ],
                );
                let ang_eff_b = v3_dot(
                    rb_cross_n,
                    [
                        rb_cross_n[0] * inv_inertia_b[0],
                        rb_cross_n[1] * inv_inertia_b[1],
                        rb_cross_n[2] * inv_inertia_b[2],
                    ],
                );
                let inv_mass_sum = inv_mass_a + inv_mass_b + ang_eff_a + ang_eff_b;

                let j = -(1.0 + materials[ci].restitution) * vel_along_normal / inv_mass_sum;
                let impulse_n = v3_scale(n, j);

                if let Some(ba) = self.bodies.get_mut(&contact.body_a)
                    && ba.is_dynamic()
                {
                    ba.linear_velocity = v3_sub(ba.linear_velocity, v3_scale(impulse_n, ba.inv_mass));
                    let ang_imp = v3_cross(ra, impulse_n);
                    ba.angular_velocity = v3_sub(
                        ba.angular_velocity,
                        [
                            ang_imp[0] * ba.inv_inertia[0],
                            ang_imp[1] * ba.inv_inertia[1],
                            ang_imp[2] * ba.inv_inertia[2],
                        ],
                    );
                }
                if let Some(bb) = self.bodies.get_mut(&contact.body_b)
                    && bb.is_dynamic()
                {
                    bb.linear_velocity = v3_add(bb.linear_velocity, v3_scale(impulse_n, bb.inv_mass));
                    let ang_imp = v3_cross(rb, impulse_n);
                    bb.angular_velocity = v3_add(
                        bb.angular_velocity,
                        [
                            ang_imp[0] * bb.inv_inertia[0],
                            ang_imp[1] * bb.inv_inertia[1],
                            ang_imp[2] * bb.inv_inertia[2],
                        ],
                    );
                }

                // Friction
                let friction = materials[ci].friction;
                if friction > 0.0 {
                    let tangent_vel = v3_sub(rel_vel, v3_scale(n, vel_along_normal));
                    let tangent_speed = v3_len(tangent_vel);
                    if tangent_speed > 1e-10 {
                        let tangent = v3_scale(tangent_vel, 1.0 / tangent_speed);
                        let jt = (-tangent_speed / inv_mass_sum)
                            .clamp(-j.abs() * friction, j.abs() * friction);
                        let impulse_t = v3_scale(tangent, jt);

                        if let Some(ba) = self.bodies.get_mut(&contact.body_a)
                            && ba.is_dynamic()
                        {
                            ba.linear_velocity =
                                v3_sub(ba.linear_velocity, v3_scale(impulse_t, ba.inv_mass));
                            let ang_t = v3_cross(ra, impulse_t);
                            ba.angular_velocity = v3_sub(
                                ba.angular_velocity,
                                [
                                    ang_t[0] * ba.inv_inertia[0],
                                    ang_t[1] * ba.inv_inertia[1],
                                    ang_t[2] * ba.inv_inertia[2],
                                ],
                            );
                        }
                        if let Some(bb) = self.bodies.get_mut(&contact.body_b)
                            && bb.is_dynamic()
                        {
                            bb.linear_velocity =
                                v3_add(bb.linear_velocity, v3_scale(impulse_t, bb.inv_mass));
                            let ang_t = v3_cross(rb, impulse_t);
                            bb.angular_velocity = v3_add(
                                bb.angular_velocity,
                                [
                                    ang_t[0] * bb.inv_inertia[0],
                                    ang_t[1] * bb.inv_inertia[1],
                                    ang_t[2] * bb.inv_inertia[2],
                                ],
                            );
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
                let correction = v3_scale(contact.normal, correction_mag);

                if let Some(ba) = self.bodies.get_mut(&contact.body_a)
                    && ba.is_dynamic()
                {
                    ba.position = v3_sub(ba.position, v3_scale(correction, ba.inv_mass));
                }
                if let Some(bb) = self.bodies.get_mut(&contact.body_b)
                    && bb.is_dynamic()
                {
                    bb.position = v3_add(bb.position, v3_scale(correction, bb.inv_mass));
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

    fn world_anchor_3d(&self, body: BodyHandle, local: [f64; 3]) -> [f64; 3] {
        let rb = match self.bodies.get(&body) {
            Some(b) => b,
            None => return local,
        };
        v3_add(rb.position, q_rotate_vec(rb.rotation, local))
    }

    fn solve_fixed_joint_3d(&mut self, joint: &Joint3d) {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);
        let diff = v3_sub(anchor_b, anchor_a);

        if let Some(ba) = self.bodies.get_mut(&joint.body_a)
            && ba.is_dynamic()
        {
            ba.position = v3_add(ba.position, v3_scale(diff, 0.5));
        }
        if let Some(bb) = self.bodies.get_mut(&joint.body_b)
            && bb.is_dynamic()
        {
            bb.position = v3_sub(bb.position, v3_scale(diff, 0.5));
        }
    }

    fn solve_distance_joint_3d(&mut self, joint: &Joint3d, length: f64) {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);
        let diff = v3_sub(anchor_b, anchor_a);
        let dist = v3_len(diff);

        if dist < 1e-10 {
            return;
        }

        let n = v3_scale(diff, 1.0 / dist);
        let correction = (dist - length) * 0.5;

        if let Some(ba) = self.bodies.get_mut(&joint.body_a)
            && ba.is_dynamic()
        {
            ba.position = v3_add(ba.position, v3_scale(n, correction));
        }
        if let Some(bb) = self.bodies.get_mut(&joint.body_b)
            && bb.is_dynamic()
        {
            bb.position = v3_sub(bb.position, v3_scale(n, correction));
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
        let diff = v3_sub(anchor_b, anchor_a);
        let dist = v3_len(diff);

        if dist < 1e-10 {
            return;
        }

        let n = v3_scale(diff, 1.0 / dist);
        let spring_force = stiffness * (dist - rest_length);

        let vel_a = self
            .bodies
            .get(&joint.body_a)
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0, 0.0]);
        let vel_b = self
            .bodies
            .get(&joint.body_b)
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0, 0.0]);
        let rel_vel = v3_sub(vel_b, vel_a);
        let damping_force = damping * v3_dot(rel_vel, n);

        let total_force = spring_force + damping_force;
        let force = v3_scale(n, total_force * dt);

        if let Some(ba) = self.bodies.get_mut(&joint.body_a)
            && ba.is_dynamic()
        {
            ba.linear_velocity = v3_add(ba.linear_velocity, v3_scale(force, ba.inv_mass));
        }
        if let Some(bb) = self.bodies.get_mut(&joint.body_b)
            && bb.is_dynamic()
        {
            bb.linear_velocity = v3_sub(bb.linear_velocity, v3_scale(force, bb.inv_mass));
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
        let dir = v3_normalize(direction);

        let mut best: Option<(f64, ColliderHandle, [f64; 3], [f64; 3])> = None;

        for collider in self.colliders.values() {
            let rb = match self.bodies.get(&collider.body) {
                Some(b) => b,
                None => continue,
            };
            let pos = v3_add(rb.position, q_rotate_vec(rb.rotation, collider.offset));

            let hit = match &collider.shape {
                ColliderShape::Ball { radius } => ray_sphere(origin, dir, pos, *radius),
                ColliderShape::Box { half_extents } => ray_aabb_3d(
                    origin,
                    dir,
                    v3_sub(pos, *half_extents),
                    v3_add(pos, *half_extents),
                ),
                _ => None,
            };

            if let Some((t, normal)) = hit
                && t >= 0.0
                && t <= max_dist
                && (best.is_none() || t < best.as_ref().unwrap().0)
            {
                let point = v3_add(origin, v3_scale(dir, t));
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
// Narrowphase contact generation
// ---------------------------------------------------------------------------

fn generate_contact_3d(
    shape_a: &ColliderShape,
    pos_a: [f64; 3],
    shape_b: &ColliderShape,
    pos_b: [f64; 3],
) -> Option<([f64; 3], f64, [f64; 3])> {
    match (shape_a, shape_b) {
        (ColliderShape::Ball { radius: ra }, ColliderShape::Ball { radius: rb }) => {
            sphere_sphere(pos_a, *ra, pos_b, *rb)
        }
        (ColliderShape::Ball { radius }, ColliderShape::Box { half_extents }) => {
            sphere_aabb(pos_a, *radius, pos_b, *half_extents)
        }
        (ColliderShape::Box { half_extents }, ColliderShape::Ball { radius }) => {
            sphere_aabb(pos_b, *radius, pos_a, *half_extents)
                .map(|(n, d, p)| (v3_scale(n, -1.0), d, p))
        }
        (
            ColliderShape::Box { half_extents: he_a },
            ColliderShape::Box { half_extents: he_b },
        ) => aabb_aabb_3d(pos_a, *he_a, pos_b, *he_b),
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
                .map(|(n, d, p)| (v3_scale(n, -1.0), d, p))
        }
        _ => None,
    }
}

fn sphere_sphere(
    pos_a: [f64; 3],
    ra: f64,
    pos_b: [f64; 3],
    rb: f64,
) -> Option<([f64; 3], f64, [f64; 3])> {
    let d = v3_sub(pos_b, pos_a);
    let dist_sq = v3_dot(d, d);
    let sum_r = ra + rb;

    if dist_sq >= sum_r * sum_r {
        return None;
    }

    let dist = dist_sq.sqrt();
    let (normal, depth) = if dist < 1e-10 {
        ([0.0, 1.0, 0.0], sum_r)
    } else {
        (v3_scale(d, 1.0 / dist), sum_r - dist)
    };

    let point = v3_add(pos_a, v3_scale(normal, ra));
    Some((normal, depth, point))
}

fn sphere_aabb(
    sphere_pos: [f64; 3],
    radius: f64,
    box_pos: [f64; 3],
    half_extents: [f64; 3],
) -> Option<([f64; 3], f64, [f64; 3])> {
    let d = v3_sub(sphere_pos, box_pos);
    let closest = [
        d[0].clamp(-half_extents[0], half_extents[0]),
        d[1].clamp(-half_extents[1], half_extents[1]),
        d[2].clamp(-half_extents[2], half_extents[2]),
    ];
    let diff = v3_sub(d, closest);
    let dist_sq = v3_dot(diff, diff);

    if dist_sq >= radius * radius {
        return None;
    }

    let dist = dist_sq.sqrt();
    let (normal, depth) = if dist < 1e-10 {
        let face_dists = [
            half_extents[0] - d[0].abs(),
            half_extents[1] - d[1].abs(),
            half_extents[2] - d[2].abs(),
        ];
        let min_axis = if face_dists[0] <= face_dists[1] && face_dists[0] <= face_dists[2] {
            0
        } else if face_dists[1] <= face_dists[2] {
            1
        } else {
            2
        };
        let mut n = [0.0, 0.0, 0.0];
        n[min_axis] = if d[min_axis] >= 0.0 { 1.0 } else { -1.0 };
        (n, face_dists[min_axis] + radius)
    } else {
        (v3_scale(diff, 1.0 / dist), radius - dist)
    };

    let point = v3_add(box_pos, closest);
    Some((normal, depth, point))
}

fn aabb_aabb_3d(
    pos_a: [f64; 3],
    he_a: [f64; 3],
    pos_b: [f64; 3],
    he_b: [f64; 3],
) -> Option<([f64; 3], f64, [f64; 3])> {
    let d = v3_sub(pos_b, pos_a);
    let overlap = [
        he_a[0] + he_b[0] - d[0].abs(),
        he_a[1] + he_b[1] - d[1].abs(),
        he_a[2] + he_b[2] - d[2].abs(),
    ];

    if overlap[0] <= 0.0 || overlap[1] <= 0.0 || overlap[2] <= 0.0 {
        return None;
    }

    let min_axis = if overlap[0] <= overlap[1] && overlap[0] <= overlap[2] {
        0
    } else if overlap[1] <= overlap[2] {
        1
    } else {
        2
    };

    let mut normal = [0.0, 0.0, 0.0];
    normal[min_axis] = if d[min_axis] >= 0.0 { 1.0 } else { -1.0 };
    let depth = overlap[min_axis];

    let mut point = pos_a;
    point[min_axis] += normal[min_axis] * he_a[min_axis];

    Some((normal, depth, point))
}

// ---------------------------------------------------------------------------
// Capsule helpers

fn closest_point_on_segment_3d(a: [f64; 3], b: [f64; 3], p: [f64; 3]) -> [f64; 3] {
    let ab = v3_sub(b, a);
    let len_sq = v3_dot(ab, ab);
    if len_sq < 1e-20 {
        return a;
    }
    let t = (v3_dot(v3_sub(p, a), ab) / len_sq).clamp(0.0, 1.0);
    v3_add(a, v3_scale(ab, t))
}

fn capsule_sphere_3d(
    cap_pos: [f64; 3],
    half_height: f64,
    cap_radius: f64,
    sphere_pos: [f64; 3],
    sphere_radius: f64,
) -> Option<([f64; 3], f64, [f64; 3])> {
    // Capsule axis along Y in local space (no rotation transform here — pos is world center)
    let ep_a = v3_add(cap_pos, [0.0, -half_height, 0.0]);
    let ep_b = v3_add(cap_pos, [0.0, half_height, 0.0]);
    let closest = closest_point_on_segment_3d(ep_a, ep_b, sphere_pos);
    sphere_sphere(closest, cap_radius, sphere_pos, sphere_radius)
}

// ---------------------------------------------------------------------------
// Ray helpers
// ---------------------------------------------------------------------------

fn ray_sphere(
    origin: [f64; 3],
    dir: [f64; 3],
    center: [f64; 3],
    radius: f64,
) -> Option<(f64, [f64; 3])> {
    let oc = v3_sub(origin, center);
    let half_b = v3_dot(oc, dir);
    let c = v3_dot(oc, oc) - radius * radius;
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

    let point = v3_add(origin, v3_scale(dir, t));
    let normal = v3_normalize(v3_sub(point, center));
    Some((t, normal))
}

fn ray_aabb_3d(
    origin: [f64; 3],
    dir: [f64; 3],
    min: [f64; 3],
    max: [f64; 3],
) -> Option<(f64, [f64; 3])> {
    let mut t_min = f64::NEG_INFINITY;
    let mut t_max = f64::INFINITY;
    let mut normal = [0.0, 0.0, 0.0];

    for i in 0..3 {
        if dir[i].abs() < 1e-10 {
            if origin[i] < min[i] || origin[i] > max[i] {
                return None;
            }
        } else {
            let inv_d = 1.0 / dir[i];
            let mut t1 = (min[i] - origin[i]) * inv_d;
            let mut t2 = (max[i] - origin[i]) * inv_d;
            let mut n = [0.0, 0.0, 0.0];
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
        let r = sphere_sphere([0.0, 0.0, 0.0], 1.0, [1.5, 0.0, 0.0], 1.0);
        assert!(r.is_some());
        let (n, d, _) = r.unwrap();
        assert!((n[0] - 1.0).abs() < EPS);
        assert!((d - 0.5).abs() < EPS);
    }

    #[test]
    fn sphere_sphere_no_overlap() {
        assert!(sphere_sphere([0.0, 0.0, 0.0], 1.0, [5.0, 0.0, 0.0], 1.0).is_none());
    }

    #[test]
    fn sphere_aabb_overlap() {
        let r = sphere_aabb([1.8, 0.0, 0.0], 0.5, [0.0, 0.0, 0.0], [1.5, 1.0, 1.0]);
        assert!(r.is_some());
    }

    #[test]
    fn sphere_aabb_miss() {
        assert!(sphere_aabb([5.0, 0.0, 0.0], 0.5, [0.0, 0.0, 0.0], [1.0, 1.0, 1.0]).is_none());
    }

    #[test]
    fn aabb_3d_overlap() {
        let r = aabb_aabb_3d([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [1.5, 0.0, 0.0], [1.0, 1.0, 1.0]);
        assert!(r.is_some());
        let (n, d, _) = r.unwrap();
        assert!((n[0] - 1.0).abs() < EPS);
        assert!((d - 0.5).abs() < EPS);
    }

    #[test]
    fn aabb_3d_no_overlap() {
        assert!(aabb_aabb_3d(
            [0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
            [5.0, 0.0, 0.0],
            [1.0, 1.0, 1.0]
        )
        .is_none());
    }

    #[test]
    fn ray_sphere_hit() {
        let r = ray_sphere([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [5.0, 0.0, 0.0], 1.0);
        assert!(r.is_some());
        let (t, _) = r.unwrap();
        assert!((t - 4.0).abs() < EPS);
    }

    #[test]
    fn ray_sphere_miss() {
        assert!(ray_sphere([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [5.0, 0.0, 0.0], 1.0).is_none());
    }

    #[test]
    fn ray_aabb_3d_hit() {
        let r = ray_aabb_3d(
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [4.0, -1.0, -1.0],
            [6.0, 1.0, 1.0],
        );
        assert!(r.is_some());
        let (t, _) = r.unwrap();
        assert!((t - 4.0).abs() < EPS);
    }

    #[test]
    fn quaternion_identity_rotation() {
        let q = q_identity();
        let v = [1.0, 0.0, 0.0];
        let result = q_rotate_vec(q, v);
        assert!((result[0] - 1.0).abs() < EPS);
        assert!(result[1].abs() < EPS);
        assert!(result[2].abs() < EPS);
    }

    #[test]
    fn quaternion_z_rotation() {
        let q = q_from_z_rotation(std::f64::consts::FRAC_PI_2);
        let v = [1.0, 0.0, 0.0];
        let result = q_rotate_vec(q, v);
        assert!(result[0].abs() < EPS);
        assert!((result[1] - 1.0).abs() < EPS);
        assert!(result[2].abs() < EPS);
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
            },
        );

        for _ in 0..60 {
            state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1);
        }

        assert!(state.bodies[&bh].position[1] < 10.0, "body should fall");
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
        });
        let mass_first = state.bodies[&bh].mass;

        state.add_collider(ColliderHandle(1), bh, &ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [2.0, 0.0, 0.0],
            material: PhysicsMaterial { density: 1.0, ..PhysicsMaterial::default() },
            is_sensor: false,
            mass: None,
        });
        assert!((state.bodies[&bh].mass - 2.0 * mass_first).abs() < EPS);
    }

    #[test]
    fn capsule_sphere_3d_overlap() {
        let r = capsule_sphere_3d([0.0, 0.0, 0.0], 1.0, 0.5, [0.8, 0.0, 0.0], 0.5);
        assert!(r.is_some());
    }

    #[test]
    fn capsule_sphere_3d_miss() {
        assert!(capsule_sphere_3d([0.0, 0.0, 0.0], 1.0, 0.5, [5.0, 0.0, 0.0], 0.5).is_none());
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
        });

        state.apply_impulse(bh, &Impulse::new(10.0, 0.0, 0.0));
        assert!(state.bodies[&bh].linear_velocity[0] > 0.0);
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
        });
        state.add_joint(JointHandle(0), &JointDesc {
            body_a: a,
            body_b: b,
            joint_type: JointType::Fixed,
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
        });

        for _ in 0..10 {
            state.step([0.0, -9.81, 0.0], 1.0 / 60.0, 4, 1);
        }
        // Joint should prevent body from falling far
        assert!(state.bodies[&b].position[1] > 2.0);
    }

    #[test]
    fn spatial_hash_3d_finds_pair() {
        let mut grid = SpatialHash3d::new(2.0);
        grid.insert(ColliderHandle(0), &Aabb3d {
            min: [0.0, 0.0, 0.0],
            max: [1.0, 1.0, 1.0],
        });
        grid.insert(ColliderHandle(1), &Aabb3d {
            min: [0.5, 0.5, 0.5],
            max: [1.5, 1.5, 1.5],
        });
        let pairs = grid.query_pairs();
        assert!(pairs.contains(&(ColliderHandle(0), ColliderHandle(1))));
    }

    #[test]
    fn spatial_hash_3d_no_false_pair() {
        let mut grid = SpatialHash3d::new(1.0);
        grid.insert(ColliderHandle(0), &Aabb3d {
            min: [0.0, 0.0, 0.0],
            max: [0.5, 0.5, 0.5],
        });
        grid.insert(ColliderHandle(1), &Aabb3d {
            min: [10.0, 10.0, 10.0],
            max: [10.5, 10.5, 10.5],
        });
        assert!(grid.query_pairs().is_empty());
    }
}
