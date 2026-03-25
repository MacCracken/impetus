//! Internal types for the 3D physics backend.

use hisab::{DQuat, DVec3};

use crate::body::{BodyHandle, BodyType};
use crate::collider::{ColliderDesc, ColliderHandle, ColliderShape};
use crate::joint::{JointMotor, JointType};
use crate::material::PhysicsMaterial;

// ---------------------------------------------------------------------------
// Named constants
// ---------------------------------------------------------------------------

/// General-purpose geometry epsilon for zero-length checks.
pub(super) const EPSILON: f64 = 1e-10;
/// Squared epsilon for distance-squared checks.
pub(super) const EPSILON_SQ: f64 = 1e-20;
/// Bodies with both linear and angular speed below this are candidates for sleep.
pub(super) const SLEEP_VELOCITY_THRESHOLD_3D: f64 = 0.01;
/// How many seconds of low motion before a body is put to sleep.
pub(super) const SLEEP_TIME_THRESHOLD_3D: f64 = 0.5;
/// Minimum mass to avoid division by zero for dynamic bodies.
pub(super) const MIN_MASS: f64 = 1e-6;
/// Minimum inertia to avoid division by zero.
pub(super) const MIN_INERTIA: f64 = 1e-10;

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
    // Split impulse — pseudo-velocities for position correction only
    pub pseudo_velocity: DVec3,
    pub pseudo_angular_velocity: DVec3,
    // Sleep state
    pub is_sleeping: bool,
    pub sleep_timer: f64,
}

impl RigidBody3d {
    pub(super) fn from_desc(handle: BodyHandle, desc: &crate::body::BodyDesc) -> Self {
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
            pseudo_velocity: DVec3::ZERO,
            pseudo_angular_velocity: DVec3::ZERO,
            is_sleeping: false,
            sleep_timer: 0.0,
        }
    }

    pub(super) fn is_dynamic(&self) -> bool {
        self.body_type == BodyType::Dynamic
    }

    pub(super) fn is_static(&self) -> bool {
        self.body_type == BodyType::Static
    }

    pub(super) fn integrate_velocities(&mut self, gravity: DVec3, dt: f64, max_velocity: f64) {
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

    pub(super) fn integrate_positions(&mut self, dt: f64) {
        if self.is_static() {
            return;
        }
        if self.is_dynamic() && self.inv_mass == 0.0 {
            return;
        }

        // Real velocity + split impulse pseudo-velocity
        let total_linear = self.linear_velocity + self.pseudo_velocity;
        self.position += total_linear * dt;

        if !self.fixed_rotation {
            let w = self.angular_velocity + self.pseudo_angular_velocity;
            let half_dt = dt * 0.5;
            let dq =
                DQuat::from_xyzw(w.x * half_dt, w.y * half_dt, w.z * half_dt, 0.0) * self.rotation;
            self.rotation = DQuat::from_xyzw(
                self.rotation.x + dq.x,
                self.rotation.y + dq.y,
                self.rotation.z + dq.z,
                self.rotation.w + dq.w,
            )
            .normalize();
        }

        // Clear pseudo-velocities after integration
        self.pseudo_velocity = DVec3::ZERO;
        self.pseudo_angular_velocity = DVec3::ZERO;
    }

    pub(super) fn clear_forces(&mut self) {
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
    pub(super) fn from_desc(handle: ColliderHandle, body: BodyHandle, desc: &ColliderDesc) -> Self {
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

    pub(crate) fn world_aabb(&self, body_pos: DVec3, body_rot: DQuat) -> Aabb3d {
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
                if min.x > max.x {
                    Aabb3d { min: wp, max: wp }
                } else {
                    Aabb3d { min, max }
                }
            }
            ColliderShape::ConvexHull { points } => {
                let mut min = DVec3::splat(f64::INFINITY);
                let mut max = DVec3::splat(f64::NEG_INFINITY);
                for p in points {
                    let wv = wp + body_rot * DVec3::new(p[0], p[1], p[2]);
                    min = min.min(wv);
                    max = max.max(wv);
                }
                if min.x > max.x {
                    Aabb3d { min: wp, max: wp }
                } else {
                    Aabb3d { min, max }
                }
            }
            ColliderShape::Segment { a, b } => {
                let wa = wp + body_rot * DVec3::from_array(*a);
                let wb = wp + body_rot * DVec3::from_array(*b);
                Aabb3d {
                    min: wa.min(wb),
                    max: wa.max(wb),
                }
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

    pub(super) fn compute_mass(&self) -> f64 {
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

    pub(super) fn compute_inertia(&self, mass: f64) -> DVec3 {
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
    pub break_force: Option<f64>,
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
    pub(super) fn overlaps(&self, other: &Aabb3d) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
            && self.min.z <= other.max.z
            && self.max.z >= other.min.z
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
