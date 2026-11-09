//! Internal types for the 3D physics backend.

use hisab::{DMat3, DQuat, DVec3};

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
    /// Body-space inertia tensor about the body origin (the centre of mass).
    pub inertia: DMat3,
    /// Body-space inverse inertia tensor.
    pub inv_inertia: DMat3,
    /// World-space inverse inertia `R·I⁻¹·Rᵀ`, cached for the solver. Refreshed
    /// whenever the rotation or mass properties change.
    pub inv_inertia_world: DMat3,
    // Split impulse — pseudo-velocities for position correction only
    pub pseudo_velocity: DVec3,
    pub pseudo_angular_velocity: DVec3,
    // Sleep state
    pub is_sleeping: bool,
    pub sleep_timer: f64,
}

impl RigidBody3d {
    #[must_use]
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
            inertia: DMat3::ZERO,
            inv_inertia: DMat3::ZERO,
            inv_inertia_world: DMat3::ZERO,
            pseudo_velocity: DVec3::ZERO,
            pseudo_angular_velocity: DVec3::ZERO,
            is_sleeping: false,
            sleep_timer: 0.0,
        }
    }

    #[inline]
    #[must_use]
    pub(super) fn is_dynamic(&self) -> bool {
        self.body_type == BodyType::Dynamic
    }

    #[inline]
    #[must_use]
    pub(super) fn is_static(&self) -> bool {
        self.body_type == BodyType::Static
    }

    /// World-space inverse inertia `R·I⁻¹·Rᵀ` for the current rotation.
    #[inline]
    #[must_use]
    pub(super) fn compute_inv_inertia_world(&self) -> DMat3 {
        if !self.is_dynamic() || self.fixed_rotation {
            return DMat3::ZERO;
        }
        let r = DMat3::from_quat(self.rotation);
        r * self.inv_inertia * r.transpose()
    }

    /// Refresh the cached [`Self::inv_inertia_world`].
    #[inline]
    pub(super) fn update_inertia_world(&mut self) {
        self.inv_inertia_world = self.compute_inv_inertia_world();
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
            // Work in the body frame, where the inertia tensor is constant.
            let q_inv = self.rotation.inverse();
            let w_body = q_inv * self.angular_velocity;
            let w_body = gyroscopic_step(self.inertia, w_body, dt);
            let tau_body = q_inv * self.torque_accumulator;
            self.angular_velocity = self.rotation * (w_body + self.inv_inertia * tau_body * dt);
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
        if self.is_static() || self.is_sleeping {
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
        self.update_inertia_world();
    }

    #[inline]
    pub(super) fn clear_forces(&mut self) {
        self.force_accumulator = DVec3::ZERO;
        self.torque_accumulator = DVec3::ZERO;
    }
}

/// Skew-symmetric cross-product matrix: `skew(v) * x == v.cross(x)`.
#[inline]
#[must_use]
pub(super) fn skew(v: DVec3) -> DMat3 {
    DMat3::from_cols(
        DVec3::new(0.0, v.z, -v.y),
        DVec3::new(-v.z, 0.0, v.x),
        DVec3::new(v.y, -v.x, 0.0),
    )
}

/// Advance the body-frame angular velocity `w` by the gyroscopic term
/// `I·ẇ = −w × (I·w)` over `dt`.
///
/// Uses one Newton step of the implicit (backward Euler) update, which is
/// stable for fast spins, then rescales the result back onto the pre-step
/// rotational kinetic energy: the gyroscopic term is workless, so it must
/// neither add energy (instability) nor remove it (implicit damping).
#[inline]
#[must_use]
pub(super) fn gyroscopic_step(inertia: DMat3, w: DVec3, dt: f64) -> DVec3 {
    let iw = inertia * w;
    let f = w.cross(iw) * dt;
    let jac = inertia + (skew(w) * inertia - skew(iw)) * dt;
    let det = jac.determinant();
    if !(det.is_finite() && det.abs() > EPSILON_SQ) {
        return w;
    }
    let w2 = w - jac.inverse() * f;
    let e1 = w.dot(iw);
    let e2 = w2.dot(inertia * w2);
    if !(w2.is_finite() && e2.is_finite()) {
        return w;
    }
    if e2 > EPSILON_SQ && e1 >= 0.0 {
        w2 * (e1 / e2).sqrt()
    } else {
        w2
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
    #[must_use]
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

    #[must_use]
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

    /// Local-space AABB `(min, max)` of a point cloud, or `None` if empty.
    #[must_use]
    fn points_aabb(points: impl Iterator<Item = [f64; 3]>) -> Option<(DVec3, DVec3)> {
        let mut min = DVec3::splat(f64::INFINITY);
        let mut max = DVec3::splat(f64::NEG_INFINITY);
        for p in points {
            let v = DVec3::from_array(p);
            min = min.min(v);
            max = max.max(v);
        }
        (min.x <= max.x).then_some((min, max))
    }

    /// Local AABB of a hull / mesh shape with non-zero volume.
    #[must_use]
    fn solid_points_aabb(&self) -> Option<(DVec3, DVec3)> {
        let aabb = match &self.shape {
            ColliderShape::ConvexHull { points } => Self::points_aabb(points.iter().copied()),
            ColliderShape::TriMesh { vertices, .. } => Self::points_aabb(vertices.iter().copied()),
            _ => None,
        }?;
        let size = aabb.1 - aabb.0;
        (size.x * size.y * size.z > EPSILON).then_some(aabb)
    }

    /// Centre of mass of the shape relative to the collider origin.
    ///
    /// Zero for symmetric primitives; the AABB centre for hulls and meshes.
    #[must_use]
    pub(super) fn local_centroid(&self) -> DVec3 {
        self.solid_points_aabb()
            .map_or(DVec3::ZERO, |(min, max)| (min + max) * 0.5)
    }

    /// Collider mass: explicit `mass`, else volume × density.
    ///
    /// Hulls and meshes use the volume of their points' AABB (an upper bound
    /// on the true hull volume); degenerate (flat) point sets, segments and
    /// heightfields fall back to 1 m³.
    #[must_use]
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
            ColliderShape::ConvexHull { .. } | ColliderShape::TriMesh { .. } => {
                self.solid_points_aabb().map_or(1.0, |(min, max)| {
                    let s = max - min;
                    s.x * s.y * s.z
                })
            }
            _ => 1.0,
        };
        (vol * self.material.density).max(MIN_MASS)
    }

    /// Inertia tensor of the shape about its own centre of mass
    /// ([`Self::local_centroid`]), in collider-local axes.
    #[must_use]
    pub(super) fn compute_inertia(&self, mass: f64) -> DMat3 {
        let solid_box = |w: f64, h: f64, d: f64| {
            DVec3::new(
                mass * (h * h + d * d) / 12.0,
                mass * (w * w + d * d) / 12.0,
                mass * (w * w + h * h) / 12.0,
            )
        };
        let diag = match &self.shape {
            ColliderShape::Ball { radius } => {
                let i = 0.4 * mass * radius * radius;
                DVec3::splat(i)
            }
            ColliderShape::Box { half_extents } => solid_box(
                2.0 * half_extents[0],
                2.0 * half_extents[1],
                2.0 * half_extents[2],
            ),
            ColliderShape::Capsule {
                half_height,
                radius,
            } => {
                // Standard capsule tensor (axis along local Y): a cylinder of
                // height h plus two hemispheres, each with its centre of mass
                // 3r/8 from its flat face.
                let r = *radius;
                let r2 = r * r;
                let h = 2.0 * half_height;

                let cyl_vol = std::f64::consts::PI * r2 * h;
                let hemis_vol = (4.0 / 3.0) * std::f64::consts::PI * r2 * r;
                let total_vol = cyl_vol + hemis_vol;
                let (m_cyl, m_hemis) = if total_vol > 0.0 {
                    let f = cyl_vol / total_vol;
                    (mass * f, mass * (1.0 - f))
                } else {
                    (0.0, mass)
                };

                let i_axis = m_cyl * r2 / 2.0 + m_hemis * 2.0 * r2 / 5.0;
                let i_perp = m_cyl * (h * h / 12.0 + r2 / 4.0)
                    + m_hemis * (2.0 * r2 / 5.0 + h * h / 4.0 + 3.0 * h * r / 8.0);
                DVec3::new(i_perp, i_axis, i_perp)
            }
            ColliderShape::ConvexHull { .. } | ColliderShape::TriMesh { .. } => {
                match self.solid_points_aabb() {
                    Some((min, max)) => {
                        let s = max - min;
                        solid_box(s.x, s.y, s.z)
                    }
                    None => DVec3::splat(mass),
                }
            }
            _ => DVec3::splat(mass),
        };
        // Diagonal inertia tensor about the shape's own centre; the body
        // tensor adds the parallel-axis shift for the collider offset.
        let clamped = diag.max(DVec3::splat(MIN_INERTIA));
        DMat3::from_diagonal(clamped)
    }
}

/// Parallel-axis shift of an inertia tensor by displacement `d` for mass `m`:
/// `m (|d|² E − d dᵀ)`.
#[inline]
#[must_use]
pub(super) fn parallel_axis(m: f64, d: DVec3) -> DMat3 {
    let dd = d.dot(d);
    DMat3::from_diagonal(DVec3::splat(m * dd))
        - DMat3::from_cols(d * (m * d.x), d * (m * d.y), d * (m * d.z))
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
    /// Relative rotation `q_a⁻¹ · q_b` when the joint was created; a Fixed
    /// joint holds the bodies at this relative orientation.
    pub reference_rotation: DQuat,
    /// Kept for snapshots; joint motors are not implemented in 3D.
    #[cfg_attr(not(feature = "serialize"), allow(dead_code))]
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
    #[inline]
    #[must_use]
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
