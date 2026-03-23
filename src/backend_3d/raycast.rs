//! Raycast and overlap query functions for the 3D physics backend.

use hisab::DVec3;

use crate::collider::{ColliderHandle, ColliderShape};
use crate::query::RayHit;

use super::types::EPSILON;
use super::state::PhysicsState3d;
use super::body_ah;

impl PhysicsState3d {
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
        let origin = DVec3::from_array(origin);
        let dir = DVec3::from_array(direction).normalize_or(DVec3::Y);

        let mut best: Option<(f64, ColliderHandle, DVec3, DVec3)> = None;

        for collider in self.colliders.values() {
            if (collider.collision_layer & layer_mask) == 0 {
                continue;
            }
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
// Ray helpers
// ---------------------------------------------------------------------------

pub(super) fn ray_sphere(
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

pub(super) fn ray_aabb_3d(
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
