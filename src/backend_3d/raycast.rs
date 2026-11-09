//! Raycast and overlap query functions for the 3D physics backend.

use hisab::DVec3;

use crate::collider::{ColliderHandle, ColliderShape};
use crate::query::RayHit;

use super::body_ah;
use super::state::PhysicsState3d;
use super::types::EPSILON;

impl PhysicsState3d {
    // -----------------------------------------------------------------------
    // Raycast
    // -----------------------------------------------------------------------

    /// Cast a ray against every collider. Returns `None` for a zero or
    /// non-finite direction. Supports Ball, Box (rotated) and Capsule;
    /// other shapes are not hit.
    #[must_use]
    pub fn raycast(&self, origin: [f64; 3], direction: [f64; 3], max_dist: f64) -> Option<RayHit> {
        self.raycast_impl(origin, direction, max_dist, None)
    }

    /// Cast a ray with a collision layer filter. Only colliders whose
    /// `collision_layer` has at least one bit in common with `layer_mask`
    /// are considered.
    #[must_use]
    pub fn raycast_filtered(
        &self,
        origin: [f64; 3],
        direction: [f64; 3],
        max_dist: f64,
        layer_mask: u32,
    ) -> Option<RayHit> {
        self.raycast_impl(origin, direction, max_dist, Some(layer_mask))
    }

    #[must_use]
    fn raycast_impl(
        &self,
        origin: [f64; 3],
        direction: [f64; 3],
        max_dist: f64,
        layer_mask: Option<u32>,
    ) -> Option<RayHit> {
        let origin = DVec3::from_array(origin);
        if !origin.is_finite() {
            return None;
        }
        let dir = DVec3::from_array(direction).try_normalize()?;

        let mut best: Option<(f64, ColliderHandle, DVec3, DVec3)> = None;

        for collider in self.colliders.values() {
            if layer_mask.is_some_and(|mask| (collider.collision_layer & mask) == 0) {
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
                    // Box-local space: rotation preserves distances along the ray.
                    let inv = rb.rotation.inverse();
                    let he = DVec3::from_array(*half_extents);
                    ray_aabb_3d(inv * (origin - pos), inv * dir, -he, he)
                        .map(|(t, n)| (t, rb.rotation * n))
                }
                ColliderShape::Capsule {
                    half_height,
                    radius,
                } => {
                    let axis = rb.rotation * DVec3::new(0.0, *half_height, 0.0);
                    ray_capsule(origin, dir, pos - axis, pos + axis, *radius)
                }
                // ConvexHull, TriMesh, Segment, Heightfield: not supported.
                _ => None,
            };

            if let Some((t, normal)) = hit
                && t >= 0.0
                && t <= max_dist
                && best.is_none_or(|b| t < b.0)
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

#[must_use]
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

#[must_use]
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

/// Ray vs capsule (segment `pa`–`pb`, radius `r`); `dir` must be unit length.
///
/// Candidates are the entry/exit of the infinite cylinder and of both end
/// spheres; the nearest one that lies on the capsule surface wins.
#[must_use]
pub(super) fn ray_capsule(
    origin: DVec3,
    dir: DVec3,
    pa: DVec3,
    pb: DVec3,
    radius: f64,
) -> Option<(f64, DVec3)> {
    let mut candidates = [f64::NAN; 6];
    // Infinite cylinder around the axis.
    let ba = pb - pa;
    let baba = ba.dot(ba);
    if baba > EPSILON * EPSILON {
        let oa = origin - pa;
        let bard = ba.dot(dir);
        let baoa = ba.dot(oa);
        let a = baba - bard * bard;
        let b = baba * dir.dot(oa) - baoa * bard;
        let c = baba * oa.dot(oa) - baoa * baoa - radius * radius * baba;
        let h = b * b - a * c;
        if a > EPSILON && h >= 0.0 {
            let sq = h.sqrt();
            candidates[0] = (-b - sq) / a;
            candidates[1] = (-b + sq) / a;
        }
    }
    // End spheres.
    for (k, centre) in [pa, pb].into_iter().enumerate() {
        let oc = origin - centre;
        let half_b = oc.dot(dir);
        let h = half_b * half_b - (oc.dot(oc) - radius * radius);
        if h >= 0.0 {
            let sq = h.sqrt();
            candidates[2 + 2 * k] = -half_b - sq;
            candidates[3 + 2 * k] = -half_b + sq;
        }
    }

    let tol = 1e-9 * radius.max(1.0);
    let mut best: Option<(f64, DVec3)> = None;
    for t in candidates {
        if t.is_nan() || t < 0.0 || best.is_some_and(|b| t >= b.0) {
            continue;
        }
        let p = origin + dir * t;
        let axis_pt = super::narrowphase::closest_point_on_segment_3d(pa, pb, p);
        let offset = p - axis_pt;
        if (offset.length() - radius).abs() <= tol {
            best = Some((t, offset.normalize_or(DVec3::Y)));
        }
    }
    best
}
