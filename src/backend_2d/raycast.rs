//! Raycast and overlap query functions for the 2D physics backend.

use crate::collider::{ColliderHandle, ColliderShape};
use crate::query::RayHit;

use super::body_ah;
use super::narrowphase::{capsule_endpoints, closest_point_on_segment, world_pos};
use super::state::PhysicsState2d;
use super::types::{Aabb2d, EPSILON};

impl PhysicsState2d {
    // -----------------------------------------------------------------------
    // Raycast
    // -----------------------------------------------------------------------

    pub fn raycast(&self, origin: [f64; 3], direction: [f64; 3], max_dist: f64) -> Option<RayHit> {
        let origin_2d = [origin[0], origin[1]];
        let direction_2d = [direction[0], direction[1]];
        let dir_len =
            (direction_2d[0] * direction_2d[0] + direction_2d[1] * direction_2d[1]).sqrt();
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
        let dir_len =
            (direction_2d[0] * direction_2d[0] + direction_2d[1] * direction_2d[1]).sqrt();
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
// Ray intersection helpers
// ---------------------------------------------------------------------------

pub(super) fn ray_circle(
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

pub(super) fn ray_aabb_2d(
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
