//! Raycast and overlap query functions for the 2D physics backend.

use crate::collider::{ColliderHandle, ColliderShape};
use crate::query::RayHit;

use super::body_ah;
use super::narrowphase::{capsule_endpoints, shapes_touch, world_pos, world_to_local};
use super::state::PhysicsState2d;
use super::types::{Aabb2d, EPSILON};

impl PhysicsState2d {
    // -----------------------------------------------------------------------
    // Raycast
    // -----------------------------------------------------------------------

    #[must_use]
    pub fn raycast(&self, origin: [f64; 3], direction: [f64; 3], max_dist: f64) -> Option<RayHit> {
        self.raycast_filtered(origin, direction, max_dist, u32::MAX)
    }

    /// Cast a ray with a collision layer filter. Only colliders whose
    /// `collision_layer` has at least one bit in common with `layer_mask`
    /// are considered. A ray starting inside a shape reports where it exits.
    #[must_use]
    pub fn raycast_filtered(
        &self,
        origin: [f64; 3],
        direction: [f64; 3],
        max_dist: f64,
        layer_mask: u32,
    ) -> Option<RayHit> {
        let o = [origin[0], origin[1]];
        let dir_len = direction[0].hypot(direction[1]);
        if dir_len.is_nan() || dir_len < EPSILON || !dir_len.is_finite() {
            return None;
        }
        let dir = [direction[0] / dir_len, direction[1] / dir_len];

        let mut best: Option<(f64, ColliderHandle, [f64; 2])> = None;
        for collider in self.colliders.values() {
            if (collider.collision_layer & layer_mask) == 0 {
                continue;
            }
            let Some(rb) = self.bodies.get(body_ah(collider.body)) else {
                continue;
            };
            let pos = world_pos(rb.position, rb.rotation, collider.offset);
            let hit = match &collider.shape {
                // Hot path: no interval bookkeeping for balls.
                ColliderShape::Ball { radius } => ray_circle_first(o, dir, pos, *radius),
                shape => ray_shape(shape, pos, rb.rotation, o, dir).and_then(RayInterval::first),
            };
            if let Some((t, normal)) = hit
                && t <= max_dist
                && best.is_none_or(|b| t < b.0)
            {
                best = Some((t, collider.handle, normal));
            }
        }

        best.map(|(distance, collider, normal)| RayHit {
            collider,
            point: [o[0] + dir[0] * distance, o[1] + dir[1] * distance, 0.0],
            normal: [normal[0], normal[1], 0.0],
            distance,
        })
    }

    // -----------------------------------------------------------------------
    // Overlap queries (brute-force)
    // -----------------------------------------------------------------------

    /// Find all colliders overlapping a sphere (circle in 2D) at the given
    /// position. Uses the same contact tests as the simulation; Heightfield
    /// and TriMesh fall back to their bounding boxes.
    #[must_use]
    pub fn overlap_sphere(&self, center: [f64; 3], radius: f64) -> Vec<ColliderHandle> {
        let c = [center[0], center[1]];
        let probe = ColliderShape::Ball { radius };
        let probe_aabb = Aabb2d {
            min: [c[0] - radius, c[1] - radius],
            max: [c[0] + radius, c[1] + radius],
        };
        self.overlap_shape(&probe, c, &probe_aabb)
    }

    /// Find all colliders overlapping an axis-aligned box.
    #[must_use]
    pub fn overlap_aabb(&self, min: [f64; 3], max: [f64; 3]) -> Vec<ColliderHandle> {
        let probe_aabb = Aabb2d {
            min: [min[0], min[1]],
            max: [max[0], max[1]],
        };
        let probe = ColliderShape::Box {
            half_extents: [
                (max[0] - min[0]).abs() * 0.5,
                (max[1] - min[1]).abs() * 0.5,
                0.0,
            ],
        };
        let c = [(min[0] + max[0]) * 0.5, (min[1] + max[1]) * 0.5];
        self.overlap_shape(&probe, c, &probe_aabb)
    }

    fn overlap_shape(
        &self,
        probe: &ColliderShape,
        probe_pos: [f64; 2],
        probe_aabb: &Aabb2d,
    ) -> Vec<ColliderHandle> {
        let mut results = Vec::new();
        for collider in self.colliders.values() {
            let Some(rb) = self.bodies.get(body_ah(collider.body)) else {
                continue;
            };
            if !probe_aabb.overlaps(&collider.world_aabb(rb.position, rb.rotation)) {
                continue;
            }
            let precise = match &collider.shape {
                ColliderShape::Heightfield { .. } | ColliderShape::TriMesh { .. } => true,
                shape => {
                    let pos = world_pos(rb.position, rb.rotation, collider.offset);
                    shapes_touch(probe, probe_pos, 0.0, shape, pos, rb.rotation)
                }
            };
            if precise {
                results.push(collider.handle);
            }
        }
        results
    }
}

// ---------------------------------------------------------------------------
// Ray intersection helpers
// ---------------------------------------------------------------------------

/// Where a ray enters and leaves a convex shape (`t` along a unit direction),
/// with the outward surface normal at each.
#[derive(Debug, Clone, Copy)]
pub(super) struct RayInterval {
    t_in: f64,
    n_in: [f64; 2],
    t_out: f64,
    n_out: [f64; 2],
}

impl RayInterval {
    /// First hit at `t ≥ 0`: the entry point, or the exit point for a ray
    /// that starts inside the shape.
    #[must_use]
    pub(super) fn first(self) -> Option<(f64, [f64; 2])> {
        if self.t_in >= 0.0 {
            Some((self.t_in, self.n_in))
        } else if self.t_out >= 0.0 {
            Some((self.t_out, self.n_out))
        } else {
            None
        }
    }

    /// Union of two overlapping intervals of a convex shape.
    fn union(self, other: Self) -> Self {
        let (t_in, n_in) = if other.t_in < self.t_in {
            (other.t_in, other.n_in)
        } else {
            (self.t_in, self.n_in)
        };
        let (t_out, n_out) = if other.t_out > self.t_out {
            (other.t_out, other.n_out)
        } else {
            (self.t_out, self.n_out)
        };
        Self {
            t_in,
            n_in,
            t_out,
            n_out,
        }
    }
}

/// Ray vs a collider shape at `pos`/`rot`. `dir` must be unit length.
fn ray_shape(
    shape: &ColliderShape,
    pos: [f64; 2],
    rot: f64,
    origin: [f64; 2],
    dir: [f64; 2],
) -> Option<RayInterval> {
    match shape {
        ColliderShape::Ball { radius } => ray_circle_interval(origin, dir, pos, *radius),
        ColliderShape::Box { half_extents } => {
            // Work in the box's frame, then rotate normals back.
            let lo = world_to_local(origin, pos, rot);
            let (sin, cos) = rot.sin_cos();
            let ld = [cos * dir[0] + sin * dir[1], -sin * dir[0] + cos * dir[1]];
            let he = [half_extents[0].abs(), half_extents[1].abs()];
            let back = |n: [f64; 2]| [cos * n[0] - sin * n[1], sin * n[0] + cos * n[1]];
            ray_aabb_interval(lo, ld, [-he[0], -he[1]], he).map(|r| RayInterval {
                n_in: back(r.n_in),
                n_out: back(r.n_out),
                ..r
            })
        }
        ColliderShape::Capsule {
            half_height,
            radius,
        } => ray_capsule_interval(origin, dir, pos, rot, *half_height, *radius),
        ColliderShape::Segment { a, b } => {
            let wa = world_pos(pos, rot, [a[0], a[1]]);
            let wb = world_pos(pos, rot, [b[0], b[1]]);
            ray_segment(origin, dir, wa, wb).map(|(t, n)| RayInterval {
                t_in: t,
                n_in: n,
                t_out: t,
                n_out: n,
            })
        }
        ColliderShape::ConvexHull { points } if points.len() >= 3 => {
            let mut verts: Vec<[f64; 2]> = points
                .iter()
                .map(|p| world_pos(pos, rot, [p[0], p[1]]))
                .collect();
            let area2: f64 = (0..verts.len())
                .map(|i| {
                    let p = verts[i];
                    let q = verts[(i + 1) % verts.len()];
                    p[0] * q[1] - q[0] * p[1]
                })
                .sum();
            if area2 < 0.0 {
                verts.reverse();
            }
            ray_polygon_interval(origin, dir, &verts)
        }
        _ => None,
    }
}

/// First hit of a ray on a circle (exit point if the origin is inside).
#[inline]
fn ray_circle_first(
    origin: [f64; 2],
    dir: [f64; 2],
    center: [f64; 2],
    radius: f64,
) -> Option<(f64, [f64; 2])> {
    let oc = [origin[0] - center[0], origin[1] - center[1]];
    let half_b = oc[0] * dir[0] + oc[1] * dir[1];
    let c = oc[0] * oc[0] + oc[1] * oc[1] - radius * radius;
    let discriminant = half_b * half_b - c;
    if discriminant.is_nan() || discriminant < 0.0 {
        return None;
    }
    let sqrt_d = discriminant.sqrt();
    let t = if -half_b - sqrt_d >= 0.0 {
        -half_b - sqrt_d
    } else if -half_b + sqrt_d >= 0.0 {
        -half_b + sqrt_d
    } else {
        return None;
    };
    let p = [oc[0] + dir[0] * t, oc[1] + dir[1] * t];
    let len = p[0].hypot(p[1]);
    let n = if len > EPSILON {
        [p[0] / len, p[1] / len]
    } else {
        [-dir[0], -dir[1]]
    };
    Some((t, n))
}

fn ray_circle_interval(
    origin: [f64; 2],
    dir: [f64; 2],
    center: [f64; 2],
    radius: f64,
) -> Option<RayInterval> {
    let oc = [origin[0] - center[0], origin[1] - center[1]];
    let half_b = oc[0] * dir[0] + oc[1] * dir[1];
    let c = oc[0] * oc[0] + oc[1] * oc[1] - radius * radius;
    let discriminant = half_b * half_b - c;
    if discriminant.is_nan() || discriminant < 0.0 {
        return None;
    }
    let sqrt_d = discriminant.sqrt();
    let normal_at = |t: f64| {
        let p = [
            origin[0] + dir[0] * t - center[0],
            origin[1] + dir[1] * t - center[1],
        ];
        let len = p[0].hypot(p[1]);
        if len > EPSILON {
            [p[0] / len, p[1] / len]
        } else {
            [-dir[0], -dir[1]]
        }
    };
    let t_in = -half_b - sqrt_d;
    let t_out = -half_b + sqrt_d;
    Some(RayInterval {
        t_in,
        n_in: normal_at(t_in),
        t_out,
        n_out: normal_at(t_out),
    })
}

fn ray_aabb_interval(
    origin: [f64; 2],
    dir: [f64; 2],
    min: [f64; 2],
    max: [f64; 2],
) -> Option<RayInterval> {
    let mut t_in = f64::NEG_INFINITY;
    let mut t_out = f64::INFINITY;
    let mut n_in = [-dir[0], -dir[1]];
    let mut n_out = dir;
    for i in 0..2 {
        if dir[i].abs() < EPSILON {
            if origin[i] < min[i] || origin[i] > max[i] {
                return None;
            }
            continue;
        }
        let inv_d = 1.0 / dir[i];
        let mut t1 = (min[i] - origin[i]) * inv_d;
        let mut t2 = (max[i] - origin[i]) * inv_d;
        // Entering through the min face has normal −axis, exiting through max +axis.
        let mut n1 = [0.0; 2];
        n1[i] = -1.0;
        let mut n2 = [0.0; 2];
        n2[i] = 1.0;
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
            std::mem::swap(&mut n1, &mut n2);
        }
        if t1 > t_in {
            t_in = t1;
            n_in = n1;
        }
        if t2 < t_out {
            t_out = t2;
            n_out = n2;
        }
        if t_in > t_out {
            return None;
        }
    }
    Some(RayInterval {
        t_in,
        n_in,
        t_out,
        n_out,
    })
}

/// Ray vs convex polygon (counter-clockwise), Cyrus–Beck clipping.
fn ray_polygon_interval(
    origin: [f64; 2],
    dir: [f64; 2],
    verts: &[[f64; 2]],
) -> Option<RayInterval> {
    let mut t_in = f64::NEG_INFINITY;
    let mut t_out = f64::INFINITY;
    let mut n_in = [-dir[0], -dir[1]];
    let mut n_out = dir;
    let n = verts.len();
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        let e = [b[0] - a[0], b[1] - a[1]];
        let len = e[0].hypot(e[1]);
        if len < EPSILON {
            continue;
        }
        let normal = [e[1] / len, -e[0] / len];
        // Signed distance of the origin outside this edge, and its rate of change.
        let dist = (origin[0] - a[0]) * normal[0] + (origin[1] - a[1]) * normal[1];
        let rate = dir[0] * normal[0] + dir[1] * normal[1];
        if rate.abs() < EPSILON {
            if dist > 0.0 {
                return None;
            }
            continue;
        }
        let t = -dist / rate;
        if rate < 0.0 {
            if t > t_in {
                t_in = t;
                n_in = normal;
            }
        } else if t < t_out {
            t_out = t;
            n_out = normal;
        }
        if t_in > t_out {
            return None;
        }
    }
    Some(RayInterval {
        t_in,
        n_in,
        t_out,
        n_out,
    })
}

/// Ray vs line segment: (t, normal facing the ray).
fn ray_segment(
    origin: [f64; 2],
    dir: [f64; 2],
    a: [f64; 2],
    b: [f64; 2],
) -> Option<(f64, [f64; 2])> {
    let e = [b[0] - a[0], b[1] - a[1]];
    let denom = dir[0] * e[1] - dir[1] * e[0];
    if denom.abs() < EPSILON {
        return None; // parallel
    }
    let ao = [a[0] - origin[0], a[1] - origin[1]];
    let t = (ao[0] * e[1] - ao[1] * e[0]) / denom;
    let u = (ao[0] * dir[1] - ao[1] * dir[0]) / denom;
    if !(0.0..=1.0).contains(&u) || t < 0.0 {
        return None;
    }
    let len = e[0].hypot(e[1]);
    let mut n = [e[1] / len, -e[0] / len];
    if n[0] * dir[0] + n[1] * dir[1] > 0.0 {
        n = [-n[0], -n[1]];
    }
    Some((t, n))
}

fn ray_capsule_interval(
    origin: [f64; 2],
    dir: [f64; 2],
    cap_pos: [f64; 2],
    cap_rot: f64,
    half_height: f64,
    radius: f64,
) -> Option<RayInterval> {
    let (ep_a, ep_b) = capsule_endpoints(cap_pos, cap_rot, half_height);
    // The capsule is the convex union of two end discs and the shaft rectangle.
    let mut parts = [
        ray_circle_interval(origin, dir, ep_a, radius),
        ray_circle_interval(origin, dir, ep_b, radius),
        None,
    ];
    let axis = [ep_b[0] - ep_a[0], ep_b[1] - ep_a[1]];
    let axis_len = axis[0].hypot(axis[1]);
    if axis_len > EPSILON {
        let ax = [axis[0] / axis_len, axis[1] / axis_len];
        let perp = [-ax[1], ax[0]];
        let rel = [origin[0] - cap_pos[0], origin[1] - cap_pos[1]];
        let lo = [
            rel[0] * ax[0] + rel[1] * ax[1],
            rel[0] * perp[0] + rel[1] * perp[1],
        ];
        let ld = [
            dir[0] * ax[0] + dir[1] * ax[1],
            dir[0] * perp[0] + dir[1] * perp[1],
        ];
        let back = |n: [f64; 2]| [n[0] * ax[0] + n[1] * perp[0], n[0] * ax[1] + n[1] * perp[1]];
        parts[2] =
            ray_aabb_interval(lo, ld, [-half_height, -radius], [half_height, radius]).map(|r| {
                RayInterval {
                    n_in: back(r.n_in),
                    n_out: back(r.n_out),
                    ..r
                }
            });
    }
    parts.into_iter().flatten().reduce(RayInterval::union)
}

#[cfg(test)]
pub(super) fn ray_circle(
    origin: [f64; 2],
    dir: [f64; 2],
    center: [f64; 2],
    radius: f64,
) -> Option<(f64, [f64; 2])> {
    let first = ray_circle_first(origin, dir, center, radius);
    debug_assert_eq!(
        first.map(|f| f.0),
        ray_circle_interval(origin, dir, center, radius)
            .and_then(RayInterval::first)
            .map(|f| f.0)
    );
    first
}

#[cfg(test)]
pub(super) fn ray_aabb_2d(
    origin: [f64; 2],
    dir: [f64; 2],
    min: [f64; 2],
    max: [f64; 2],
) -> Option<(f64, [f64; 2])> {
    ray_aabb_interval(origin, dir, min, max).and_then(RayInterval::first)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ray_from_inside_box_reports_exit_face() {
        let (t, n) = ray_aabb_2d([5.0, 0.0], [1.0, 0.0], [4.0, -1.0], [6.0, 1.0]).unwrap();
        assert!((t - 1.0).abs() < 1e-12);
        assert_eq!(n, [1.0, 0.0]);
    }

    #[test]
    fn ray_from_inside_capsule_exits_through_cap() {
        // Vertical capsule at origin, hh 1, r 0.5; ray up its axis from the centre.
        let r = ray_capsule_interval([0.0, 0.0], [0.0, 1.0], [0.0, 0.0], 0.0, 1.0, 0.5)
            .and_then(RayInterval::first)
            .unwrap();
        assert!((r.0 - 1.5).abs() < 1e-12, "t = {}", r.0);
        assert!((r.1[1] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn ray_hits_segment_and_hull() {
        let (t, n) = ray_segment([0.0, 5.0], [0.0, -1.0], [-1.0, 0.0], [1.0, 0.0]).unwrap();
        assert!((t - 5.0).abs() < 1e-12 && n == [0.0, 1.0]);
        let sq = [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
        let (t, n) = ray_polygon_interval([-5.0, 0.0], [1.0, 0.0], &sq)
            .and_then(RayInterval::first)
            .unwrap();
        assert!((t - 4.0).abs() < 1e-12 && n == [-1.0, 0.0]);
    }
}
