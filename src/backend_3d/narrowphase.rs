//! Narrowphase contact generation functions for 3D shapes.

use hisab::{DQuat, DVec3};

use super::types::{EPSILON, EPSILON_SQ};
use crate::collider::ColliderShape;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

pub(super) fn is_identity_quat(q: DQuat) -> bool {
    (q.x.abs() < EPSILON)
        && (q.y.abs() < EPSILON)
        && (q.z.abs() < EPSILON)
        && ((q.w.abs() - 1.0).abs() < EPSILON)
}

pub(super) fn closest_point_on_segment_3d(a: DVec3, b: DVec3, p: DVec3) -> DVec3 {
    let ab = b - a;
    let len_sq = ab.dot(ab);
    if len_sq < EPSILON_SQ {
        return a;
    }
    let t = ((p - a).dot(ab) / len_sq).clamp(0.0, 1.0);
    a + ab * t
}

/// Capsule endpoints in world space given position, rotation, and half-height.
pub(super) fn capsule_endpoints_3d(pos: DVec3, rot: DQuat, half_height: f64) -> (DVec3, DVec3) {
    let axis = rot * DVec3::new(0.0, half_height, 0.0);
    (pos - axis, pos + axis)
}

/// Closest points between two 3D line segments. Returns (point_on_ab, point_on_cd).
pub(super) fn closest_points_segments_3d(a: DVec3, b: DVec3, c: DVec3, d: DVec3) -> (DVec3, DVec3) {
    let r = b - a; // direction of segment 1
    let s = d - c; // direction of segment 2
    let w = a - c;

    let rr = r.dot(r); // |r|^2
    let ss = s.dot(s); // |s|^2
    let rs = r.dot(s);
    let rw = r.dot(w);
    let sw = s.dot(w);

    let denom = rr * ss - rs * rs;

    let (sc, tc);

    if denom.abs() < EPSILON_SQ {
        // Nearly parallel
        sc = 0.0;
        tc = if ss.abs() < EPSILON_SQ {
            0.0
        } else {
            (sw / ss).clamp(0.0, 1.0)
        };
    } else {
        let sn = (rs * sw - ss * rw) / denom;
        let tn = (rr * sw - rs * rw) / denom;

        if sn < 0.0 {
            let t = if ss.abs() < EPSILON_SQ {
                0.0
            } else {
                (sw / ss).clamp(0.0, 1.0)
            };
            sc = 0.0;
            tc = t;
        } else if sn > 1.0 {
            let t = if ss.abs() < EPSILON_SQ {
                0.0
            } else {
                ((sw + rs) / ss).clamp(0.0, 1.0)
            };
            sc = 1.0;
            tc = t;
        } else if tn < 0.0 {
            tc = 0.0;
            sc = if rr.abs() < EPSILON_SQ {
                0.0
            } else {
                (-rw / rr).clamp(0.0, 1.0)
            };
        } else if tn > 1.0 {
            tc = 1.0;
            sc = if rr.abs() < EPSILON_SQ {
                0.0
            } else {
                ((rs - rw) / rr).clamp(0.0, 1.0)
            };
        } else {
            sc = sn;
            tc = tn;
        }
    }

    (a + r * sc, c + s * tc)
}

// ---------------------------------------------------------------------------
// Contact generation dispatch
// ---------------------------------------------------------------------------

pub(super) fn generate_contact_3d(
    shape_a: &ColliderShape,
    pos_a: DVec3,
    rot_a: DQuat,
    shape_b: &ColliderShape,
    pos_b: DVec3,
    rot_b: DQuat,
) -> Option<(DVec3, f64, DVec3)> {
    match (shape_a, shape_b) {
        // Ball vs Ball
        (ColliderShape::Ball { radius: ra }, ColliderShape::Ball { radius: rb }) => {
            sphere_sphere(pos_a, *ra, pos_b, *rb)
        }
        // Ball vs Box
        (ColliderShape::Ball { radius }, ColliderShape::Box { half_extents }) => sphere_obb(
            pos_a,
            *radius,
            pos_b,
            rot_b,
            DVec3::from_array(*half_extents),
        ),
        (ColliderShape::Box { half_extents }, ColliderShape::Ball { radius }) => sphere_obb(
            pos_b,
            *radius,
            pos_a,
            rot_a,
            DVec3::from_array(*half_extents),
        )
        .map(|(n, d, p)| (-n, d, p)),
        // Box vs Box — OBB when rotated, AABB fast path otherwise
        (ColliderShape::Box { half_extents: he_a }, ColliderShape::Box { half_extents: he_b }) => {
            let hea = DVec3::from_array(*he_a);
            let heb = DVec3::from_array(*he_b);
            if is_identity_quat(rot_a) && is_identity_quat(rot_b) {
                aabb_aabb_3d(pos_a, hea, pos_b, heb)
            } else {
                obb_obb_3d(pos_a, rot_a, hea, pos_b, rot_b, heb)
            }
        }
        // Capsule vs Sphere
        (
            ColliderShape::Capsule {
                half_height,
                radius: cr,
            },
            ColliderShape::Ball { radius: br },
        ) => capsule_sphere_3d(pos_a, rot_a, *half_height, *cr, pos_b, *br),
        (
            ColliderShape::Ball { radius: br },
            ColliderShape::Capsule {
                half_height,
                radius: cr,
            },
        ) => capsule_sphere_3d(pos_b, rot_b, *half_height, *cr, pos_a, *br)
            .map(|(n, d, p)| (-n, d, p)),
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
        ) => capsule_capsule_3d(pos_a, rot_a, *hh_a, *cr_a, pos_b, rot_b, *hh_b, *cr_b),
        // Capsule vs Box
        (
            ColliderShape::Capsule {
                half_height,
                radius: cr,
            },
            ColliderShape::Box { half_extents },
        ) => capsule_box_3d(
            pos_a,
            rot_a,
            *half_height,
            *cr,
            pos_b,
            rot_b,
            DVec3::from_array(*half_extents),
        ),
        (
            ColliderShape::Box { half_extents },
            ColliderShape::Capsule {
                half_height,
                radius: cr,
            },
        ) => capsule_box_3d(
            pos_b,
            rot_b,
            *half_height,
            *cr,
            pos_a,
            rot_a,
            DVec3::from_array(*half_extents),
        )
        .map(|(n, d, p)| (-n, d, p)),
        // Segment vs Ball
        (ColliderShape::Segment { a, b }, ColliderShape::Ball { radius }) => segment_sphere_3d(
            pos_a,
            rot_a,
            DVec3::from_array(*a),
            DVec3::from_array(*b),
            pos_b,
            *radius,
        ),
        (ColliderShape::Ball { radius }, ColliderShape::Segment { a, b }) => segment_sphere_3d(
            pos_b,
            rot_b,
            DVec3::from_array(*a),
            DVec3::from_array(*b),
            pos_a,
            *radius,
        )
        .map(|(n, d, p)| (-n, d, p)),
        // Segment vs Box
        (ColliderShape::Segment { a, b }, ColliderShape::Box { half_extents }) => segment_box_3d(
            pos_a,
            rot_a,
            DVec3::from_array(*a),
            DVec3::from_array(*b),
            pos_b,
            rot_b,
            DVec3::from_array(*half_extents),
        ),
        (ColliderShape::Box { half_extents }, ColliderShape::Segment { a, b }) => segment_box_3d(
            pos_b,
            rot_b,
            DVec3::from_array(*a),
            DVec3::from_array(*b),
            pos_a,
            rot_a,
            DVec3::from_array(*half_extents),
        )
        .map(|(n, d, p)| (-n, d, p)),
        // ConvexHull vs Ball
        (ColliderShape::ConvexHull { points }, ColliderShape::Ball { radius }) => {
            convex_hull_sphere_3d(points, pos_a, rot_a, pos_b, *radius)
        }
        (ColliderShape::Ball { radius }, ColliderShape::ConvexHull { points }) => {
            convex_hull_sphere_3d(points, pos_b, rot_b, pos_a, *radius).map(|(n, d, p)| (-n, d, p))
        }
        // ConvexHull vs Box — GJK+EPA fallback
        (ColliderShape::ConvexHull { points: _ }, ColliderShape::Box { half_extents: _ }) => {
            gjk_epa_shapes(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b)
        }
        (ColliderShape::Box { half_extents: _ }, ColliderShape::ConvexHull { points: _ }) => {
            gjk_epa_shapes(shape_b, pos_b, rot_b, shape_a, pos_a, rot_a).map(|(n, d, p)| (-n, d, p))
        }
        // ConvexHull vs ConvexHull — GJK+EPA
        (ColliderShape::ConvexHull { .. }, ColliderShape::ConvexHull { .. }) => {
            gjk_epa_shapes(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b)
        }
        // ConvexHull vs Capsule — GJK+EPA fallback
        (ColliderShape::ConvexHull { .. }, ColliderShape::Capsule { .. }) => {
            gjk_epa_shapes(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b)
        }
        (ColliderShape::Capsule { .. }, ColliderShape::ConvexHull { .. }) => {
            gjk_epa_shapes(shape_b, pos_b, rot_b, shape_a, pos_a, rot_a).map(|(n, d, p)| (-n, d, p))
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// GJK + EPA (f64, native implementation)
// ---------------------------------------------------------------------------

/// Support function: furthest point in a direction for a shape in world space.
#[inline]
fn shape_support(shape: &ColliderShape, pos: DVec3, rot: DQuat, dir: DVec3) -> DVec3 {
    let local_dir = rot.inverse() * dir;
    let local_pt = match shape {
        ColliderShape::Ball { radius } => {
            let len = local_dir.length();
            if len < EPSILON {
                DVec3::new(0.0, *radius, 0.0)
            } else {
                local_dir / len * *radius
            }
        }
        ColliderShape::Box { half_extents } => DVec3::new(
            if local_dir.x >= 0.0 {
                half_extents[0]
            } else {
                -half_extents[0]
            },
            if local_dir.y >= 0.0 {
                half_extents[1]
            } else {
                -half_extents[1]
            },
            if local_dir.z >= 0.0 {
                half_extents[2]
            } else {
                -half_extents[2]
            },
        ),
        ColliderShape::Capsule {
            half_height,
            radius,
        } => {
            let axis_pt = if local_dir.y >= 0.0 {
                DVec3::new(0.0, *half_height, 0.0)
            } else {
                DVec3::new(0.0, -*half_height, 0.0)
            };
            let len = local_dir.length();
            if len < EPSILON {
                axis_pt
            } else {
                axis_pt + local_dir / len * *radius
            }
        }
        ColliderShape::ConvexHull { points } => {
            let mut best = DVec3::ZERO;
            let mut best_dot = f64::NEG_INFINITY;
            for p in points {
                let v = DVec3::new(p[0], p[1], p[2]);
                let d = v.dot(local_dir);
                if d > best_dot {
                    best_dot = d;
                    best = v;
                }
            }
            best
        }
        // For unsupported shapes, return position (degenerate)
        _ => DVec3::ZERO,
    };
    pos + rot * local_pt
}

/// Minkowski difference support: support_A(dir) - support_B(-dir).
#[inline]
fn minkowski_support(
    shape_a: &ColliderShape,
    pos_a: DVec3,
    rot_a: DQuat,
    shape_b: &ColliderShape,
    pos_b: DVec3,
    rot_b: DQuat,
    dir: DVec3,
) -> DVec3 {
    shape_support(shape_a, pos_a, rot_a, dir) - shape_support(shape_b, pos_b, rot_b, -dir)
}

/// GJK+EPA collision detection for arbitrary convex shapes.
/// Returns (normal_A_to_B, depth, contact_point).
fn gjk_epa_shapes(
    shape_a: &ColliderShape,
    pos_a: DVec3,
    rot_a: DQuat,
    shape_b: &ColliderShape,
    pos_b: DVec3,
    rot_b: DQuat,
) -> Option<(DVec3, f64, DVec3)> {
    const GJK_MAX_ITER: usize = 64;
    const EPA_MAX_ITER: usize = 64;

    let initial_dir = pos_b - pos_a;
    let dir = if initial_dir.length_squared() < EPSILON_SQ {
        DVec3::X
    } else {
        initial_dir
    };

    // GJK phase: build simplex
    let mut simplex: Vec<DVec3> = Vec::with_capacity(4);
    simplex.push(minkowski_support(
        shape_a, pos_a, rot_a, shape_b, pos_b, rot_b, dir,
    ));

    let mut search_dir = -simplex[0];

    for _ in 0..GJK_MAX_ITER {
        if search_dir.length_squared() < EPSILON_SQ {
            break;
        }
        let new_pt = minkowski_support(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b, search_dir);

        if new_pt.dot(search_dir) < 0.0 {
            return None; // No intersection
        }

        simplex.push(new_pt);

        if gjk_do_simplex(&mut simplex, &mut search_dir) {
            // Intersection found — run EPA
            return epa_penetration(
                shape_a,
                pos_a,
                rot_a,
                shape_b,
                pos_b,
                rot_b,
                &simplex,
                EPA_MAX_ITER,
            );
        }
    }

    None
}

/// GJK simplex evolution. Returns true if origin is enclosed.
fn gjk_do_simplex(simplex: &mut Vec<DVec3>, dir: &mut DVec3) -> bool {
    match simplex.len() {
        2 => {
            // Line case
            let a = simplex[1];
            let b = simplex[0];
            let ab = b - a;
            let ao = -a;
            if ab.dot(ao) > 0.0 {
                *dir = ab.cross(ao).cross(ab);
            } else {
                simplex.clear();
                simplex.push(a);
                *dir = ao;
            }
            false
        }
        3 => {
            // Triangle case
            let a = simplex[2];
            let b = simplex[1];
            let c = simplex[0];
            let ab = b - a;
            let ac = c - a;
            let ao = -a;
            let abc = ab.cross(ac);

            if abc.cross(ac).dot(ao) > 0.0 {
                if ac.dot(ao) > 0.0 {
                    simplex.clear();
                    simplex.push(c);
                    simplex.push(a);
                    *dir = ac.cross(ao).cross(ac);
                } else {
                    simplex.clear();
                    simplex.push(b);
                    simplex.push(a);
                    return gjk_do_simplex(simplex, dir);
                }
            } else if ab.cross(abc).dot(ao) > 0.0 {
                simplex.clear();
                simplex.push(b);
                simplex.push(a);
                return gjk_do_simplex(simplex, dir);
            } else if abc.dot(ao) > 0.0 {
                *dir = abc;
            } else {
                // Below triangle
                simplex.swap(0, 1);
                *dir = -abc;
            }
            false
        }
        4 => {
            // Tetrahedron case
            let a = simplex[3];
            let b = simplex[2];
            let c = simplex[1];
            let d = simplex[0];
            let ao = -a;

            let ab = b - a;
            let ac = c - a;
            let ad = d - a;

            let abc = ab.cross(ac);
            let acd = ac.cross(ad);
            let adb = ad.cross(ab);

            if abc.dot(ao) > 0.0 {
                simplex.clear();
                simplex.push(c);
                simplex.push(b);
                simplex.push(a);
                return gjk_do_simplex(simplex, dir);
            }
            if acd.dot(ao) > 0.0 {
                simplex.clear();
                simplex.push(d);
                simplex.push(c);
                simplex.push(a);
                return gjk_do_simplex(simplex, dir);
            }
            if adb.dot(ao) > 0.0 {
                simplex.clear();
                simplex.push(b);
                simplex.push(d);
                simplex.push(a);
                return gjk_do_simplex(simplex, dir);
            }
            true // Origin is inside tetrahedron
        }
        _ => false,
    }
}

/// EPA: expand the simplex polytope to find the penetration normal and depth.
#[allow(clippy::too_many_arguments)]
fn epa_penetration(
    shape_a: &ColliderShape,
    pos_a: DVec3,
    rot_a: DQuat,
    shape_b: &ColliderShape,
    pos_b: DVec3,
    rot_b: DQuat,
    simplex: &[DVec3],
    max_iterations: usize,
) -> Option<(DVec3, f64, DVec3)> {
    if simplex.len() < 4 {
        return None;
    }

    // Build initial polytope from tetrahedron
    let mut vertices: Vec<DVec3> = simplex.to_vec();
    // Faces as vertex index triples, wound so normal points outward
    let mut faces: Vec<[usize; 3]> = vec![[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]];

    for _ in 0..max_iterations {
        // Find the closest face to the origin
        let mut min_dist = f64::INFINITY;
        let mut min_normal = DVec3::ZERO;

        for face in faces.iter() {
            let a = vertices[face[0]];
            let b = vertices[face[1]];
            let c = vertices[face[2]];
            let normal = (b - a).cross(c - a);
            let len = normal.length();
            if len < EPSILON {
                continue;
            }
            let normal = normal / len;
            let dist = normal.dot(a);

            // Ensure normal points away from origin
            let (normal, dist) = if dist < 0.0 {
                (-normal, -dist)
            } else {
                (normal, dist)
            };

            if dist < min_dist {
                min_dist = dist;
                min_normal = normal;
            }
        }

        if min_normal.length_squared() < EPSILON_SQ {
            return None;
        }

        // Get new support point along the closest face normal
        let new_pt = minkowski_support(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b, min_normal);
        let new_dist = new_pt.dot(min_normal);

        if new_dist - min_dist < 1e-6 {
            // Converged — compute contact point
            let sa = shape_support(shape_a, pos_a, rot_a, min_normal);
            let sb = shape_support(shape_b, pos_b, rot_b, -min_normal);
            let contact_point = (sa + sb) * 0.5;
            return Some((min_normal, min_dist, contact_point));
        }

        // Expand polytope: remove faces visible from new point, add new faces
        let new_idx = vertices.len();
        vertices.push(new_pt);

        let mut edges: Vec<[usize; 2]> = Vec::new();
        faces.retain(|face| {
            let a = vertices[face[0]];
            let b = vertices[face[1]];
            let c = vertices[face[2]];
            let normal = (b - a).cross(c - a);
            let len = normal.length();
            if len < EPSILON {
                return false;
            }
            let normal = normal / len;
            // If face is visible from new point, remove it and collect edges
            if normal.dot(new_pt - a) > 0.0 {
                // Add edges (in reverse order for proper winding)
                let edge_pairs = [[face[0], face[1]], [face[1], face[2]], [face[2], face[0]]];
                for edge in &edge_pairs {
                    // Check if reverse edge already exists (shared edge)
                    if let Some(pos) = edges
                        .iter()
                        .position(|e| e[0] == edge[1] && e[1] == edge[0])
                    {
                        edges.swap_remove(pos);
                    } else {
                        edges.push(*edge);
                    }
                }
                false
            } else {
                true
            }
        });

        // Create new faces from horizon edges to new point
        for edge in &edges {
            faces.push([edge[0], edge[1], new_idx]);
        }

        if faces.is_empty() {
            return None;
        }
    }

    None
}

// ---------------------------------------------------------------------------
// Shape-vs-shape contact functions
// ---------------------------------------------------------------------------

pub(super) fn sphere_sphere(
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
    let (normal, depth) = if dist < EPSILON {
        (DVec3::Y, sum_r)
    } else {
        (d / dist, sum_r - dist)
    };

    let point = pos_a + normal * ra;
    Some((normal, depth, point))
}

/// Sphere vs OBB (oriented bounding box): transform sphere into box-local space.
pub(super) fn sphere_obb(
    sphere_pos: DVec3,
    radius: f64,
    box_pos: DVec3,
    box_rot: DQuat,
    half_extents: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    // Transform sphere center into box-local space
    let inv_rot = box_rot.inverse();
    let local_sphere = inv_rot * (sphere_pos - box_pos);

    let closest = local_sphere.clamp(-half_extents, half_extents);
    let diff = local_sphere - closest;
    let dist_sq = diff.dot(diff);

    if dist_sq >= radius * radius {
        return None;
    }

    let dist = dist_sq.sqrt();
    let (local_normal, depth) = if dist < EPSILON {
        // Sphere center inside box — push out along minimum-penetration face
        let face_dists = DVec3::new(
            half_extents.x - local_sphere.x.abs(),
            half_extents.y - local_sphere.y.abs(),
            half_extents.z - local_sphere.z.abs(),
        );
        let min_axis = if face_dists.x <= face_dists.y && face_dists.x <= face_dists.z {
            0
        } else if face_dists.y <= face_dists.z {
            1
        } else {
            2
        };
        let mut n = DVec3::ZERO;
        n[min_axis] = if local_sphere[min_axis] >= 0.0 {
            1.0
        } else {
            -1.0
        };
        (n, face_dists[min_axis] + radius)
    } else {
        (diff / dist, radius - dist)
    };

    // Transform normal and contact point back to world space
    let normal = box_rot * local_normal;
    let point = box_rot * closest + box_pos;
    Some((normal, depth, point))
}

pub(super) fn aabb_aabb_3d(
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

pub(super) fn capsule_sphere_3d(
    cap_pos: DVec3,
    cap_rot: DQuat,
    half_height: f64,
    cap_radius: f64,
    sphere_pos: DVec3,
    sphere_radius: f64,
) -> Option<(DVec3, f64, DVec3)> {
    let (ep_a, ep_b) = capsule_endpoints_3d(cap_pos, cap_rot, half_height);
    let closest = closest_point_on_segment_3d(ep_a, ep_b, sphere_pos);
    sphere_sphere(closest, cap_radius, sphere_pos, sphere_radius)
}

/// Capsule vs Capsule: closest points between two segments, then sphere-sphere test.
#[allow(clippy::too_many_arguments)]
pub(super) fn capsule_capsule_3d(
    pos_a: DVec3,
    rot_a: DQuat,
    hh_a: f64,
    cr_a: f64,
    pos_b: DVec3,
    rot_b: DQuat,
    hh_b: f64,
    cr_b: f64,
) -> Option<(DVec3, f64, DVec3)> {
    let (a1, a2) = capsule_endpoints_3d(pos_a, rot_a, hh_a);
    let (b1, b2) = capsule_endpoints_3d(pos_b, rot_b, hh_b);
    let (pa, pb) = closest_points_segments_3d(a1, a2, b1, b2);
    sphere_sphere(pa, cr_a, pb, cr_b)
}

/// Capsule vs Box: find closest point on capsule segment to box, then sphere-OBB test.
pub(super) fn capsule_box_3d(
    cap_pos: DVec3,
    cap_rot: DQuat,
    half_height: f64,
    cap_radius: f64,
    box_pos: DVec3,
    box_rot: DQuat,
    half_extents: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    let (ep_a, ep_b) = capsule_endpoints_3d(cap_pos, cap_rot, half_height);

    // Transform capsule endpoints into box-local space
    let inv_rot = box_rot.inverse();
    let local_a = inv_rot * (ep_a - box_pos);
    let local_b = inv_rot * (ep_b - box_pos);

    // Find the point on the capsule segment closest to the box in local space.
    // We find the closest point on the segment to the box by clamping.
    let ab = local_b - local_a;
    let len_sq = ab.dot(ab);

    // Sample along the segment and find the parameter t that minimizes distance to the box
    let best_t = if len_sq < EPSILON_SQ {
        0.0
    } else {
        // Analytical: project box center (origin in local space) onto segment, clamp
        let t_center = (-local_a).dot(ab) / len_sq;
        t_center.clamp(0.0, 1.0)
    };

    let closest_on_seg = local_a + ab * best_t;
    // The closest point on the capsule segment in world space
    let world_seg_pt = box_rot * closest_on_seg + box_pos;

    // Now do a sphere-OBB test with the sphere centered at the closest segment point
    sphere_obb(world_seg_pt, cap_radius, box_pos, box_rot, half_extents)
}

/// OBB vs OBB using SAT with 6 face normals (3 per box).
pub(super) fn obb_obb_3d(
    pos_a: DVec3,
    rot_a: DQuat,
    he_a: DVec3,
    pos_b: DVec3,
    rot_b: DQuat,
    he_b: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    // Get the 3 local axes for each box
    let axes_a = [rot_a * DVec3::X, rot_a * DVec3::Y, rot_a * DVec3::Z];
    let axes_b = [rot_b * DVec3::X, rot_b * DVec3::Y, rot_b * DVec3::Z];
    let he_a_arr = [he_a.x, he_a.y, he_a.z];
    let he_b_arr = [he_b.x, he_b.y, he_b.z];

    let d = pos_b - pos_a;

    let mut min_overlap = f64::INFINITY;
    let mut best_axis = DVec3::ZERO;

    // Test 6 face normals (3 per box)
    // Test all 6 face normals (3 per box)
    let all_axes = [
        axes_a[0], axes_a[1], axes_a[2], axes_b[0], axes_b[1], axes_b[2],
    ];

    for axis in &all_axes {
        // Project half-extents of both boxes onto this axis
        let proj_a = he_a_arr[0] * axes_a[0].dot(*axis).abs()
            + he_a_arr[1] * axes_a[1].dot(*axis).abs()
            + he_a_arr[2] * axes_a[2].dot(*axis).abs();
        let proj_b = he_b_arr[0] * axes_b[0].dot(*axis).abs()
            + he_b_arr[1] * axes_b[1].dot(*axis).abs()
            + he_b_arr[2] * axes_b[2].dot(*axis).abs();

        let dist = d.dot(*axis).abs();
        let overlap = proj_a + proj_b - dist;

        if overlap <= 0.0 {
            return None; // Separating axis found
        }

        if overlap < min_overlap {
            min_overlap = overlap;
            best_axis = *axis;
            // Ensure normal points from A to B
            if d.dot(best_axis) < 0.0 {
                best_axis = -best_axis;
            }
        }
    }

    // Contact point: midpoint of the overlap region projected onto the separating axis
    let point = pos_a
        + best_axis
            * (he_a.x * axes_a[0].dot(best_axis).abs()
                + he_a.y * axes_a[1].dot(best_axis).abs()
                + he_a.z * axes_a[2].dot(best_axis).abs());

    // Better contact point: average of the face centers along the normal
    let face_a = pos_a
        + best_axis
            * (he_a.x * axes_a[0].dot(best_axis)
                + he_a.y * axes_a[1].dot(best_axis)
                + he_a.z * axes_a[2].dot(best_axis));
    let face_b = pos_b
        - best_axis
            * (he_b.x * axes_b[0].dot(best_axis)
                + he_b.y * axes_b[1].dot(best_axis)
                + he_b.z * axes_b[2].dot(best_axis));
    let contact_point = (face_a + face_b) * 0.5;

    let _ = point;

    Some((best_axis, min_overlap, contact_point))
}

/// Segment vs Sphere: closest point on segment to sphere center.
pub(super) fn segment_sphere_3d(
    seg_pos: DVec3,
    seg_rot: DQuat,
    local_a: DVec3,
    local_b: DVec3,
    sphere_pos: DVec3,
    radius: f64,
) -> Option<(DVec3, f64, DVec3)> {
    let wa = seg_pos + seg_rot * local_a;
    let wb = seg_pos + seg_rot * local_b;
    let closest = closest_point_on_segment_3d(wa, wb, sphere_pos);
    // Treat segment as zero-radius, then test against sphere
    sphere_sphere(closest, 0.0, sphere_pos, radius)
}

/// Segment vs Box: closest point on segment to box surface.
pub(super) fn segment_box_3d(
    seg_pos: DVec3,
    seg_rot: DQuat,
    local_a: DVec3,
    local_b: DVec3,
    box_pos: DVec3,
    box_rot: DQuat,
    half_extents: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    let wa = seg_pos + seg_rot * local_a;
    let wb = seg_pos + seg_rot * local_b;

    // Transform segment into box-local space
    let inv_rot = box_rot.inverse();
    let la = inv_rot * (wa - box_pos);
    let lb = inv_rot * (wb - box_pos);

    // Find closest point on segment to box (clamped to box surface)
    let ab = lb - la;
    let len_sq = ab.dot(ab);

    // Sample several points along the segment to find best contact
    let steps = 8;
    let mut best_depth = f64::NEG_INFINITY;
    let mut best_normal = DVec3::ZERO;
    let mut best_point = DVec3::ZERO;

    for i in 0..=steps {
        let t = i as f64 / steps as f64;
        let seg_pt = la + ab * t;
        let clamped = seg_pt.clamp(-half_extents, half_extents);
        let diff = seg_pt - clamped;
        let dist_sq = diff.dot(diff);

        if dist_sq < EPSILON_SQ {
            // Point is inside the box
            let face_dists = DVec3::new(
                half_extents.x - seg_pt.x.abs(),
                half_extents.y - seg_pt.y.abs(),
                half_extents.z - seg_pt.z.abs(),
            );
            let min_axis = if face_dists.x <= face_dists.y && face_dists.x <= face_dists.z {
                0
            } else if face_dists.y <= face_dists.z {
                1
            } else {
                2
            };
            let depth = face_dists[min_axis];
            if depth > best_depth {
                best_depth = depth;
                let mut n = DVec3::ZERO;
                n[min_axis] = if seg_pt[min_axis] >= 0.0 { 1.0 } else { -1.0 };
                best_normal = n;
                best_point = clamped;
            }
        } else {
            let dist = dist_sq.sqrt();
            // Negative depth means penetration — segment point is outside
            let depth = -dist;
            if depth > best_depth && dist_sq < EPSILON {
                best_depth = depth;
            }
        }
    }

    // Also check: closest point on segment to box center
    let t_center = if len_sq < EPSILON_SQ {
        0.0
    } else {
        (-la).dot(ab) / len_sq
    };
    let t_center = t_center.clamp(0.0, 1.0);
    let seg_at_center = la + ab * t_center;
    let clamped = seg_at_center.clamp(-half_extents, half_extents);
    let diff = seg_at_center - clamped;
    let dist_sq = diff.dot(diff);

    if dist_sq < EPSILON_SQ {
        let face_dists = DVec3::new(
            half_extents.x - seg_at_center.x.abs(),
            half_extents.y - seg_at_center.y.abs(),
            half_extents.z - seg_at_center.z.abs(),
        );
        let min_axis = if face_dists.x <= face_dists.y && face_dists.x <= face_dists.z {
            0
        } else if face_dists.y <= face_dists.z {
            1
        } else {
            2
        };
        let depth = face_dists[min_axis];
        if depth > best_depth {
            best_depth = depth;
            let mut n = DVec3::ZERO;
            n[min_axis] = if seg_at_center[min_axis] >= 0.0 {
                1.0
            } else {
                -1.0
            };
            best_normal = n;
            best_point = clamped;
        }
    }

    if best_depth <= 0.0 {
        return None;
    }

    // Transform back to world space
    let normal = box_rot * best_normal;
    let point = box_rot * best_point + box_pos;
    Some((normal, best_depth, point))
}

/// ConvexHull vs Sphere: closest point on hull surface to sphere center.
pub(super) fn convex_hull_sphere_3d(
    hull_points: &[[f64; 3]],
    hull_pos: DVec3,
    hull_rot: DQuat,
    sphere_pos: DVec3,
    radius: f64,
) -> Option<(DVec3, f64, DVec3)> {
    if hull_points.len() < 3 {
        return None;
    }

    // Transform hull points to world space
    let world_pts: Vec<DVec3> = hull_points
        .iter()
        .map(|p| hull_pos + hull_rot * DVec3::from_array(*p))
        .collect();

    // Find closest point on hull to sphere center.
    // For a convex hull, we test all edges (treating the hull as a wireframe).
    // This is a v1.0 approximation — works well for sphere tests.
    let n = world_pts.len();
    let mut best_dist_sq = f64::INFINITY;
    let mut best_closest = world_pts[0];

    for i in 0..n {
        let a = world_pts[i];
        let b = world_pts[(i + 1) % n];
        let closest = closest_point_on_segment_3d(a, b, sphere_pos);
        let dist_sq = (sphere_pos - closest).dot(sphere_pos - closest);
        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_closest = closest;
        }
    }

    if best_dist_sq >= radius * radius {
        return None;
    }

    let diff = sphere_pos - best_closest;
    let dist = best_dist_sq.sqrt();

    let (normal, depth) = if dist < EPSILON {
        (DVec3::Y, radius)
    } else {
        (diff / dist, radius - dist)
    };

    Some((normal, depth, best_closest))
}
