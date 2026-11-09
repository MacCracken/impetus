//! Narrowphase contact generation functions for 3D shapes.

use hisab::{DQuat, DVec3};

use super::types::{EPSILON, EPSILON_SQ};
use crate::collider::ColliderShape;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

#[inline]
#[must_use]
pub(super) fn is_identity_quat(q: DQuat) -> bool {
    (q.x.abs() < EPSILON)
        && (q.y.abs() < EPSILON)
        && (q.z.abs() < EPSILON)
        && ((q.w.abs() - 1.0).abs() < EPSILON)
}

#[inline]
#[must_use]
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
#[inline]
#[must_use]
pub(super) fn capsule_endpoints_3d(pos: DVec3, rot: DQuat, half_height: f64) -> (DVec3, DVec3) {
    let axis = rot * DVec3::new(0.0, half_height, 0.0);
    (pos - axis, pos + axis)
}

/// Closest points between two 3D line segments. Returns (point_on_ab, point_on_cd).
///
/// The minimum of the (convex) squared distance over the parameter square is
/// either on its boundary — one of the four endpoint-to-segment projections —
/// or in its interior, found by the clamped analytic solution. Taking the best
/// of the five candidates is exact, including for parallel segments.
#[must_use]
pub(super) fn closest_points_segments_3d(a: DVec3, b: DVec3, c: DVec3, d: DVec3) -> (DVec3, DVec3) {
    let ab = b - a;
    let cd = d - c;
    let d1 = ab.dot(ab);
    let d2 = cd.dot(cd);

    let candidates = [
        (a, closest_point_on_segment_3d(c, d, a)),
        (b, closest_point_on_segment_3d(c, d, b)),
        (closest_point_on_segment_3d(a, b, c), c),
        (closest_point_on_segment_3d(a, b, d), d),
    ];
    let mut best = candidates[0];
    let mut best_d = (best.0 - best.1).length_squared();
    for &(p, q) in &candidates[1..] {
        let dd = (p - q).length_squared();
        if dd < best_d {
            best = (p, q);
            best_d = dd;
        }
    }

    if d1 > EPSILON_SQ && d2 > EPSILON_SQ {
        let r = a - c;
        let d3 = ab.dot(cd);
        let d4 = ab.dot(r);
        let d5 = cd.dot(r);
        let denom = d1 * d2 - d3 * d3;
        // Relative test: denom = |ab|²|cd|² sin²θ.
        if denom > 1e-12 * d1 * d2 {
            let mut s = ((d3 * d5 - d4 * d2) / denom).clamp(0.0, 1.0);
            let mut t = ((d3 * s + d5) / d2).clamp(0.0, 1.0);
            s = ((t * d3 - d4) / d1).clamp(0.0, 1.0);
            t = ((d3 * s + d5) / d2).clamp(0.0, 1.0);
            let p = a + ab * s;
            let q = c + cd * t;
            if (p - q).length_squared() < best_d {
                best = (p, q);
            }
        }
    }

    best
}

// ---------------------------------------------------------------------------
// Contact generation dispatch
// ---------------------------------------------------------------------------

#[must_use]
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
        // Ball vs Box (`sphere_obb` returns the box -> sphere normal)
        (ColliderShape::Ball { radius }, ColliderShape::Box { half_extents }) => sphere_obb(
            pos_a,
            *radius,
            pos_b,
            rot_b,
            DVec3::from_array(*half_extents),
        )
        .map(|(n, d, p)| (-n, d, p)),
        (ColliderShape::Box { half_extents }, ColliderShape::Ball { radius }) => sphere_obb(
            pos_b,
            *radius,
            pos_a,
            rot_a,
            DVec3::from_array(*half_extents),
        ),
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
        // Capsule vs Box (`capsule_box_3d` returns the box -> capsule normal)
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
        )
        .map(|(n, d, p)| (-n, d, p)),
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
        ),
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
        // Segment vs Box (`segment_box_3d` returns the box -> segment normal)
        (ColliderShape::Segment { a, b }, ColliderShape::Box { half_extents }) => segment_box_3d(
            pos_a,
            rot_a,
            DVec3::from_array(*a),
            DVec3::from_array(*b),
            pos_b,
            rot_b,
            DVec3::from_array(*half_extents),
        )
        .map(|(n, d, p)| (-n, d, p)),
        (ColliderShape::Box { half_extents }, ColliderShape::Segment { a, b }) => segment_box_3d(
            pos_b,
            rot_b,
            DVec3::from_array(*a),
            DVec3::from_array(*b),
            pos_a,
            rot_a,
            DVec3::from_array(*half_extents),
        ),
        // ConvexHull vs Ball — GJK+EPA on the point cloud (hull -> ball normal)
        (ColliderShape::ConvexHull { points }, ColliderShape::Ball { radius }) => {
            convex_hull_sphere_3d(points, pos_a, rot_a, pos_b, *radius)
        }
        (ColliderShape::Ball { radius }, ColliderShape::ConvexHull { points }) => {
            convex_hull_sphere_3d(points, pos_b, rot_b, pos_a, *radius).map(|(n, d, p)| (-n, d, p))
        }
        // ConvexHull vs Box / Capsule / ConvexHull — GJK+EPA (A -> B)
        (
            ColliderShape::ConvexHull { .. },
            ColliderShape::Box { .. }
            | ColliderShape::Capsule { .. }
            | ColliderShape::ConvexHull { .. },
        )
        | (
            ColliderShape::Box { .. } | ColliderShape::Capsule { .. },
            ColliderShape::ConvexHull { .. },
        ) => gjk_epa_shapes(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// GJK + EPA (f64, native implementation)
// ---------------------------------------------------------------------------

/// Support function: furthest point in a direction for a shape in world space.
#[inline]
#[must_use]
fn shape_support(shape: &ColliderShape, pos: DVec3, rot: DQuat, dir: DVec3) -> DVec3 {
    let local_dir = rot.inverse() * dir;
    let local_pt = match shape {
        ColliderShape::Ball { radius } => ball_support(*radius, local_dir),
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
        ColliderShape::ConvexHull { points } => hull_support(points, local_dir),
        // For unsupported shapes, return position (degenerate)
        _ => DVec3::ZERO,
    };
    pos + rot * local_pt
}

/// Local-space support point of a sphere centred at the origin.
#[inline]
#[must_use]
fn ball_support(radius: f64, local_dir: DVec3) -> DVec3 {
    let len = local_dir.length();
    if len < EPSILON {
        DVec3::new(0.0, radius, 0.0)
    } else {
        local_dir / len * radius
    }
}

/// Local-space support point of a point cloud (convex hull).
#[inline]
#[must_use]
fn hull_support(points: &[[f64; 3]], local_dir: DVec3) -> DVec3 {
    let mut best = DVec3::ZERO;
    let mut best_dot = f64::NEG_INFINITY;
    for p in points {
        let v = DVec3::from_array(*p);
        let d = v.dot(local_dir);
        if d > best_dot {
            best_dot = d;
            best = v;
        }
    }
    best
}

/// A vertex of the Minkowski difference A − B, with the support points on A
/// and B that produced it (used to reconstruct the contact point).
#[derive(Debug, Clone, Copy)]
struct SupportPoint {
    w: DVec3,
    a: DVec3,
    b: DVec3,
}

/// GJK+EPA collision detection for arbitrary convex shapes.
/// Returns (normal_A_to_B, depth, contact_point).
#[must_use]
pub(super) fn gjk_epa_shapes(
    shape_a: &ColliderShape,
    pos_a: DVec3,
    rot_a: DQuat,
    shape_b: &ColliderShape,
    pos_b: DVec3,
    rot_b: DQuat,
) -> Option<(DVec3, f64, DVec3)> {
    gjk_epa(
        |d| shape_support(shape_a, pos_a, rot_a, d),
        |d| shape_support(shape_b, pos_b, rot_b, d),
        pos_b - pos_a,
    )
}

/// GJK (intersection) followed by EPA (penetration) on world-space support
/// functions. Returns (normal_A_to_B, depth, contact_point).
#[must_use]
fn gjk_epa(
    support_a: impl Fn(DVec3) -> DVec3,
    support_b: impl Fn(DVec3) -> DVec3,
    initial_dir: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    const GJK_MAX_ITER: usize = 64;
    const EPA_MAX_ITER: usize = 64;

    let support = |dir: DVec3| {
        let a = support_a(dir);
        let b = support_b(-dir);
        SupportPoint { w: a - b, a, b }
    };

    let initial_dir = if initial_dir.length_squared() < EPSILON_SQ {
        DVec3::X
    } else {
        initial_dir
    };

    let mut simplex: Vec<SupportPoint> = Vec::with_capacity(4);
    simplex.push(support(initial_dir));
    let mut search_dir = -simplex[0].w;

    for _ in 0..GJK_MAX_ITER {
        if search_dir.length_squared() < EPSILON_SQ {
            // The origin lies on the current simplex (touching). Search in a
            // direction orthogonal to it instead of giving up.
            search_dir = degenerate_search_dir(&simplex, initial_dir);
        }
        let new_pt = support(search_dir);
        if new_pt.w.dot(search_dir) < 0.0 {
            return None; // Separating axis found
        }
        simplex.push(new_pt);

        if gjk_do_simplex(&mut simplex, &mut search_dir) {
            return epa_penetration(&support, &simplex, EPA_MAX_ITER);
        }
    }

    None
}

/// A search direction for a simplex whose closest feature contains the origin.
#[inline]
#[must_use]
fn degenerate_search_dir(simplex: &[SupportPoint], initial_dir: DVec3) -> DVec3 {
    match simplex.len() {
        2 => (simplex[1].w - simplex[0].w).any_orthogonal_vector(),
        3 => {
            let n = (simplex[1].w - simplex[0].w).cross(simplex[2].w - simplex[0].w);
            if n.length_squared() > EPSILON_SQ {
                n
            } else {
                (simplex[1].w - simplex[0].w).any_orthogonal_vector()
            }
        }
        _ => -initial_dir,
    }
}

/// GJK simplex evolution. The newest point is last. Returns true if the
/// origin is enclosed by a tetrahedron.
fn gjk_do_simplex(simplex: &mut Vec<SupportPoint>, dir: &mut DVec3) -> bool {
    match simplex.len() {
        2 => {
            // Line case
            let a = simplex[1].w;
            let b = simplex[0].w;
            let ab = b - a;
            let ao = -a;
            if ab.dot(ao) > 0.0 {
                let perp = ab.cross(ao).cross(ab);
                *dir = if perp.length_squared() > EPSILON_SQ * ab.length_squared().max(1.0) {
                    perp
                } else {
                    // Origin on the line through a and b.
                    ab.any_orthogonal_vector()
                };
            } else {
                let keep = simplex[1];
                simplex.clear();
                simplex.push(keep);
                *dir = ao;
            }
            false
        }
        3 => {
            // Triangle case
            let (pa, pb, pc) = (simplex[2], simplex[1], simplex[0]);
            let (a, b, c) = (pa.w, pb.w, pc.w);
            let ab = b - a;
            let ac = c - a;
            let ao = -a;
            let abc = ab.cross(ac);

            if abc.length_squared() < EPSILON_SQ {
                // Collinear points: drop the oldest and treat as a line.
                simplex.clear();
                simplex.push(pb);
                simplex.push(pa);
                return gjk_do_simplex(simplex, dir);
            }

            if abc.cross(ac).dot(ao) > 0.0 {
                if ac.dot(ao) > 0.0 {
                    simplex.clear();
                    simplex.push(pc);
                    simplex.push(pa);
                    *dir = ac.cross(ao).cross(ac);
                } else {
                    simplex.clear();
                    simplex.push(pb);
                    simplex.push(pa);
                    return gjk_do_simplex(simplex, dir);
                }
            } else if ab.cross(abc).dot(ao) > 0.0 {
                simplex.clear();
                simplex.push(pb);
                simplex.push(pa);
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
            let (pa, pb, pc, pd) = (simplex[3], simplex[2], simplex[1], simplex[0]);
            let a = pa.w;
            let ao = -a;

            let ab = pb.w - a;
            let ac = pc.w - a;
            let ad = pd.w - a;

            let abc = ab.cross(ac);
            let acd = ac.cross(ad);
            let adb = ad.cross(ab);

            if abc.dot(ad).abs() < EPSILON_SQ {
                // Flat tetrahedron: drop the oldest point and search off-plane.
                simplex.clear();
                simplex.push(pc);
                simplex.push(pb);
                simplex.push(pa);
                *dir = if abc.dot(ao) >= 0.0 { abc } else { -abc };
                if dir.length_squared() < EPSILON_SQ {
                    *dir = ab.any_orthogonal_vector();
                }
                return false;
            }

            if abc.dot(ao) > 0.0 {
                simplex.clear();
                simplex.push(pc);
                simplex.push(pb);
                simplex.push(pa);
                return gjk_do_simplex(simplex, dir);
            }
            if acd.dot(ao) > 0.0 {
                simplex.clear();
                simplex.push(pd);
                simplex.push(pc);
                simplex.push(pa);
                return gjk_do_simplex(simplex, dir);
            }
            if adb.dot(ao) > 0.0 {
                simplex.clear();
                simplex.push(pb);
                simplex.push(pd);
                simplex.push(pa);
                return gjk_do_simplex(simplex, dir);
            }
            true // Origin is inside tetrahedron
        }
        _ => false,
    }
}

/// A face of the EPA polytope, wound so `normal` points away from the interior.
#[derive(Debug, Clone, Copy)]
struct EpaFace {
    idx: [usize; 3],
    normal: DVec3,
    dist: f64,
    degenerate: bool,
}

impl EpaFace {
    #[inline]
    #[must_use]
    fn new(vertices: &[SupportPoint], idx: [usize; 3]) -> Self {
        let a = vertices[idx[0]].w;
        let n = (vertices[idx[1]].w - a).cross(vertices[idx[2]].w - a);
        let len = n.length();
        if len < EPSILON {
            Self {
                idx,
                normal: DVec3::ZERO,
                dist: f64::INFINITY,
                degenerate: true,
            }
        } else {
            let normal = n / len;
            Self {
                idx,
                normal,
                dist: normal.dot(a),
                degenerate: false,
            }
        }
    }
}

/// EPA: expand the simplex polytope to find the penetration normal and depth.
///
/// If the iteration cap is hit (slow convergence on curved shapes), the
/// closest face found so far is returned.
#[must_use]
fn epa_penetration(
    support: &impl Fn(DVec3) -> SupportPoint,
    simplex: &[SupportPoint],
    max_iterations: usize,
) -> Option<(DVec3, f64, DVec3)> {
    if simplex.len() < 4 {
        return None;
    }

    let mut vertices: Vec<SupportPoint> = simplex.to_vec();
    // Orient the tetrahedron so faces [0,1,2],[0,3,1],[0,2,3],[1,3,2] wind
    // outward: face [0,1,2] must face away from vertex 3.
    let orient = (vertices[1].w - vertices[0].w)
        .cross(vertices[2].w - vertices[0].w)
        .dot(vertices[3].w - vertices[0].w);
    if orient.abs() < EPSILON_SQ {
        return None;
    }
    if orient > 0.0 {
        vertices.swap(1, 2);
    }
    let mut faces: Vec<EpaFace> = [[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]]
        .into_iter()
        .map(|idx| EpaFace::new(&vertices, idx))
        .collect();

    let mut best: Option<EpaFace> = None;

    for _ in 0..max_iterations {
        // Closest non-degenerate face to the origin
        let closest = faces
            .iter()
            .filter(|f| !f.degenerate)
            .min_by(|x, y| x.dist.total_cmp(&y.dist))
            .copied();
        let Some(face) = closest else {
            break;
        };
        best = Some(face);

        let new_pt = support(face.normal);
        let new_dist = new_pt.w.dot(face.normal);

        if new_dist - face.dist < 1e-9 + 1e-9 * face.dist.abs() {
            return Some(epa_result(&vertices, &face, support));
        }

        // Expand polytope: remove faces visible from the new point (and any
        // degenerate faces), keeping their boundary as the horizon.
        let new_idx = vertices.len();
        vertices.push(new_pt);

        let mut edges: Vec<[usize; 2]> = Vec::new();
        faces.retain(|f| {
            let a = vertices[f.idx[0]].w;
            if f.degenerate || f.normal.dot(new_pt.w - a) > 0.0 {
                for edge in [
                    [f.idx[0], f.idx[1]],
                    [f.idx[1], f.idx[2]],
                    [f.idx[2], f.idx[0]],
                ] {
                    // A shared edge appears reversed in the neighbouring face.
                    if let Some(pos) = edges
                        .iter()
                        .position(|e| e[0] == edge[1] && e[1] == edge[0])
                    {
                        edges.swap_remove(pos);
                    } else {
                        edges.push(edge);
                    }
                }
                false
            } else {
                true
            }
        });

        if edges.is_empty() {
            break;
        }
        for edge in &edges {
            faces.push(EpaFace::new(&vertices, [edge[0], edge[1], new_idx]));
        }
    }

    best.map(|face| epa_result(&vertices, &face, support))
}

/// Normal, depth and contact point for an EPA face. The contact point is the
/// midpoint of the witness points on A and B, reconstructed from the
/// barycentric coordinates of the origin's projection onto the face.
#[must_use]
fn epa_result(
    vertices: &[SupportPoint],
    face: &EpaFace,
    support: &impl Fn(DVec3) -> SupportPoint,
) -> (DVec3, f64, DVec3) {
    let [i0, i1, i2] = face.idx;
    let (v0, v1, v2) = (vertices[i0], vertices[i1], vertices[i2]);
    let p = face.normal * face.dist;
    let e0 = v1.w - v0.w;
    let e1 = v2.w - v0.w;
    let e2 = p - v0.w;
    let d00 = e0.dot(e0);
    let d01 = e0.dot(e1);
    let d11 = e1.dot(e1);
    let d20 = e2.dot(e0);
    let d21 = e2.dot(e1);
    let denom = d00 * d11 - d01 * d01;
    let point = if denom.abs() > EPSILON_SQ {
        let v = (d11 * d20 - d01 * d21) / denom;
        let w = (d00 * d21 - d01 * d20) / denom;
        let u = 1.0 - v - w;
        let on_a = v0.a * u + v1.a * v + v2.a * w;
        let on_b = v0.b * u + v1.b * v + v2.b * w;
        (on_a + on_b) * 0.5
    } else {
        let s = support(face.normal);
        (s.a + s.b) * 0.5
    };
    (face.normal, face.dist.max(0.0), point)
}

// ---------------------------------------------------------------------------
// Shape-vs-shape contact functions
// ---------------------------------------------------------------------------

#[inline]
#[must_use]
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
#[must_use]
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

#[must_use]
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

    // Contact point: centre of the overlap region.
    let lo = (pos_a - he_a).max(pos_b - he_b);
    let hi = (pos_a + he_a).min(pos_b + he_b);
    let point = (lo + hi) * 0.5;

    Some((normal, depth, point))
}

// ---------------------------------------------------------------------------
// Capsule helpers

#[must_use]
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
#[must_use]
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

/// Capsule vs Box: sphere-OBB tests at candidate points along the capsule
/// segment, keeping the deepest. Returns the box -> capsule normal.
///
/// Candidates are the midpoint (preferred on ties, so a capsule lying flat
/// gets a centred contact), the projection of the box centre, the point found
/// by alternating projection between segment and box, and both endpoints.
#[must_use]
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

    // Work in box-local space.
    let inv_rot = box_rot.inverse();
    let la = inv_rot * (ep_a - box_pos);
    let lb = inv_rot * (ep_b - box_pos);
    let ab = lb - la;
    let len_sq = ab.dot(ab);

    let project = |p: DVec3| -> f64 {
        if len_sq < EPSILON_SQ {
            0.0
        } else {
            ((p - la).dot(ab) / len_sq).clamp(0.0, 1.0)
        }
    };
    let gap_sq = |t: f64| {
        let p = la + ab * t;
        (p - p.clamp(-half_extents, half_extents)).length_squared()
    };

    let t_center = project(DVec3::ZERO);

    // Alternating projection (segment <-> box) from the closest seed.
    let mut t_alt = [0.5, t_center, 0.0, 1.0]
        .into_iter()
        .fold((0.5, f64::INFINITY), |acc, t| {
            let g = gap_sq(t);
            if g < acc.1 { (t, g) } else { acc }
        })
        .0;
    for _ in 0..8 {
        let p = la + ab * t_alt;
        t_alt = project(p.clamp(-half_extents, half_extents));
    }

    let mut best: Option<(DVec3, f64, DVec3)> = None;
    for t in [0.5, t_center, t_alt, 0.0, 1.0] {
        let world_pt = box_rot * (la + ab * t) + box_pos;
        if let Some(c) = sphere_obb(world_pt, cap_radius, box_pos, box_rot, half_extents)
            && best.is_none_or(|b| c.1 > b.1 + 1e-9)
        {
            best = Some(c);
        }
    }
    best
}

/// OBB vs OBB using the separating axis test over all 15 axes (3 face
/// normals per box plus the 9 edge-edge cross products). Returns the A -> B
/// normal, penetration depth and a contact point at the centre of the contact
/// patch (reference-face clipping for face contacts, closest points between
/// the two edges for edge contacts).
#[must_use]
pub(super) fn obb_obb_3d(
    pos_a: DVec3,
    rot_a: DQuat,
    he_a: DVec3,
    pos_b: DVec3,
    rot_b: DQuat,
    he_b: DVec3,
) -> Option<(DVec3, f64, DVec3)> {
    /// Prefer face axes (and box A) over nearly-equal alternatives so the
    /// reference feature doesn't flicker between frames.
    const REL_TOL: f64 = 0.98;
    const ABS_TOL: f64 = 1e-3;

    #[derive(Clone, Copy)]
    enum Feature {
        FaceA(usize),
        FaceB(usize),
        Edge(usize, usize),
    }

    let axes_a = [rot_a * DVec3::X, rot_a * DVec3::Y, rot_a * DVec3::Z];
    let axes_b = [rot_b * DVec3::X, rot_b * DVec3::Y, rot_b * DVec3::Z];
    let ha = he_a.to_array();
    let hb = he_b.to_array();
    let d = pos_b - pos_a;

    let overlap_on = |axis: DVec3| {
        let proj_a = ha[0] * axes_a[0].dot(axis).abs()
            + ha[1] * axes_a[1].dot(axis).abs()
            + ha[2] * axes_a[2].dot(axis).abs();
        let proj_b = hb[0] * axes_b[0].dot(axis).abs()
            + hb[1] * axes_b[1].dot(axis).abs()
            + hb[2] * axes_b[2].dot(axis).abs();
        proj_a + proj_b - d.dot(axis).abs()
    };

    // Face axes of A, then B.
    let mut best_a = (f64::INFINITY, 0);
    let mut best_b = (f64::INFINITY, 0);
    for i in 0..3 {
        let oa = overlap_on(axes_a[i]);
        if oa <= 0.0 {
            return None;
        }
        if oa < best_a.0 {
            best_a = (oa, i);
        }
        let ob = overlap_on(axes_b[i]);
        if ob <= 0.0 {
            return None;
        }
        if ob < best_b.0 {
            best_b = (ob, i);
        }
    }
    let (mut depth, mut feature, mut axis) = if best_b.0 < REL_TOL * best_a.0 - ABS_TOL {
        (best_b.0, Feature::FaceB(best_b.1), axes_b[best_b.1])
    } else {
        (best_a.0, Feature::FaceA(best_a.1), axes_a[best_a.1])
    };

    // Edge-edge axes (skipping near-parallel edge pairs).
    let mut best_edge = (f64::INFINITY, 0, 0, DVec3::ZERO);
    for (i, ea) in axes_a.iter().enumerate() {
        for (j, eb) in axes_b.iter().enumerate() {
            let c = ea.cross(*eb);
            let len = c.length();
            if len < 1e-6 {
                continue;
            }
            let ax = c / len;
            let o = overlap_on(ax);
            if o <= 0.0 {
                return None;
            }
            if o < best_edge.0 {
                best_edge = (o, i, j, ax);
            }
        }
    }
    if best_edge.0 < REL_TOL * depth - ABS_TOL {
        depth = best_edge.0;
        feature = Feature::Edge(best_edge.1, best_edge.2);
        axis = best_edge.3;
    }

    // Ensure normal points from A to B
    let normal = if d.dot(axis) < 0.0 { -axis } else { axis };

    let point = match feature {
        Feature::FaceA(i) => face_contact_point(pos_a, &axes_a, ha, i, normal, pos_b, &axes_b, hb),
        Feature::FaceB(j) => face_contact_point(pos_b, &axes_b, hb, j, -normal, pos_a, &axes_a, ha),
        Feature::Edge(i, j) => {
            let sign = |x: f64| if x >= 0.0 { 1.0 } else { -1.0 };
            let mut centre_a = pos_a;
            let mut centre_b = pos_b;
            for k in 0..3 {
                if k != i {
                    centre_a += axes_a[k] * (sign(axes_a[k].dot(normal)) * ha[k]);
                }
                if k != j {
                    centre_b -= axes_b[k] * (sign(axes_b[k].dot(normal)) * hb[k]);
                }
            }
            let (pa, pb) = closest_points_segments_3d(
                centre_a - axes_a[i] * ha[i],
                centre_a + axes_a[i] * ha[i],
                centre_b - axes_b[j] * hb[j],
                centre_b + axes_b[j] * hb[j],
            );
            (pa + pb) * 0.5
        }
    };

    Some((normal, depth, point))
}

/// Contact point for a face contact: clip the incident box's most
/// anti-parallel face against the side planes of the reference face, and
/// average the clipped points that penetrate the reference face (each moved
/// halfway to the reference plane).
#[allow(clippy::too_many_arguments)]
#[must_use]
fn face_contact_point(
    ref_pos: DVec3,
    ref_axes: &[DVec3; 3],
    ref_he: [f64; 3],
    ref_axis: usize,
    ref_normal: DVec3,
    inc_pos: DVec3,
    inc_axes: &[DVec3; 3],
    inc_he: [f64; 3],
) -> DVec3 {
    // Incident face: the face of the other box most anti-parallel to the
    // reference normal.
    let mut inc_axis = 0;
    let mut best_dot = 0.0;
    for (k, ax) in inc_axes.iter().enumerate() {
        let dt = ax.dot(ref_normal).abs();
        if dt > best_dot {
            best_dot = dt;
            inc_axis = k;
        }
    }
    let inc_n = if inc_axes[inc_axis].dot(ref_normal) > 0.0 {
        -inc_axes[inc_axis]
    } else {
        inc_axes[inc_axis]
    };
    let inc_centre = inc_pos + inc_n * inc_he[inc_axis];
    let (u, v) = ((inc_axis + 1) % 3, (inc_axis + 2) % 3);
    let eu = inc_axes[u] * inc_he[u];
    let ev = inc_axes[v] * inc_he[v];

    let mut poly = [DVec3::ZERO; 8];
    poly[..4].copy_from_slice(&[
        inc_centre + eu + ev,
        inc_centre - eu + ev,
        inc_centre - eu - ev,
        inc_centre + eu - ev,
    ]);
    let mut count = 4;

    // Clip against the 4 side planes of the reference face.
    for k in 0..3 {
        if k == ref_axis {
            continue;
        }
        for s in [1.0, -1.0] {
            let plane_n = ref_axes[k] * s;
            let plane_d = plane_n.dot(ref_pos) + ref_he[k];
            let (clipped, n_out) = clip_polygon(&poly[..count], plane_n, plane_d);
            poly = clipped;
            count = n_out;
            if count == 0 {
                break;
            }
        }
    }

    let ref_offset = ref_normal.dot(ref_pos) + ref_he[ref_axis];
    let mut sum = DVec3::ZERO;
    let mut n_pen = 0usize;
    for p in &poly[..count] {
        let pen = ref_offset - ref_normal.dot(*p);
        if pen >= 0.0 {
            sum += *p + ref_normal * (pen * 0.5);
            n_pen += 1;
        }
    }
    if n_pen > 0 {
        sum / n_pen as f64
    } else if count > 0 {
        poly[..count].iter().copied().sum::<DVec3>() / count as f64
    } else {
        // No overlap after clipping (shouldn't happen for overlapping boxes):
        // midpoint between the face centres.
        (ref_pos + ref_normal * ref_he[ref_axis] + inc_centre) * 0.5
    }
}

/// Sutherland–Hodgman: keep the part of `poly` with `n·p <= d`.
#[inline]
#[must_use]
fn clip_polygon(poly: &[DVec3], n: DVec3, d: f64) -> ([DVec3; 8], usize) {
    let mut out = [DVec3::ZERO; 8];
    let mut count = 0usize;
    let len = poly.len();
    for i in 0..len {
        let p = poly[i];
        let q = poly[(i + 1) % len];
        let dp = n.dot(p) - d;
        let dq = n.dot(q) - d;
        if dp <= 0.0 && count < 8 {
            out[count] = p;
            count += 1;
        }
        if (dp < 0.0) != (dq < 0.0) && (dp - dq).abs() > EPSILON && count < 8 {
            let t = dp / (dp - dq);
            out[count] = p + (q - p) * t;
            count += 1;
        }
    }
    (out, count)
}

/// Segment vs Sphere: closest point on segment to sphere center.
#[must_use]
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
#[must_use]
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

/// ConvexHull vs Sphere via GJK/EPA on the hull's point cloud.
/// Returns the hull -> sphere normal.
#[must_use]
pub(super) fn convex_hull_sphere_3d(
    hull_points: &[[f64; 3]],
    hull_pos: DVec3,
    hull_rot: DQuat,
    sphere_pos: DVec3,
    radius: f64,
) -> Option<(DVec3, f64, DVec3)> {
    if hull_points.is_empty() {
        return None;
    }
    let inv_rot = hull_rot.inverse();
    gjk_epa(
        |d| hull_pos + hull_rot * hull_support(hull_points, inv_rot * d),
        |d| sphere_pos + ball_support(radius, d),
        sphere_pos - hull_pos,
    )
}
