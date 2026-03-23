//! Narrowphase contact generation functions for 2D shapes.

use crate::collider::ColliderShape;
use super::types::EPSILON;
use super::types::EPSILON_SQ;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

pub(super) fn world_pos(body_pos: [f64; 2], body_rot: f64, offset: [f64; 2]) -> [f64; 2] {
    let (sin, cos) = body_rot.sin_cos();
    [
        body_pos[0] + cos * offset[0] - sin * offset[1],
        body_pos[1] + sin * offset[0] + cos * offset[1],
    ]
}

/// Transform a world-space point into body-local coordinates.
pub(super) fn world_to_local(world_pt: [f64; 2], body_pos: [f64; 2], body_rot: f64) -> [f64; 2] {
    let dx = world_pt[0] - body_pos[0];
    let dy = world_pt[1] - body_pos[1];
    let (sin, cos) = body_rot.sin_cos();
    // Inverse rotation: transpose of rotation matrix
    [cos * dx + sin * dy, -sin * dx + cos * dy]
}

/// Transform a body-local point into world-space coordinates.
pub(super) fn local_to_world(local_pt: [f64; 2], body_pos: [f64; 2], body_rot: f64) -> [f64; 2] {
    let (sin, cos) = body_rot.sin_cos();
    [
        body_pos[0] + cos * local_pt[0] - sin * local_pt[1],
        body_pos[1] + sin * local_pt[0] + cos * local_pt[1],
    ]
}

/// Create an ordered manifold key from a collider pair (smaller handle first).
pub(super) fn ordered_manifold_key(a: crate::collider::ColliderHandle, b: crate::collider::ColliderHandle) -> super::types::ManifoldKey {
    if a.0 <= b.0 { (a, b) } else { (b, a) }
}

// ---------------------------------------------------------------------------
// Narrowphase contact generation
// ---------------------------------------------------------------------------

pub(super) fn generate_contact(
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
        // Box vs Box — use OBB-OBB SAT when either box is rotated, fast AABB path otherwise
        (
            ColliderShape::Box { half_extents: he_a },
            ColliderShape::Box { half_extents: he_b },
        ) => {
            if rot_a.abs() < EPSILON && rot_b.abs() < EPSILON {
                aabb_aabb_contact(pos_a, [he_a[0], he_a[1]], pos_b, [he_b[0], he_b[1]])
            } else {
                obb_obb_contact(pos_a, rot_a, [he_a[0], he_a[1]], pos_b, rot_b, [he_b[0], he_b[1]])
            }
        }
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
        // ConvexHull vs Ball
        (ColliderShape::ConvexHull { points }, ColliderShape::Ball { radius }) => {
            convex_hull_circle(points, pos_a, rot_a, pos_b, *radius)
        }
        (ColliderShape::Ball { radius }, ColliderShape::ConvexHull { points }) => {
            convex_hull_circle(points, pos_b, rot_b, pos_a, *radius)
                .map(|(n, d, p)| ([-n[0], -n[1]], d, p))
        }
        // ConvexHull vs ConvexHull
        (
            ColliderShape::ConvexHull { points: pts_a },
            ColliderShape::ConvexHull { points: pts_b },
        ) => convex_convex_contact(pts_a, pos_a, rot_a, pts_b, pos_b, rot_b),
        // ConvexHull vs Box (treat box as 4-vertex convex hull)
        (ColliderShape::ConvexHull { points }, ColliderShape::Box { half_extents }) => {
            let box_pts = box_to_convex_points(*half_extents);
            convex_convex_contact(points, pos_a, rot_a, &box_pts, pos_b, rot_b)
        }
        (ColliderShape::Box { half_extents }, ColliderShape::ConvexHull { points }) => {
            let box_pts = box_to_convex_points(*half_extents);
            convex_convex_contact(&box_pts, pos_a, rot_a, points, pos_b, rot_b)
        }
        // Segment vs Ball
        (ColliderShape::Segment { a, b }, ColliderShape::Ball { radius }) => {
            segment_circle(pos_a, rot_a, *a, *b, pos_b, *radius)
        }
        (ColliderShape::Ball { radius }, ColliderShape::Segment { a, b }) => {
            segment_circle(pos_b, rot_b, *a, *b, pos_a, *radius)
                .map(|(n, d, p)| ([-n[0], -n[1]], d, p))
        }
        // Segment vs Box
        (ColliderShape::Segment { a, b }, ColliderShape::Box { half_extents }) => {
            segment_box(pos_a, rot_a, *a, *b, pos_b, rot_b, [half_extents[0], half_extents[1]])
        }
        (ColliderShape::Box { half_extents }, ColliderShape::Segment { a, b }) => {
            segment_box(pos_b, rot_b, *a, *b, pos_a, rot_a, [half_extents[0], half_extents[1]])
                .map(|(n, d, p)| ([-n[0], -n[1]], d, p))
        }
        _ => None,
    }
}

/// ConvexHull vs Circle contact.
/// `hull_points` are in the hull's local space. `hull_pos`/`hull_rot` transform them to world.
/// Returns (normal_from_hull_to_circle, depth, contact_point).
fn convex_hull_circle(
    hull_points: &[[f64; 3]],
    hull_pos: [f64; 2],
    hull_rot: f64,
    circle_pos: [f64; 2],
    radius: f64,
) -> Option<([f64; 2], f64, [f64; 2])> {
    if hull_points.len() < 2 {
        return None;
    }

    let (sin, cos) = hull_rot.sin_cos();

    // Transform hull points to world space (2D)
    let world_pts: Vec<[f64; 2]> = hull_points
        .iter()
        .map(|p| {
            [
                hull_pos[0] + cos * p[0] - sin * p[1],
                hull_pos[1] + sin * p[0] + cos * p[1],
            ]
        })
        .collect();

    // Find closest point on hull perimeter to circle center
    let n = world_pts.len();
    let mut best_dist_sq = f64::INFINITY;
    let mut best_closest = world_pts[0];

    for i in 0..n {
        let a = world_pts[i];
        let b = world_pts[(i + 1) % n];
        let (closest, _) = closest_point_on_segment(a, b, circle_pos);
        let dx = circle_pos[0] - closest[0];
        let dy = circle_pos[1] - closest[1];
        let dist_sq = dx * dx + dy * dy;
        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_closest = closest;
        }
    }

    if best_dist_sq >= radius * radius {
        return None;
    }

    let dx = circle_pos[0] - best_closest[0];
    let dy = circle_pos[1] - best_closest[1];
    let dist = best_dist_sq.sqrt();

    let (normal, depth) = if dist < EPSILON {
        // Circle center is on the hull edge; use edge normal
        // Find the edge that gave the closest point and compute its outward normal
        ([0.0, 1.0], radius)
    } else {
        ([dx / dist, dy / dist], radius - dist)
    };

    Some((normal, depth, best_closest))
}

pub(super) fn circle_circle(
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
    let (normal, depth) = if dist < EPSILON {
        ([0.0, 1.0], sum_r)
    } else {
        ([dx / dist, dy / dist], sum_r - dist)
    };

    let point = [pos_a[0] + normal[0] * ra, pos_a[1] + normal[1] * ra];
    Some((normal, depth, point))
}

pub(super) fn circle_aabb(
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
    let (normal, depth) = if dist < EPSILON {
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

pub(super) fn aabb_aabb_contact(
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
// OBB-OBB contact (Separating Axis Theorem for 2D oriented bounding boxes)
// ---------------------------------------------------------------------------

/// OBB-OBB contact using 2D SAT with 4 separating axes (2 edge normals per box).
/// Returns (normal_from_a_to_b, penetration_depth, contact_point).
#[allow(clippy::too_many_arguments)]
fn obb_obb_contact(
    pos_a: [f64; 2],
    rot_a: f64,
    he_a: [f64; 2],
    pos_b: [f64; 2],
    rot_b: f64,
    he_b: [f64; 2],
) -> Option<([f64; 2], f64, [f64; 2])> {
    let (sin_a, cos_a) = rot_a.sin_cos();
    let (sin_b, cos_b) = rot_b.sin_cos();

    // Local axes for each box
    let axes_a = [[cos_a, sin_a], [-sin_a, cos_a]];
    let axes_b = [[cos_b, sin_b], [-sin_b, cos_b]];

    // Centre-to-centre vector
    let d = [pos_b[0] - pos_a[0], pos_b[1] - pos_a[1]];

    let mut min_overlap = f64::INFINITY;
    let mut best_axis = [0.0f64; 2];

    // Test all 4 axes (2 per box)
    let all_axes = [axes_a[0], axes_a[1], axes_b[0], axes_b[1]];

    for axis in &all_axes {
        // Project half-extents of A onto axis
        let proj_a = he_a[0] * (axes_a[0][0] * axis[0] + axes_a[0][1] * axis[1]).abs()
            + he_a[1] * (axes_a[1][0] * axis[0] + axes_a[1][1] * axis[1]).abs();
        // Project half-extents of B onto axis
        let proj_b = he_b[0] * (axes_b[0][0] * axis[0] + axes_b[0][1] * axis[1]).abs()
            + he_b[1] * (axes_b[1][0] * axis[0] + axes_b[1][1] * axis[1]).abs();
        // Distance between centres along this axis
        let dist = d[0] * axis[0] + d[1] * axis[1];

        let overlap = proj_a + proj_b - dist.abs();
        if overlap <= 0.0 {
            return None; // Separating axis found
        }

        if overlap < min_overlap {
            min_overlap = overlap;
            // Normal should point from A to B
            if dist >= 0.0 {
                best_axis = *axis;
            } else {
                best_axis = [-axis[0], -axis[1]];
            }
        }
    }

    // Contact point: midpoint of support points on each box's surface toward the other.
    let cp_a = obb_support_point(pos_a, he_a, &axes_a, best_axis);
    let cp_b = obb_support_point(pos_b, he_b, &axes_b, [-best_axis[0], -best_axis[1]]);
    let point = [(cp_a[0] + cp_b[0]) * 0.5, (cp_a[1] + cp_b[1]) * 0.5];

    Some((best_axis, min_overlap, point))
}

/// Compute the support point of an OBB in a given direction.
/// Returns the corner of the box that is furthest in `dir`.
fn obb_support_point(
    center: [f64; 2],
    half_extents: [f64; 2],
    axes: &[[f64; 2]; 2],
    dir: [f64; 2],
) -> [f64; 2] {
    let mut point = center;
    for i in 0..2 {
        let dot = axes[i][0] * dir[0] + axes[i][1] * dir[1];
        let sign = if dot >= 0.0 { 1.0 } else { -1.0 };
        point[0] += sign * half_extents[i] * axes[i][0];
        point[1] += sign * half_extents[i] * axes[i][1];
    }
    point
}

// ---------------------------------------------------------------------------
// Capsule helpers
// ---------------------------------------------------------------------------

/// Capsule endpoints in world space. A 2D capsule is a segment with radius.
pub(super) fn capsule_endpoints(pos: [f64; 2], rot: f64, half_height: f64) -> ([f64; 2], [f64; 2]) {
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
pub(super) fn closest_point_on_segment(a: [f64; 2], b: [f64; 2], p: [f64; 2]) -> ([f64; 2], f64) {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let len_sq = ab[0] * ab[0] + ab[1] * ab[1];
    if len_sq < EPSILON_SQ {
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
    if d1 > EPSILON_SQ && d2 > EPSILON_SQ {
        let r = [a[0] - c[0], a[1] - c[1]];
        let d3 = ab[0] * cd[0] + ab[1] * cd[1]; // AB·CD
        let d4 = ab[0] * r[0] + ab[1] * r[1]; // AB·r
        let d5 = cd[0] * r[0] + cd[1] * r[1]; // CD·r
        let denom = d1 * d2 - d3 * d3;

        if denom.abs() > EPSILON_SQ {
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
// ConvexHull-vs-ConvexHull (2D SAT)
// ---------------------------------------------------------------------------

/// Convert a box (half_extents) into 4 convex hull points in local space (CCW).
fn box_to_convex_points(half_extents: [f64; 3]) -> Vec<[f64; 3]> {
    let hx = half_extents[0];
    let hy = half_extents[1];
    vec![
        [-hx, -hy, 0.0],
        [hx, -hy, 0.0],
        [hx, hy, 0.0],
        [-hx, hy, 0.0],
    ]
}

/// Transform hull points from local to world 2D.
fn transform_hull(points: &[[f64; 3]], pos: [f64; 2], rot: f64) -> Vec<[f64; 2]> {
    let (sin, cos) = rot.sin_cos();
    points
        .iter()
        .map(|p| {
            [
                pos[0] + cos * p[0] - sin * p[1],
                pos[1] + sin * p[0] + cos * p[1],
            ]
        })
        .collect()
}

/// 2D SAT (Separating Axis Theorem) for two convex polygons.
/// Returns (normal_from_a_to_b, penetration_depth, contact_point).
fn convex_convex_contact(
    points_a: &[[f64; 3]],
    pos_a: [f64; 2],
    rot_a: f64,
    points_b: &[[f64; 3]],
    pos_b: [f64; 2],
    rot_b: f64,
) -> Option<([f64; 2], f64, [f64; 2])> {
    let world_a = transform_hull(points_a, pos_a, rot_a);
    let world_b = transform_hull(points_b, pos_b, rot_b);

    if world_a.len() < 2 || world_b.len() < 2 {
        return None;
    }

    let mut min_overlap = f64::INFINITY;
    let mut best_axis = [0.0f64; 2];

    // Centre-to-centre direction for consistent normal orientation
    let center_d = [pos_b[0] - pos_a[0], pos_b[1] - pos_a[1]];

    // Test edge normals from both polygons
    for hull in [&world_a, &world_b] {
        let n = hull.len();
        for i in 0..n {
            let j = (i + 1) % n;
            let edge = [hull[j][0] - hull[i][0], hull[j][1] - hull[i][1]];
            let len = (edge[0] * edge[0] + edge[1] * edge[1]).sqrt();
            if len < EPSILON {
                continue;
            }
            // Outward normal (perpendicular to edge)
            let axis = [-edge[1] / len, edge[0] / len];

            // Project both hulls onto this axis
            let (min_a, max_a) = project_hull(&world_a, axis);
            let (min_b, max_b) = project_hull(&world_b, axis);

            let overlap = (max_a.min(max_b)) - (min_a.max(min_b));
            if overlap <= 0.0 {
                return None; // Separating axis found
            }

            if overlap < min_overlap {
                min_overlap = overlap;
                // Ensure normal points from A to B
                let dot = center_d[0] * axis[0] + center_d[1] * axis[1];
                if dot >= 0.0 {
                    best_axis = axis;
                } else {
                    best_axis = [-axis[0], -axis[1]];
                }
            }
        }
    }

    // Contact point: average of support points
    let cp_a = support_point_poly(&world_a, best_axis);
    let cp_b = support_point_poly(&world_b, [-best_axis[0], -best_axis[1]]);
    let point = [(cp_a[0] + cp_b[0]) * 0.5, (cp_a[1] + cp_b[1]) * 0.5];

    Some((best_axis, min_overlap, point))
}

/// Project all points of a 2D polygon onto an axis, return (min, max).
fn project_hull(points: &[[f64; 2]], axis: [f64; 2]) -> (f64, f64) {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for p in points {
        let proj = p[0] * axis[0] + p[1] * axis[1];
        min = min.min(proj);
        max = max.max(proj);
    }
    (min, max)
}

/// Find the vertex of a 2D polygon furthest in a given direction.
fn support_point_poly(points: &[[f64; 2]], dir: [f64; 2]) -> [f64; 2] {
    let mut best = points[0];
    let mut best_dot = best[0] * dir[0] + best[1] * dir[1];
    for p in &points[1..] {
        let d = p[0] * dir[0] + p[1] * dir[1];
        if d > best_dot {
            best_dot = d;
            best = *p;
        }
    }
    best
}

// ---------------------------------------------------------------------------
// Segment narrowphase contacts
// ---------------------------------------------------------------------------

/// Segment-vs-Ball: find closest point on segment to ball center, test distance vs radius.
/// The segment endpoints are in the segment body's local space.
fn segment_circle(
    seg_pos: [f64; 2],
    seg_rot: f64,
    seg_a: [f64; 3],
    seg_b: [f64; 3],
    circle_pos: [f64; 2],
    circle_radius: f64,
) -> Option<([f64; 2], f64, [f64; 2])> {
    let (sin, cos) = seg_rot.sin_cos();
    let wa = [
        seg_pos[0] + cos * seg_a[0] - sin * seg_a[1],
        seg_pos[1] + sin * seg_a[0] + cos * seg_a[1],
    ];
    let wb = [
        seg_pos[0] + cos * seg_b[0] - sin * seg_b[1],
        seg_pos[1] + sin * seg_b[0] + cos * seg_b[1],
    ];
    let (closest, _) = closest_point_on_segment(wa, wb, circle_pos);
    // Segment has zero radius, so it's like a capsule_circle with r=0
    let dx = circle_pos[0] - closest[0];
    let dy = circle_pos[1] - closest[1];
    let dist_sq = dx * dx + dy * dy;

    if dist_sq >= circle_radius * circle_radius {
        return None;
    }

    let dist = dist_sq.sqrt();
    let (normal, depth) = if dist < EPSILON {
        // Degenerate: circle center is on the segment
        ([0.0, 1.0], circle_radius)
    } else {
        ([dx / dist, dy / dist], circle_radius - dist)
    };

    Some((normal, depth, closest))
}

/// Segment-vs-Box: find overlap between a line segment and an axis-aligned box.
#[allow(clippy::too_many_arguments)]
fn segment_box(
    seg_pos: [f64; 2],
    seg_rot: f64,
    seg_a: [f64; 3],
    seg_b: [f64; 3],
    box_pos: [f64; 2],
    _box_rot: f64,
    half_extents: [f64; 2],
) -> Option<([f64; 2], f64, [f64; 2])> {
    // Transform segment to world space
    let (sin, cos) = seg_rot.sin_cos();
    let wa = [
        seg_pos[0] + cos * seg_a[0] - sin * seg_a[1],
        seg_pos[1] + sin * seg_a[0] + cos * seg_a[1],
    ];
    let wb = [
        seg_pos[0] + cos * seg_b[0] - sin * seg_b[1],
        seg_pos[1] + sin * seg_b[0] + cos * seg_b[1],
    ];

    // Find closest point on segment to box center
    let (closest_on_seg, _) = closest_point_on_segment(wa, wb, box_pos);

    // Clamp that point to box bounds to find closest point on box
    let dx = closest_on_seg[0] - box_pos[0];
    let dy = closest_on_seg[1] - box_pos[1];
    let cx = dx.clamp(-half_extents[0], half_extents[0]);
    let cy = dy.clamp(-half_extents[1], half_extents[1]);
    let box_closest = [box_pos[0] + cx, box_pos[1] + cy];

    // Now find closest point on segment to that box point
    let (seg_closest, _) = closest_point_on_segment(wa, wb, box_closest);

    let diff_x = seg_closest[0] - box_closest[0];
    let diff_y = seg_closest[1] - box_closest[1];
    let dist_sq = diff_x * diff_x + diff_y * diff_y;

    // Segment has zero radius — we need the segment point to be inside the box
    // or touching it. Check if segment point is inside the box.
    let rel_x = seg_closest[0] - box_pos[0];
    let rel_y = seg_closest[1] - box_pos[1];

    if rel_x.abs() <= half_extents[0] && rel_y.abs() <= half_extents[1] {
        // Segment point is inside the box — push out along minimum penetration axis
        let pen_x = half_extents[0] - rel_x.abs();
        let pen_y = half_extents[1] - rel_y.abs();
        if pen_x < pen_y {
            let sign = if rel_x >= 0.0 { 1.0 } else { -1.0 };
            Some(([sign, 0.0], pen_x, seg_closest))
        } else {
            let sign = if rel_y >= 0.0 { 1.0 } else { -1.0 };
            Some(([0.0, sign], pen_y, seg_closest))
        }
    } else if dist_sq < EPSILON_SQ {
        // Touching but not inside — very shallow contact
        None
    } else {
        // Not overlapping
        None
    }
}
