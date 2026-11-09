//! Narrowphase contact generation functions for 2D shapes.

use super::types::EPSILON;
use super::types::EPSILON_SQ;
use crate::collider::ColliderShape;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

#[inline]
pub(super) fn world_pos(body_pos: [f64; 2], body_rot: f64, offset: [f64; 2]) -> [f64; 2] {
    let (sin, cos) = body_rot.sin_cos();
    [
        body_pos[0] + cos * offset[0] - sin * offset[1],
        body_pos[1] + sin * offset[0] + cos * offset[1],
    ]
}

/// Transform a world-space point into body-local coordinates.
#[inline]
pub(super) fn world_to_local(world_pt: [f64; 2], body_pos: [f64; 2], body_rot: f64) -> [f64; 2] {
    let dx = world_pt[0] - body_pos[0];
    let dy = world_pt[1] - body_pos[1];
    let (sin, cos) = body_rot.sin_cos();
    // Inverse rotation: transpose of rotation matrix
    [cos * dx + sin * dy, -sin * dx + cos * dy]
}

/// Transform a body-local point into world-space coordinates.
#[inline]
pub(super) fn local_to_world(local_pt: [f64; 2], body_pos: [f64; 2], body_rot: f64) -> [f64; 2] {
    let (sin, cos) = body_rot.sin_cos();
    [
        body_pos[0] + cos * local_pt[0] - sin * local_pt[1],
        body_pos[1] + sin * local_pt[0] + cos * local_pt[1],
    ]
}

/// Create an ordered manifold key from a collider pair (smaller handle first).
pub(super) fn ordered_manifold_key(
    a: crate::collider::ColliderHandle,
    b: crate::collider::ColliderHandle,
) -> super::types::ManifoldKey {
    if a.0 <= b.0 { (a, b) } else { (b, a) }
}

// ---------------------------------------------------------------------------
// Narrowphase contact generation
// ---------------------------------------------------------------------------

/// A contact: (normal from A to B, penetration depth, world-space point).
pub(super) type Contact2 = ([f64; 2], f64, [f64; 2]);

/// Deepest contact between two shapes, or `None` if they don't touch.
/// Normal points from A to B.
#[cfg(test)]
pub(super) fn generate_contact(
    shape_a: &ColliderShape,
    pos_a: [f64; 2],
    rot_a: f64,
    shape_b: &ColliderShape,
    pos_b: [f64; 2],
    rot_b: f64,
) -> Option<Contact2> {
    deepest(generate_contacts_multi(
        shape_a, pos_a, rot_a, shape_b, pos_b, rot_b,
    ))
}

/// Whether two shapes touch, without building a manifold when a single
/// round-shape test answers it (used by overlap queries).
pub(super) fn shapes_touch(
    shape_a: &ColliderShape,
    pos_a: [f64; 2],
    rot_a: f64,
    shape_b: &ColliderShape,
    pos_b: [f64; 2],
    rot_b: f64,
) -> bool {
    if matches!(shape_a, ColliderShape::Ball { .. })
        || matches!(shape_b, ColliderShape::Ball { .. })
    {
        return round_contact(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b).is_some();
    }
    !generate_contacts_multi(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b).is_empty()
}

/// Single-point contact for pairs involving a round shape (Ball, Capsule).
/// Returns `None` for unsupported pairs. Normal points from A to B.
fn round_contact(
    shape_a: &ColliderShape,
    pos_a: [f64; 2],
    rot_a: f64,
    shape_b: &ColliderShape,
    pos_b: [f64; 2],
    rot_b: f64,
) -> Option<Contact2> {
    match (shape_a, shape_b) {
        // Ball vs Ball
        (ColliderShape::Ball { radius: ra }, ColliderShape::Ball { radius: rb }) => {
            circle_circle(pos_a, *ra, pos_b, *rb)
        }
        // Ball vs Box (oriented). `circle_obb` returns box→ball normals.
        (ColliderShape::Ball { radius }, ColliderShape::Box { half_extents }) => {
            circle_obb(pos_a, *radius, pos_b, rot_b, he2(half_extents)).map(flip_normal)
        }
        (ColliderShape::Box { half_extents }, ColliderShape::Ball { radius }) => {
            circle_obb(pos_b, *radius, pos_a, rot_a, he2(half_extents))
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
        ) => capsule_circle(pos_b, rot_b, *hh, *cr, pos_a, *br)
            .map(|(n, d, p)| ([-n[0], -n[1]], d, p)),
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
        // Segment vs Ball
        (ColliderShape::Segment { a, b }, ColliderShape::Ball { radius }) => {
            segment_circle(pos_a, rot_a, *a, *b, pos_b, *radius)
        }
        (ColliderShape::Ball { radius }, ColliderShape::Segment { a, b }) => {
            segment_circle(pos_b, rot_b, *a, *b, pos_a, *radius)
                .map(|(n, d, p)| ([-n[0], -n[1]], d, p))
        }
        _ => None,
    }
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

// ---------------------------------------------------------------------------
// Capsule helpers
// ---------------------------------------------------------------------------

/// Capsule endpoints in world space. A 2D capsule is a segment with radius.
pub(super) fn capsule_endpoints(pos: [f64; 2], rot: f64, half_height: f64) -> ([f64; 2], [f64; 2]) {
    let (sin, cos) = rot.sin_cos();
    // Capsule axis is along local Y
    let dx = -sin * half_height;
    let dy = cos * half_height;
    ([pos[0] - dx, pos[1] - dy], [pos[0] + dx, pos[1] + dy])
}

/// Closest point on segment (a, b) to point p. Returns (closest_point, t_parameter).
#[inline]
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

/// Project all points of a 2D polygon onto an axis, return (min, max).
#[inline]
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
#[inline]
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
// Small helpers
// ---------------------------------------------------------------------------

#[inline]
fn he2(he: &[f64; 3]) -> [f64; 2] {
    [he[0], he[1]]
}

#[inline]
fn flip_normal((n, d, p): Contact2) -> Contact2 {
    ([-n[0], -n[1]], d, p)
}

#[inline]
fn dot2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

#[inline]
fn rotate(v: [f64; 2], rot: f64) -> [f64; 2] {
    let (sin, cos) = rot.sin_cos();
    [cos * v[0] - sin * v[1], sin * v[0] + cos * v[1]]
}

/// The deepest contact of a set.
fn deepest(contacts: impl IntoIterator<Item = Contact2>) -> Option<Contact2> {
    contacts
        .into_iter()
        .fold(None, |best: Option<Contact2>, c| match best {
            Some(b) if b.1 >= c.1 => Some(b),
            _ => Some(c),
        })
}

/// Twice the signed area of a polygon (positive for counter-clockwise).
fn signed_area_2x(pts: &[[f64; 2]]) -> f64 {
    let n = pts.len();
    (0..n)
        .map(|i| {
            let a = pts[i];
            let b = pts[(i + 1) % n];
            a[0] * b[1] - b[0] * a[1]
        })
        .sum()
}

/// Outward unit normal of edge `a → b` for a polygon with the given winding
/// (`+1.0` counter-clockwise, `-1.0` clockwise). `None` for degenerate edges.
#[inline]
fn edge_normal(a: [f64; 2], b: [f64; 2], winding: f64) -> Option<[f64; 2]> {
    let e = [b[0] - a[0], b[1] - a[1]];
    let len = (e[0] * e[0] + e[1] * e[1]).sqrt();
    if len < EPSILON {
        return None;
    }
    Some([winding * e[1] / len, -winding * e[0] / len])
}

// ---------------------------------------------------------------------------
// Oriented-box queries (computed in the box's local frame)
// ---------------------------------------------------------------------------

/// Map a contact computed in a box's local frame back to world space.
#[inline]
fn box_to_world((n, d, p): Contact2, box_pos: [f64; 2], box_rot: f64) -> Contact2 {
    (rotate(n, box_rot), d, local_to_world(p, box_pos, box_rot))
}

/// Circle vs oriented box. Normal points from the box toward the circle.
fn circle_obb(
    circle_pos: [f64; 2],
    radius: f64,
    box_pos: [f64; 2],
    box_rot: f64,
    half_extents: [f64; 2],
) -> Option<Contact2> {
    let local = world_to_local(circle_pos, box_pos, box_rot);
    circle_aabb(local, radius, [0.0, 0.0], half_extents).map(|c| box_to_world(c, box_pos, box_rot))
}

/// Capsule vs oriented box: one contact per penetrating end cap (so a capsule
/// lying on a box gets a two-point manifold), falling back to the axis point
/// nearest the box centre. Normals point from the box toward the capsule.
#[allow(clippy::too_many_arguments)]
fn capsule_obb(
    cap_pos: [f64; 2],
    cap_rot: f64,
    half_height: f64,
    cap_radius: f64,
    box_pos: [f64; 2],
    box_rot: f64,
    half_extents: [f64; 2],
) -> Vec<Contact2> {
    let local_pos = world_to_local(cap_pos, box_pos, box_rot);
    let local_rot = cap_rot - box_rot;
    let (e0, e1) = capsule_endpoints(local_pos, local_rot, half_height);
    capsule_end_contacts(e0, e1, |p| {
        circle_aabb(p, cap_radius, [0.0, 0.0], half_extents)
    })
    .into_iter()
    .map(|c| box_to_world(c, box_pos, box_rot))
    .collect()
}

/// Contacts for a capsule (segment `e0`–`e1`) against another shape, using
/// `circle_test(centre)` for a capsule-radius circle at a given axis point.
/// Tests both end caps; if neither touches, tests the axis midpoint and the
/// axis point nearest the origin of the test frame. When the two end caps give
/// different normals only the deeper one is kept (a manifold shares one
/// normal).
fn capsule_end_contacts(
    e0: [f64; 2],
    e1: [f64; 2],
    mut circle_test: impl FnMut([f64; 2]) -> Option<Contact2>,
) -> Vec<Contact2> {
    let mut out: Vec<Contact2> = [e0, e1].into_iter().filter_map(&mut circle_test).collect();
    if out.len() == 2 && dot2(out[0].0, out[1].0) < 0.999 {
        let keep = if out[0].1 >= out[1].1 { 0 } else { 1 };
        out = vec![out[keep]];
    }
    if out.is_empty() {
        let mid = [(e0[0] + e1[0]) * 0.5, (e0[1] + e1[1]) * 0.5];
        let (nearest, _) = closest_point_on_segment(e0, e1, [0.0, 0.0]);
        out.extend(deepest([mid, nearest].into_iter().filter_map(circle_test)));
    }
    out
}

/// Capsule vs convex hull. Normals point from the hull toward the capsule.
fn capsule_hull(
    cap_pos: [f64; 2],
    cap_rot: f64,
    half_height: f64,
    cap_radius: f64,
    hull_points: &[[f64; 3]],
    hull_pos: [f64; 2],
    hull_rot: f64,
) -> Vec<Contact2> {
    // Work in the hull's frame so the fallback axis point is "nearest the hull origin".
    let local_pos = world_to_local(cap_pos, hull_pos, hull_rot);
    let (e0, e1) = capsule_endpoints(local_pos, cap_rot - hull_rot, half_height);
    capsule_end_contacts(e0, e1, |p| {
        convex_hull_circle(hull_points, [0.0, 0.0], 0.0, p, cap_radius)
    })
    .into_iter()
    .map(|c| box_to_world(c, hull_pos, hull_rot))
    .collect()
}

/// Capsule vs segment. Normals point from the segment toward the capsule.
#[allow(clippy::too_many_arguments)]
fn capsule_segment(
    cap_pos: [f64; 2],
    cap_rot: f64,
    half_height: f64,
    cap_radius: f64,
    seg_pos: [f64; 2],
    seg_rot: f64,
    seg_a: [f64; 3],
    seg_b: [f64; 3],
) -> Vec<Contact2> {
    let (e0, e1) = capsule_endpoints(cap_pos, cap_rot, half_height);
    let mut out = Vec::with_capacity(2);
    for e in [e0, e1] {
        if let Some(c) = segment_circle(seg_pos, seg_rot, seg_a, seg_b, e, cap_radius) {
            out.push(c);
        }
    }
    if out.len() == 2 && dot2(out[0].0, out[1].0) < 0.999 {
        let keep = if out[0].1 >= out[1].1 { 0 } else { 1 };
        out = vec![out[keep]];
    }
    if out.is_empty() {
        let wa = world_pos(seg_pos, seg_rot, [seg_a[0], seg_a[1]]);
        let wb = world_pos(seg_pos, seg_rot, [seg_b[0], seg_b[1]]);
        let (on_seg, on_cap) = closest_points_segments(wa, wb, e0, e1);
        out.extend(circle_circle(on_seg, 0.0, on_cap, cap_radius));
    }
    out
}

// ---------------------------------------------------------------------------
// Convex hull vs circle
// ---------------------------------------------------------------------------

/// ConvexHull vs Circle contact. `hull_points` are in the hull's local frame
/// (either winding). Handles a circle centre inside the hull. Returns
/// (normal from hull to circle, depth, point on the hull surface).
fn convex_hull_circle(
    hull_points: &[[f64; 3]],
    hull_pos: [f64; 2],
    hull_rot: f64,
    circle_pos: [f64; 2],
    radius: f64,
) -> Option<Contact2> {
    let n = hull_points.len();
    if n < 2 {
        return None;
    }
    let pts: Vec<[f64; 2]> = hull_points.iter().map(|p| [p[0], p[1]]).collect();
    let winding = if signed_area_2x(&pts) < 0.0 {
        -1.0
    } else {
        1.0
    };
    let c = world_to_local(circle_pos, hull_pos, hull_rot);

    let mut best_dist_sq = f64::INFINITY;
    let mut best_closest = pts[0];
    let mut max_sep = f64::NEG_INFINITY;
    let mut sep_normal = [0.0, 1.0];
    for i in 0..n {
        let a = pts[i];
        let b = pts[(i + 1) % n];
        let (cp, _) = closest_point_on_segment(a, b, c);
        let d2 = (c[0] - cp[0]).powi(2) + (c[1] - cp[1]).powi(2);
        if d2 < best_dist_sq {
            best_dist_sq = d2;
            best_closest = cp;
        }
        if let Some(nrm) = edge_normal(a, b, winding) {
            let sep = dot2(nrm, [c[0] - a[0], c[1] - a[1]]);
            if sep > max_sep {
                max_sep = sep;
                sep_normal = nrm;
            }
        }
    }

    let (normal, depth, point) = if n >= 3 && max_sep <= 0.0 {
        // Centre inside the hull: push out through the least-penetrated face.
        let on_face = [
            c[0] - sep_normal[0] * max_sep,
            c[1] - sep_normal[1] * max_sep,
        ];
        (sep_normal, radius - max_sep, on_face)
    } else {
        if best_dist_sq >= radius * radius {
            return None;
        }
        let dist = best_dist_sq.sqrt();
        if dist < EPSILON {
            (sep_normal, radius, best_closest)
        } else {
            (
                [
                    (c[0] - best_closest[0]) / dist,
                    (c[1] - best_closest[1]) / dist,
                ],
                radius - dist,
                best_closest,
            )
        }
    };
    Some((
        rotate(normal, hull_rot),
        depth,
        local_to_world(point, hull_pos, hull_rot),
    ))
}

// ---------------------------------------------------------------------------
// Polygon contacts (Box, ConvexHull, Segment) — reference/incident face clipping
// ---------------------------------------------------------------------------

/// World-space vertices of a polygon-like shape, counter-clockwise.
/// Boxes and convex hulls are polygons; a segment is a two-vertex polygon.
fn polygon_vertices(shape: &ColliderShape, pos: [f64; 2], rot: f64) -> Option<Vec<[f64; 2]>> {
    match shape {
        ColliderShape::Box { half_extents } => {
            let (hx, hy) = (half_extents[0], half_extents[1]);
            Some(
                [[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]]
                    .into_iter()
                    .map(|p| local_to_world(p, pos, rot))
                    .collect(),
            )
        }
        ColliderShape::ConvexHull { points } if points.len() >= 2 => {
            let mut v = transform_hull(points, pos, rot);
            if signed_area_2x(&v) < 0.0 {
                v.reverse();
            }
            Some(v)
        }
        ColliderShape::Segment { a, b } => Some(vec![
            world_pos(pos, rot, [a[0], a[1]]),
            world_pos(pos, rot, [b[0], b[1]]),
        ]),
        _ => None,
    }
}

/// Largest separation of `poly2` from any edge of `poly1` (both CCW).
/// Returns (edge index, separation); separation > 0 means a separating axis.
fn max_separation(poly1: &[[f64; 2]], poly2: &[[f64; 2]]) -> Option<(usize, f64)> {
    let n = poly1.len();
    let mut best: Option<(usize, f64)> = None;
    for i in 0..n {
        let v1 = poly1[i];
        let Some(nrm) = edge_normal(v1, poly1[(i + 1) % n], 1.0) else {
            continue;
        };
        let sep = poly2
            .iter()
            .map(|v| dot2(nrm, [v[0] - v1[0], v[1] - v1[1]]))
            .fold(f64::INFINITY, f64::min);
        if best.is_none_or(|(_, b)| sep > b) {
            best = Some((i, sep));
        }
    }
    best
}

/// Clip segment `v` against the half-plane `normal·p <= offset`.
/// Returns `None` when fewer than two points remain.
fn clip_segment(v: [[f64; 2]; 2], normal: [f64; 2], offset: f64) -> Option<[[f64; 2]; 2]> {
    let d0 = dot2(normal, v[0]) - offset;
    let d1 = dot2(normal, v[1]) - offset;
    let mut out = [[0.0; 2]; 2];
    let mut count = 0;
    if d0 <= 0.0 {
        out[count] = v[0];
        count += 1;
    }
    if d1 <= 0.0 {
        out[count] = v[1];
        count += 1;
    }
    if d0 * d1 < 0.0 && count < 2 {
        let t = d0 / (d0 - d1);
        out[count] = [
            v[0][0] + t * (v[1][0] - v[0][0]),
            v[0][1] + t * (v[1][1] - v[0][1]),
        ];
        count += 1;
    }
    (count == 2).then_some(out)
}

/// Contact manifold between two convex polygons (CCW, ≥2 vertices each),
/// following the reference/incident face method (Box2D `b2CollidePolygons`).
///
/// Returns `None` if a separating axis exists, otherwise up to two contacts
/// (possibly none if clipping degenerates). Normals point from A to B.
fn collide_polygons(poly_a: &[[f64; 2]], poly_b: &[[f64; 2]]) -> Option<Vec<Contact2>> {
    /// Tolerance favouring A as the reference polygon (frame coherence).
    const REFERENCE_TOLERANCE: f64 = 5e-4;

    let (edge_a, sep_a) = max_separation(poly_a, poly_b)?;
    if sep_a > 0.0 {
        return None;
    }
    let (edge_b, sep_b) = max_separation(poly_b, poly_a)?;
    if sep_b > 0.0 {
        return None;
    }

    let (ref_poly, inc_poly, ref_edge, flip) = if sep_b > sep_a + REFERENCE_TOLERANCE {
        (poly_b, poly_a, edge_b, true)
    } else {
        (poly_a, poly_b, edge_a, false)
    };

    let n_ref = ref_poly.len();
    let v11 = ref_poly[ref_edge];
    let v12 = ref_poly[(ref_edge + 1) % n_ref];
    let ref_normal = edge_normal(v11, v12, 1.0)?;
    let tangent = [-ref_normal[1], ref_normal[0]];

    // Incident edge: the edge of the incident polygon most anti-parallel to the reference normal.
    let n_inc = inc_poly.len();
    let mut inc_edge = 0;
    let mut min_dot = f64::INFINITY;
    for i in 0..n_inc {
        if let Some(nrm) = edge_normal(inc_poly[i], inc_poly[(i + 1) % n_inc], 1.0) {
            let d = dot2(nrm, ref_normal);
            if d < min_dot {
                min_dot = d;
                inc_edge = i;
            }
        }
    }
    let incident = [inc_poly[inc_edge], inc_poly[(inc_edge + 1) % n_inc]];

    // Clip the incident edge to the reference edge's side planes.
    let Some(clipped) = clip_segment(incident, [-tangent[0], -tangent[1]], -dot2(tangent, v11))
        .and_then(|c| clip_segment(c, tangent, dot2(tangent, v12)))
    else {
        return Some(Vec::new());
    };

    let front = dot2(ref_normal, v11);
    let normal = if flip {
        [-ref_normal[0], -ref_normal[1]]
    } else {
        ref_normal
    };
    let contacts = clipped
        .into_iter()
        .filter_map(|v| {
            let sep = dot2(ref_normal, v) - front;
            (sep <= 0.0).then(|| {
                // Midway between the incident vertex and the reference face.
                let p = [
                    v[0] - ref_normal[0] * sep * 0.5,
                    v[1] - ref_normal[1] * sep * 0.5,
                ];
                (normal, -sep, p)
            })
        })
        .collect();
    Some(contacts)
}

/// Single-point SAT contact between two world-space polygons — fallback when
/// clipping degenerates. Normal points from A to B.
fn sat_polygon_contact(
    world_a: &[[f64; 2]],
    world_b: &[[f64; 2]],
    center_a: [f64; 2],
    center_b: [f64; 2],
) -> Option<Contact2> {
    let center_d = [center_b[0] - center_a[0], center_b[1] - center_a[1]];
    let mut min_overlap = f64::INFINITY;
    let mut best_axis = [0.0f64; 2];
    for hull in [world_a, world_b] {
        let n = hull.len();
        for i in 0..n {
            let Some(axis) = edge_normal(hull[i], hull[(i + 1) % n], 1.0) else {
                continue;
            };
            let (min_a, max_a) = project_hull(world_a, axis);
            let (min_b, max_b) = project_hull(world_b, axis);
            let overlap = max_a.min(max_b) - min_a.max(min_b);
            if overlap <= 0.0 {
                return None;
            }
            if overlap < min_overlap {
                min_overlap = overlap;
                best_axis = if dot2(center_d, axis) >= 0.0 {
                    axis
                } else {
                    [-axis[0], -axis[1]]
                };
            }
        }
    }
    if !min_overlap.is_finite() {
        return None;
    }
    let cp_a = support_point_poly(world_a, best_axis);
    let cp_b = support_point_poly(world_b, [-best_axis[0], -best_axis[1]]);
    let point = [(cp_a[0] + cp_b[0]) * 0.5, (cp_a[1] + cp_b[1]) * 0.5];
    Some((best_axis, min_overlap, point))
}

/// Contacts between two polygon-like shapes (Box, ConvexHull, Segment).
fn polygon_contacts(
    shape_a: &ColliderShape,
    pos_a: [f64; 2],
    rot_a: f64,
    shape_b: &ColliderShape,
    pos_b: [f64; 2],
    rot_b: f64,
) -> Vec<Contact2> {
    let (Some(va), Some(vb)) = (
        polygon_vertices(shape_a, pos_a, rot_a),
        polygon_vertices(shape_b, pos_b, rot_b),
    ) else {
        return Vec::new();
    };
    match collide_polygons(&va, &vb) {
        None => Vec::new(),
        Some(contacts) if !contacts.is_empty() => contacts,
        Some(_) => sat_polygon_contact(&va, &vb, pos_a, pos_b)
            .into_iter()
            .collect(),
    }
}

#[inline]
fn is_polygon(shape: &ColliderShape) -> bool {
    matches!(
        shape,
        ColliderShape::Box { .. }
            | ColliderShape::ConvexHull { .. }
            | ColliderShape::Segment { .. }
    )
}

// ---------------------------------------------------------------------------
// Contact manifold generation
// ---------------------------------------------------------------------------

/// All contact points between two shapes (one-shot manifold, up to two points
/// sharing one normal). Normals point from A to B. Unsupported pairs
/// (Heightfield, TriMesh) produce no contacts.
pub(super) fn generate_contacts_multi(
    shape_a: &ColliderShape,
    pos_a: [f64; 2],
    rot_a: f64,
    shape_b: &ColliderShape,
    pos_b: [f64; 2],
    rot_b: f64,
) -> Vec<Contact2> {
    use ColliderShape::{Box as BoxS, Capsule, ConvexHull, Segment};
    match (shape_a, shape_b) {
        (a, b) if is_polygon(a) && is_polygon(b) => {
            polygon_contacts(a, pos_a, rot_a, b, pos_b, rot_b)
        }
        // Capsule vs Box / ConvexHull / Segment: helpers return other→capsule normals.
        (
            Capsule {
                half_height,
                radius,
            },
            BoxS { half_extents },
        ) => capsule_obb(
            pos_a,
            rot_a,
            *half_height,
            *radius,
            pos_b,
            rot_b,
            he2(half_extents),
        )
        .into_iter()
        .map(flip_normal)
        .collect(),
        (
            BoxS { half_extents },
            Capsule {
                half_height,
                radius,
            },
        ) => capsule_obb(
            pos_b,
            rot_b,
            *half_height,
            *radius,
            pos_a,
            rot_a,
            he2(half_extents),
        ),
        (
            Capsule {
                half_height,
                radius,
            },
            ConvexHull { points },
        ) => capsule_hull(pos_a, rot_a, *half_height, *radius, points, pos_b, rot_b)
            .into_iter()
            .map(flip_normal)
            .collect(),
        (
            ConvexHull { points },
            Capsule {
                half_height,
                radius,
            },
        ) => capsule_hull(pos_b, rot_b, *half_height, *radius, points, pos_a, rot_a),
        (
            Capsule {
                half_height,
                radius,
            },
            Segment { a, b },
        ) => capsule_segment(pos_a, rot_a, *half_height, *radius, pos_b, rot_b, *a, *b)
            .into_iter()
            .map(flip_normal)
            .collect(),
        (
            Segment { a, b },
            Capsule {
                half_height,
                radius,
            },
        ) => capsule_segment(pos_b, rot_b, *half_height, *radius, pos_a, rot_a, *a, *b),
        _ => round_contact(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b)
            .into_iter()
            .collect(),
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn oneshot_aabb_aabb_generates_contacts() {
        let shape_a = ColliderShape::Box {
            half_extents: [1.0, 1.0, 0.0],
        };
        let shape_b = ColliderShape::Box {
            half_extents: [1.0, 1.0, 0.0],
        };
        // Two overlapping axis-aligned boxes
        let contacts =
            generate_contacts_multi(&shape_a, [0.0, 0.0], 0.0, &shape_b, [1.5, 0.0], 0.0);
        assert!(!contacts.is_empty(), "should generate at least one contact");
        for (normal, depth, _point) in &contacts {
            assert!(depth > &0.0, "depth should be positive");
            let len = (normal[0] * normal[0] + normal[1] * normal[1]).sqrt();
            assert!((len - 1.0).abs() < 0.01, "normal should be unit length");
        }
    }

    #[test]
    fn oneshot_obb_obb_multi_point() {
        let shape = ColliderShape::Box {
            half_extents: [1.0, 1.0, 0.0],
        };
        // Rotated box overlap should produce multiple contacts via clipping
        let contacts = generate_contacts_multi(
            &shape,
            [0.0, 0.0],
            0.0,
            &shape,
            [1.5, 0.0],
            0.3, // rotated ~17 degrees
        );
        assert!(
            !contacts.is_empty(),
            "should generate contacts for overlapping rotated boxes"
        );
    }

    #[test]
    fn oneshot_no_overlap() {
        let shape = ColliderShape::Box {
            half_extents: [1.0, 1.0, 0.0],
        };
        let contacts = generate_contacts_multi(&shape, [0.0, 0.0], 0.0, &shape, [5.0, 0.0], 0.0);
        assert!(
            contacts.is_empty(),
            "separated boxes should produce no contacts"
        );
    }

    #[test]
    fn oneshot_ball_ball_single_contact() {
        let shape_a = ColliderShape::Ball { radius: 1.0 };
        let shape_b = ColliderShape::Ball { radius: 1.0 };
        let contacts =
            generate_contacts_multi(&shape_a, [0.0, 0.0], 0.0, &shape_b, [1.5, 0.0], 0.0);
        assert_eq!(
            contacts.len(),
            1,
            "ball-ball should produce exactly one contact"
        );
    }

    fn bx(hx: f64, hy: f64) -> ColliderShape {
        ColliderShape::Box {
            half_extents: [hx, hy, 0.0],
        }
    }

    fn square_hull(h: f64, clockwise: bool) -> ColliderShape {
        let mut points = vec![[-h, -h, 0.0], [h, -h, 0.0], [h, h, 0.0], [-h, h, 0.0]];
        if clockwise {
            points.reverse();
        }
        ColliderShape::ConvexHull { points }
    }

    /// Normal must point from A to B for every box pairing, in both orders.
    #[test]
    fn box_pair_normals_point_a_to_b() {
        let ball = ColliderShape::Ball { radius: 0.5 };
        let cap = ColliderShape::Capsule {
            half_height: 0.5,
            radius: 0.25,
        };
        let seg = ColliderShape::Segment {
            a: [-0.5, 0.0, 0.0],
            b: [0.5, 0.0, 0.0],
        };
        let floor = bx(5.0, 0.5);
        for (other, y) in [(&ball, 0.9), (&cap, 1.2), (&seg, 0.45)] {
            let (n, d, _) = generate_contact(other, [0.0, y], 0.0, &floor, [0.0, 0.0], 0.0)
                .expect("other-on-floor contact");
            assert!(n[1] < -0.99 && d > 0.0, "(X, Box): n = {n:?}");
            let (n, _, _) = generate_contact(&floor, [0.0, 0.0], 0.0, other, [0.0, y], 0.0)
                .expect("floor-under-other contact");
            assert!(n[1] > 0.99, "(Box, X): n = {n:?}");
        }
    }

    /// Ball/Capsule vs Box must use the box's rotation.
    #[test]
    fn round_vs_rotated_box_respects_rotation() {
        let thin = bx(1.0, 0.1);
        let rot = std::f64::consts::FRAC_PI_2; // spans x ∈ [-0.1, 0.1], y ∈ [-1, 1]
        let ball = ColliderShape::Ball { radius: 0.3 };
        assert!(generate_contact(&ball, [0.8, 0.0], 0.0, &thin, [0.0, 0.0], rot).is_none());
        let (n, d, _) = generate_contact(&ball, [0.35, 0.5], 0.0, &thin, [0.0, 0.0], rot)
            .expect("ball touches rotated box side");
        assert!(n[0] < -0.99, "n = {n:?}");
        assert!((d - 0.05).abs() < 1e-9, "d = {d}");
    }

    /// Two overlapping hulls must collide with a two-point manifold, for
    /// either winding.
    #[test]
    fn hull_hull_two_point_manifold() {
        for cw in [false, true] {
            let h = square_hull(1.0, cw);
            let c = generate_contacts_multi(&h, [0.0, 0.0], 0.0, &h, [0.0, 1.9], 0.0);
            assert_eq!(c.len(), 2, "cw={cw}: {c:?}");
            for (n, d, _) in &c {
                assert!(n[1] > 0.99 && (d - 0.1).abs() < 1e-9, "n={n:?} d={d}");
            }
        }
    }

    /// Nearly flat resting boxes get two contacts, located under the top box.
    #[test]
    fn resting_boxes_two_points_under_top_box() {
        let ground = bx(10.0, 0.5);
        let small = bx(0.5, 0.5);
        for rot in [0.0, 1e-3] {
            let c = generate_contacts_multi(&ground, [0.0, 0.0], 0.0, &small, [8.0, 0.99], rot);
            assert_eq!(c.len(), 2, "rot={rot}: {c:?}");
            for (n, _, p) in &c {
                assert!(n[1] > 0.99, "n = {n:?}");
                assert!((7.4..=8.6).contains(&p[0]), "p = {p:?}");
            }
        }
    }

    /// A circle centred inside a hull is pushed out through the nearest face.
    #[test]
    fn circle_inside_hull_pushes_out() {
        let pts = vec![
            [-1.0, -1.0, 0.0],
            [1.0, -1.0, 0.0],
            [1.0, 1.0, 0.0],
            [-1.0, 1.0, 0.0],
        ];
        let (n, d, _) = convex_hull_circle(&pts, [0.0, 0.0], 0.0, [0.0, 0.8], 0.5).unwrap();
        assert!(n[1] > 0.99 && (d - 0.7).abs() < 1e-9, "n={n:?} d={d}");
        assert!(convex_hull_circle(&pts, [0.0, 0.0], 0.0, [0.0, 0.0], 0.2).is_some());
    }

    /// A capsule lying on a box and a box resting on a segment both get two
    /// contacts.
    #[test]
    fn lying_capsule_and_segment_floor_two_points() {
        let cap = ColliderShape::Capsule {
            half_height: 1.0,
            radius: 0.25,
        };
        let floor = bx(5.0, 0.5);
        let lying = std::f64::consts::FRAC_PI_2;
        let c = generate_contacts_multi(&floor, [0.0, 0.0], 0.0, &cap, [0.0, 0.74], lying);
        assert_eq!(c.len(), 2, "{c:?}");

        let seg = ColliderShape::Segment {
            a: [-5.0, 0.0, 0.0],
            b: [5.0, 0.0, 0.0],
        };
        let c = generate_contacts_multi(&seg, [0.0, 0.0], 0.0, &bx(0.5, 0.5), [1.0, 0.49], 0.0);
        assert_eq!(c.len(), 2, "{c:?}");
        for (n, _, _) in &c {
            assert!(n[1] > 0.99, "n = {n:?}");
        }
    }

    #[test]
    fn convex_hull_circle_degenerate_normal() {
        // Test that circle center on hull edge gets proper edge normal, not [0,1]
        let hull = vec![
            [0.0, -1.0, 0.0],
            [2.0, -1.0, 0.0],
            [2.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
        ];
        // Circle centered exactly on the left edge of the hull
        let result = convex_hull_circle(&hull, [1.0, 0.0], 0.0, [0.0, 0.0], 0.5);
        if let Some((normal, _depth, _point)) = result {
            let len = (normal[0] * normal[0] + normal[1] * normal[1]).sqrt();
            assert!((len - 1.0).abs() < 0.01, "normal should be unit length");
        }
    }
}
