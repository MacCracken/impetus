//! Collider shape builders shared by the 2D emitters (`m_b2_narrow`, `m_b2_ray`). Every grid value is
//! built without a transcendental (the generator's rule): hull vertices are rational points on an
//! ellipse.

use impetus::ColliderShape;

use crate::grids::Rng;

pub fn ball(r: f64) -> ColliderShape {
    ColliderShape::Ball { radius: r }
}

pub fn bx(hx: f64, hy: f64) -> ColliderShape {
    ColliderShape::Box {
        half_extents: [hx, hy, 0.0],
    }
}

pub fn capsule(hh: f64, r: f64) -> ColliderShape {
    ColliderShape::Capsule {
        half_height: hh,
        radius: r,
    }
}

pub fn segment(a: [f64; 2], bb: [f64; 2]) -> ColliderShape {
    ColliderShape::Segment {
        a: [a[0], a[1], 0.0],
        b: [bb[0], bb[1], 0.0],
    }
}

pub fn hull(pts: &[[f64; 2]]) -> ColliderShape {
    ColliderShape::ConvexHull {
        points: pts.iter().map(|p| [p[0], p[1], 0.0]).collect(),
    }
}

/// A convex polygon of `n` vertices on an ellipse of radii (rx, ry) about the origin, from exact
/// rational points on the unit circle (no transcendental), counter-clockwise or clockwise.
pub fn ellipse_hull(n: u32, rx: f64, ry: f64, ccw: bool) -> ColliderShape {
    // t is the half-angle tangent, swept over (-3, 3]: the angles run from about -143° to 143°,
    // increasing, so the points are distinct and in counter-clockwise order on the curve.
    let mut pts: Vec<[f64; 2]> = (0..n)
        .map(|k| {
            let t = -3.0 + 6.0 * f64::from(k + 1) / f64::from(n);
            let d = 1.0 + t * t;
            [rx * (1.0 - t * t) / d, ry * (2.0 * t) / d]
        })
        .collect();
    if !ccw {
        pts.reverse();
    }
    hull(&pts)
}

/// A seeded shape: positive sizes only, so no box reaches Rust's `f64::clamp` with an inverted
/// range (module comment).
pub fn random_shape(rng: &mut Rng) -> ColliderShape {
    match rng.int(0, 5) {
        0 => ball(rng.uniform(0.05, 2.0)),
        1 => bx(rng.uniform(0.05, 2.0), rng.uniform(0.05, 2.0)),
        2 => capsule(rng.uniform(0.0, 1.5), rng.uniform(0.05, 1.0)),
        3 => {
            let n = u32::try_from(rng.int(3, 9)).unwrap_or(3);
            let (rx, ry) = (rng.uniform(0.2, 2.0), rng.uniform(0.2, 2.0));
            let ccw = rng.int(0, 1) == 1;
            ellipse_hull(n, rx, ry, ccw)
        }
        4 => segment(
            [rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5)],
            [rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5)],
        ),
        _ => {
            let n = u32::try_from(rng.int(4, 6)).unwrap_or(4);
            ellipse_hull(n, rng.uniform(0.3, 1.5), rng.uniform(0.3, 1.5), true)
        }
    }
}
