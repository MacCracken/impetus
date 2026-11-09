//! `b2_narrow.tsv` — the 2D narrowphase (M2c), through the oracle's pair hooks `contacts_2d` and
//! `shapes_touch_2d`.
//!
//! A row's inputs are shape A (`flat::shape`), A's position x, y and rotation, then shape B and its
//! pose. `b2::contacts` outputs the contacts `generate_contacts_multi` returns, in order: `v:N`,
//! then per contact the normal x, y (from A to B), the depth and the world point x, y. It is
//! class C: every world transform goes through `sin_cos`, so the reference is the corrected oracle
//! (ADR 0011; `validate.py` recomputes every row and writes `b2_narrow.cr.tsv` where glibc
//! misrounds). `b2::shapes_touch` outputs the overlap test as `i:0` / `i:1`, class A.
//!
//! The grid: every ordered pair of a shape catalog (round, box, capsule, hulls in both windings,
//! a triangle, segments, the degenerate ones) at rotations 0, π/4, π/2 and two others, B placed
//! along three directions at six distances from coincident to apart; exactly touching and
//! just-apart poses; the Rust tests' poses; non-finite poses; hulls of 24 and 40 vertices; and
//! seeded random shapes and poses.
//! Every box's half extents are finite and non-negative: a round shape against a box reaches Rust's
//! `f64::clamp(-h, h)`, which panics on an inverted or NaN range.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use impetus::ColliderShape;
use impetus::oracle;

use crate::flat;
use crate::grids::Rng;
use crate::tsv::{Class, Module, Tok, b, f};

type Pose = ([f64; 2], f64);

fn ins(a: &ColliderShape, pa: Pose, bs: &ColliderShape, pb: Pose) -> Vec<Tok> {
    let mut t = flat::shape(a);
    t.extend([f(pa.0[0]), f(pa.0[1]), f(pa.1)]);
    t.extend(flat::shape(bs));
    t.extend([f(pb.0[0]), f(pb.0[1]), f(pb.1)]);
    t
}

fn contacts(m: &mut Module, a: &ColliderShape, pa: Pose, bs: &ColliderShape, pb: Pose) {
    let c = oracle::contacts_2d(a, pa.0, pa.1, bs, pb.0, pb.1);
    let mut out = vec![Tok::V(c.len())];
    for (n, d, p) in &c {
        out.extend([f(n[0]), f(n[1]), f(*d), f(p[0]), f(p[1])]);
    }
    m.row("b2::contacts", Class::C, &ins(a, pa, bs, pb), &out);
}

fn touch(m: &mut Module, a: &ColliderShape, pa: Pose, bs: &ColliderShape, pb: Pose) {
    let t = oracle::shapes_touch_2d(a, pa.0, pa.1, bs, pb.0, pb.1);
    m.row("b2::shapes_touch", Class::A, &ins(a, pa, bs, pb), &[b(t)]);
}

fn both(m: &mut Module, a: &ColliderShape, pa: Pose, bs: &ColliderShape, pb: Pose) {
    contacts(m, a, pa, bs, pb);
    touch(m, a, pa, bs, pb);
}

fn ball(r: f64) -> ColliderShape {
    ColliderShape::Ball { radius: r }
}

fn bx(hx: f64, hy: f64) -> ColliderShape {
    ColliderShape::Box {
        half_extents: [hx, hy, 0.0],
    }
}

fn capsule(hh: f64, r: f64) -> ColliderShape {
    ColliderShape::Capsule {
        half_height: hh,
        radius: r,
    }
}

fn segment(a: [f64; 2], bb: [f64; 2]) -> ColliderShape {
    ColliderShape::Segment {
        a: [a[0], a[1], 0.0],
        b: [bb[0], bb[1], 0.0],
    }
}

fn hull(pts: &[[f64; 2]]) -> ColliderShape {
    ColliderShape::ConvexHull {
        points: pts.iter().map(|p| [p[0], p[1], 0.0]).collect(),
    }
}

/// A convex polygon of `n` vertices on an ellipse of radii (rx, ry) about the origin, from exact
/// rational points on the unit circle (no transcendental), counter-clockwise or clockwise.
fn ellipse_hull(n: u32, rx: f64, ry: f64, ccw: bool) -> ColliderShape {
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

/// The catalog the ordered-pair sweep runs over.
fn catalog() -> Vec<ColliderShape> {
    let sq = [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]];
    let mut sq_cw = sq;
    sq_cw.reverse();
    vec![
        ball(0.5),
        bx(1.0, 0.5),
        capsule(0.5, 0.25),
        hull(&sq),
        hull(&sq_cw),
        hull(&[[-0.5, -0.3], [0.7, -0.2], [0.1, 0.8]]),
        segment([-0.75, 0.0], [0.75, 0.0]),
        hull(&[[-0.5, 0.0], [0.5, 0.0]]),
        ball(0.0),
        bx(1.0, 0.0),
        capsule(0.0, 0.4),
        hull(&[[-0.5, -0.5], [0.0, 0.0], [0.5, 0.5]]),
    ]
}

/// A seeded shape: positive sizes only, so no box reaches Rust's `f64::clamp` with an inverted
/// range (module comment).
fn random_shape(rng: &mut Rng) -> ColliderShape {
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

#[allow(clippy::too_many_lines)] // the grid
pub fn emit() -> Module {
    let mut m = Module::new("b2_narrow");
    let shapes = catalog();
    let o = [0.0, 0.0];

    // Every ordered pair: rotations × directions × distances.
    let rots = [
        (0.0, 0.0),
        (0.0, FRAC_PI_2),
        (FRAC_PI_4, 0.0),
        (0.0, FRAC_PI_4),
        (0.3, -1.1),
    ];
    let dirs = [[1.0, 0.0], [0.6, 0.8], [0.0, -1.0]];
    let dists = [0.0, 0.4, 0.8, 1.2, 1.6, 2.4];
    for a in &shapes {
        for bs in &shapes {
            for &(ra, rb) in &rots {
                for u in dirs {
                    for d in dists {
                        let pb = [u[0] * d, u[1] * d];
                        contacts(&mut m, a, (o, ra), bs, (pb, rb));
                    }
                }
            }
            // The overlap test on a sample of the same poses, and A away from the origin.
            for d in [0.0, 0.8, 1.6] {
                let pa = [3.5, -2.25];
                let pb = [pa[0] + 0.6 * d, pa[1] + 0.8 * d];
                both(&mut m, a, (pa, FRAC_PI_4), bs, (pb, FRAC_PI_2));
                touch(&mut m, a, (o, 0.0), bs, ([d, 0.0], 0.0));
            }
        }
    }

    // Heightfields and triangle meshes generate no contacts, with any partner.
    let field = ColliderShape::Heightfield {
        heights: vec![0.0, 1.0, 0.5],
        scale: [1.0, 1.0, 1.0],
    };
    let mesh = ColliderShape::TriMesh {
        vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        indices: vec![[0, 1, 2]],
    };
    for s in &shapes {
        for odd in [&field, &mesh] {
            both(&mut m, odd, (o, 0.0), s, ([0.25, 0.25], 0.0));
            both(&mut m, s, (o, 0.0), odd, ([0.25, 0.25], 0.0));
        }
    }

    // Exactly touching, a hair apart, a hair overlapping: the strict and non-strict tests.
    let hair = 1e-9;
    for e in [-hair, 0.0, hair] {
        let r = (ball(0.5), ball(0.25));
        both(&mut m, &r.0, (o, 0.0), &r.1, ([0.75 + e, 0.0], 0.0));
        both(
            &mut m,
            &ball(0.5),
            (o, 0.0),
            &bx(1.0, 0.5),
            ([0.0, -1.0 - e], 0.0),
        );
        both(
            &mut m,
            &bx(1.0, 0.5),
            (o, 0.0),
            &bx(0.5, 0.5),
            ([0.25, 1.0 + e], 0.0),
        );
        both(
            &mut m,
            &bx(1.0, 0.5),
            (o, 0.0),
            &bx(0.5, 0.5),
            ([1.5 + e, 1.0 + e], 0.0),
        );
        both(
            &mut m,
            &capsule(1.0, 0.25),
            (o, FRAC_PI_2),
            &bx(5.0, 0.5),
            ([0.0, -0.75 - e], 0.0),
        );
        both(
            &mut m,
            &segment([-5.0, 0.0], [5.0, 0.0]),
            (o, 0.0),
            &bx(0.5, 0.5),
            ([1.0, 0.5 + e], 0.0),
        );
        both(
            &mut m,
            &capsule(0.5, 0.25),
            (o, 0.0),
            &segment([-1.0, 0.0], [1.0, 0.0]),
            ([0.0, -0.75 - e], FRAC_PI_2),
        );
    }

    // The Rust tests' poses (backend_2d/tests.rs and narrowphase.rs).
    let floor = bx(5.0, 0.5);
    let sq_cw = {
        let mut p = vec![[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]];
        p.reverse();
        hull(&p)
    };
    let sq = hull(&[[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]);
    let cases: Vec<(ColliderShape, Pose, ColliderShape, Pose)> = vec![
        (ball(1.0), (o, 0.0), ball(1.0), ([1.5, 0.0], 0.0)),
        (ball(1.0), (o, 0.0), ball(1.0), ([3.0, 0.0], 0.0)),
        (ball(1.0), (o, 0.0), ball(1.0), (o, 0.0)),
        (bx(1.0, 1.0), (o, 0.0), bx(1.0, 1.0), ([1.5, 0.0], 0.0)),
        (bx(1.0, 1.0), (o, 0.0), bx(1.0, 1.0), ([1.5, 0.0], 0.3)),
        (bx(1.0, 1.0), (o, 0.0), bx(1.0, 1.0), ([5.0, 0.0], 0.0)),
        (bx(1.0, 1.0), (o, 0.0), bx(1.0, 1.0), ([0.0, 1.5], 0.0)),
        (ball(0.5), ([0.0, 0.9], 0.0), floor.clone(), (o, 0.0)),
        (floor.clone(), (o, 0.0), ball(0.5), ([0.0, 0.9], 0.0)),
        (
            capsule(0.5, 0.25),
            ([0.0, 1.2], 0.0),
            floor.clone(),
            (o, 0.0),
        ),
        (
            floor.clone(),
            (o, 0.0),
            capsule(0.5, 0.25),
            ([0.0, 1.2], 0.0),
        ),
        (
            segment([-0.5, 0.0], [0.5, 0.0]),
            ([0.0, 0.45], 0.0),
            floor.clone(),
            (o, 0.0),
        ),
        (
            floor.clone(),
            (o, 0.0),
            segment([-0.5, 0.0], [0.5, 0.0]),
            ([0.0, 0.45], 0.0),
        ),
        (ball(0.3), ([0.8, 0.0], 0.0), bx(1.0, 0.1), (o, FRAC_PI_2)),
        (ball(0.3), ([0.35, 0.5], 0.0), bx(1.0, 0.1), (o, FRAC_PI_2)),
        (sq.clone(), (o, 0.0), sq.clone(), ([0.0, 1.9], 0.0)),
        (sq_cw.clone(), (o, 0.0), sq_cw.clone(), ([0.0, 1.9], 0.0)),
        (bx(10.0, 0.5), (o, 0.0), bx(0.5, 0.5), ([8.0, 0.99], 0.0)),
        (bx(10.0, 0.5), (o, 0.0), bx(0.5, 0.5), ([8.0, 0.99], 1e-3)),
        (sq.clone(), (o, 0.0), ball(0.5), ([0.0, 0.8], 0.0)),
        (sq.clone(), (o, 0.0), ball(0.2), (o, 0.0)),
        (
            floor.clone(),
            (o, 0.0),
            capsule(1.0, 0.25),
            ([0.0, 0.74], FRAC_PI_2),
        ),
        (
            segment([-5.0, 0.0], [5.0, 0.0]),
            (o, 0.0),
            bx(0.5, 0.5),
            ([1.0, 0.49], 0.0),
        ),
        (
            hull(&[[0.0, -1.0], [2.0, -1.0], [2.0, 1.0], [0.0, 1.0]]),
            ([1.0, 0.0], 0.0),
            ball(0.5),
            (o, 0.0),
        ),
        (capsule(1.0, 0.5), (o, 0.0), ball(0.5), ([0.8, 0.0], 0.0)),
        (capsule(1.0, 0.5), (o, 0.0), ball(0.5), ([3.0, 0.0], 0.0)),
        (capsule(1.0, 0.5), (o, 0.0), ball(0.5), ([0.0, 1.8], 0.0)),
        (capsule(1.0, 0.5), (o, 0.0), bx(1.0, 1.0), ([1.2, 0.0], 0.0)),
        (capsule(1.0, 0.5), (o, 0.0), bx(1.0, 1.0), ([5.0, 0.0], 0.0)),
        (
            capsule(1.0, 0.5),
            (o, 0.0),
            capsule(1.0, 0.5),
            ([0.8, 0.0], 0.0),
        ),
        (
            capsule(1.0, 0.5),
            (o, 0.0),
            capsule(1.0, 0.5),
            ([5.0, 0.0], 0.0),
        ),
        (
            capsule(1.0, 0.5),
            (o, 0.0),
            capsule(1.0, 0.5),
            ([0.0, 0.0], FRAC_PI_2),
        ),
    ];
    for (a, pa, bs, pb) in &cases {
        both(&mut m, a, *pa, bs, *pb);
    }

    // Non-finite and degenerate poses: NaN fails every comparison, so the tests fall through.
    let nan = f64::NAN;
    let inf = f64::INFINITY;
    for s in &shapes {
        for pose in [([nan, 0.0], 0.0), ([0.0, 0.0], nan), ([inf, 0.0], 0.0)] {
            both(&mut m, &ball(0.5), (o, 0.0), s, pose);
            both(&mut m, s, pose, &bx(1.0, 0.5), (o, 0.0));
        }
    }

    // Hulls past 16 vertices (the port's polygon paths move them off the stack), against each kind
    // and each other.
    let big = [
        ellipse_hull(24, 1.2, 0.8, true),
        ellipse_hull(40, 0.9, 1.1, false),
    ];
    for h in &big {
        for s in shapes.iter().chain(big.iter()) {
            for d in [0.5, 1.0, 1.5] {
                both(&mut m, h, (o, 0.3), s, ([0.6 * d, 0.8 * d], -0.7));
                contacts(&mut m, s, ([0.6 * d, 0.8 * d], -0.7), h, (o, 0.3));
            }
        }
    }

    // Seeded shapes and poses.
    let mut rng = Rng::new(0x2a17);
    let pi = std::f64::consts::PI;
    for k in 0..2400 {
        let a = random_shape(&mut rng);
        let bs = random_shape(&mut rng);
        let pa = [rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0)];
        let pb = [
            pa[0] + rng.uniform(-3.0, 3.0),
            pa[1] + rng.uniform(-3.0, 3.0),
        ];
        let (ra, rb) = (rng.uniform(-pi, pi), rng.uniform(-pi, pi));
        if k % 4 == 0 {
            both(&mut m, &a, (pa, ra), &bs, (pb, rb));
        } else {
            contacts(&mut m, &a, (pa, ra), &bs, (pb, rb));
        }
    }
    m
}
