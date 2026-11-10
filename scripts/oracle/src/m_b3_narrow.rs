//! `b3_narrow.tsv` — the 3D narrowphase (M4 part 2), through the oracle's pair hook `contact_3d`
//! (`generate_contact_3d`).
//!
//! A row's inputs are shape A (`flat::shape`), A's position x, y, z and rotation x, y, z, w, then shape B and its
//! pose. `b3::contact` outputs `n:` (no contact) or the normal x, y, z (from A to B), the depth and the world point
//! x, y, z. It is class A: the 3D narrowphase is + − × ÷ √, abs, min and max only (no transcendental), so the port,
//! which keeps Rust's operation order (ADR 0012), must match every bit, signs of zero included.
//!
//! The grid: every ordered pair of a shape catalog (balls, boxes, capsules, segments and hulls, the degenerate ones
//! among them) at four rotation pairs, B placed along three directions at five distances from coincident to apart;
//! the shapes the narrowphase skips (mesh, heightfield, empty and one-point hulls) against all of them at one pose; then exact touching, concentric, inside-the-box, edge–edge,
//! lying-flat and degenerate-hull cases, large coordinates, non-finite poses, and seeded random shapes and poses.
//! Every rotation is a quaternion normalized by glam from rational components (a square root, no transcendental),
//! so no row depends on the host's libm; every box's half extents are finite and non-negative.

use hisab::{DQuat, DVec3};
use impetus::ColliderShape;
use impetus::oracle;

use crate::flat;
use crate::grids::Rng;
use crate::tsv::{Class, Module, Tok, f};

type Pose = ([f64; 3], [f64; 4]);

fn ball(r: f64) -> ColliderShape {
    ColliderShape::Ball { radius: r }
}

fn bx(hx: f64, hy: f64, hz: f64) -> ColliderShape {
    ColliderShape::Box {
        half_extents: [hx, hy, hz],
    }
}

fn capsule(hh: f64, r: f64) -> ColliderShape {
    ColliderShape::Capsule {
        half_height: hh,
        radius: r,
    }
}

fn segment(a: [f64; 3], b: [f64; 3]) -> ColliderShape {
    ColliderShape::Segment { a, b }
}

fn hull(points: &[[f64; 3]]) -> ColliderShape {
    ColliderShape::ConvexHull {
        points: points.to_vec(),
    }
}

fn cube(h: f64) -> Vec<[f64; 3]> {
    let mut v = Vec::new();
    for k in 0..8 {
        v.push([
            if k & 1 == 0 { -h } else { h },
            if k & 2 == 0 { -h } else { h },
            if k & 4 == 0 { -h } else { h },
        ]);
    }
    v
}

/// `n` seeded points on an ellipsoid of radii `r`: uniform points in the cube pushed onto the unit sphere (a square
/// root only, no transcendental: the generator's rule), then scaled per axis.
fn blob(n: usize, r: [f64; 3], rng: &mut Rng) -> Vec<[f64; 3]> {
    (0..n)
        .map(|_| {
            // A uniform point in the cube, pushed onto the unit sphere.
            let mut p = [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)];
            let l = (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt().max(1e-3);
            for (k, c) in p.iter_mut().enumerate() {
                *c = *c / l * r[k];
            }
            p
        })
        .collect()
}

/// The unit quaternion along (x, y, z, w), normalized by glam (a square root only, so no input depends on the
/// host's sin and cos).
fn rq(x: f64, y: f64, z: f64, w: f64) -> [f64; 4] {
    DQuat::from_xyzw(x, y, z, w).normalize().to_array()
}

const IDENT: [f64; 4] = [0.0, 0.0, 0.0, 1.0];

fn ins(a: &ColliderShape, pa: Pose, bs: &ColliderShape, pb: Pose) -> Vec<Tok> {
    let mut t = flat::shape(a);
    t.extend(pa.0.iter().chain(pa.1.iter()).map(|&x| f(x)));
    t.extend(flat::shape(bs));
    t.extend(pb.0.iter().chain(pb.1.iter()).map(|&x| f(x)));
    t
}

fn contact(m: &mut Module, a: &ColliderShape, pa: Pose, bs: &ColliderShape, pb: Pose) {
    let out = match oracle::contact_3d(a, pa.0, pa.1, bs, pb.0, pb.1) {
        None => vec![Tok::None],
        Some((n, d, p)) => vec![f(n[0]), f(n[1]), f(n[2]), f(d), f(p[0]), f(p[1]), f(p[2])],
    };
    m.row("b3::contact", Class::A, &ins(a, pa, bs, pb), &out);
}

/// Both orders of a pair.
fn both(m: &mut Module, a: &ColliderShape, pa: Pose, bs: &ColliderShape, pb: Pose) {
    contact(m, a, pa, bs, pb);
    contact(m, bs, pb, a, pa);
}

/// The catalog the ordered-pair sweep runs over.
fn catalog() -> Vec<ColliderShape> {
    vec![
        ball(0.5),
        ball(0.0),
        bx(0.5, 0.5, 0.5),
        bx(1.0, 0.25, 0.5),
        bx(1.0, 0.0, 0.5),
        capsule(0.5, 0.25),
        capsule(0.0, 0.4),
        capsule(0.75, 0.0),
        segment([-0.75, 0.0, 0.0], [0.75, 0.0, 0.0]),
        segment([0.2, 0.1, 0.0], [0.2, 0.1, 0.0]),
        hull(&cube(0.5)),
        hull(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]),
        hull(&[[0.6, 0.0, 0.0], [-0.6, 0.0, 0.0], [0.0, 0.6, 0.0], [0.0, -0.6, 0.0], [0.0, 0.0, 0.6], [0.0, 0.0, -0.6]]),
        hull(&[[-0.5, -0.5, 0.0], [0.5, -0.5, 0.0], [0.5, 0.5, 0.0], [-0.5, 0.5, 0.0]]),
        hull(&[[-0.5, 0.0, 0.0], [0.5, 0.0, 0.0]]),
    ]
}

/// Shapes the narrowphase returns nothing for, or nearly nothing: the mesh and heightfield (no pair arm), an empty
/// hull, and a single point. Each meets every catalog shape at one pose, both orders.
fn odd_ones() -> Vec<ColliderShape> {
    vec![
        hull(&[[0.1, 0.2, 0.3]]),
        hull(&[]),
        ColliderShape::TriMesh {
            vertices: cube(0.5),
            indices: vec![[0, 1, 2], [1, 3, 2]],
        },
        ColliderShape::Heightfield {
            heights: vec![0.0, 0.5, 0.25],
            scale: [1.0, 1.0, 1.0],
        },
    ]
}

/// A seeded shape of a kind the narrowphase handles, sizes log-uniform over about [0.03, 4).
fn random_shape(rng: &mut Rng) -> ColliderShape {
    let kind = rng.int(0, 4);
    let mut s = || rng.log_uniform(-1.3, 0.7);
    match kind {
        0 => ball(s()),
        1 => bx(s(), s(), s()),
        2 => capsule(s(), s()),
        3 => segment([-s(), 0.0, s() * 0.25], [s(), s() * 0.5, 0.0]),
        _ => {
            let r = [s(), s(), s()];
            let n = rng.int(4, 24) as usize;
            hull(&blob(n, r, rng))
        }
    }
}

fn random_quat(rng: &mut Rng) -> [f64; 4] {
    let v = DVec3::new(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)).normalize_or(DVec3::Z);
    let w = rng.uniform(-1.0, 1.0);
    let s = (1.0 - w * w).max(0.0).sqrt();
    rq(v.x * s, v.y * s, v.z * s, w)
}

#[allow(clippy::too_many_lines)] // the grid
pub fn emit() -> Module {
    let mut m = Module::new("b3_narrow");
    // About 45° about z, 90° about x, and two oblique rotations.
    let r45z = rq(0.0, 0.0, 0.414_213_562_373_095, 1.0);
    let r90x = rq(1.0, 0.0, 0.0, 1.0);
    let rxy = rq(0.34, 0.15, -0.05, 0.92);
    let rxyz = rq(-0.2, 0.5, 0.12, 0.83);

    // ── the ordered-pair sweep ──
    let cat = catalog();
    let dirs: [[f64; 3]; 3] = [[1.0, 0.0, 0.0], [0.6, 0.8, 0.0], [0.48, 0.6, -0.64]];
    let dists = [0.0, 0.3, 0.7, 1.0, 1.6];
    let rot_pairs = [(IDENT, IDENT), (IDENT, r45z), (rxy, r90x), (rxyz, IDENT)];
    for a in &cat {
        for bs in &cat {
            for &(ra, rb) in &rot_pairs {
                for dir in &dirs {
                    for &d in &dists {
                        let pb = [dir[0] * d, dir[1] * d, dir[2] * d];
                        contact(&mut m, a, ([0.0; 3], ra), bs, (pb, rb));
                    }
                }
            }
        }
    }

    for odd in &odd_ones() {
        for o in cat.iter().chain(odd_ones().iter()) {
            both(&mut m, odd, ([0.0; 3], rxy), o, ([0.2, 0.1, 0.0], IDENT));
        }
    }

    // ── balls: exactly touching, concentric, nearly concentric ──
    both(&mut m, &ball(0.5), ([0.0; 3], IDENT), &ball(0.5), ([1.0, 0.0, 0.0], IDENT));
    both(&mut m, &ball(0.5), ([0.0; 3], IDENT), &ball(0.25), ([0.0; 3], IDENT));
    both(&mut m, &ball(0.5), ([0.0; 3], IDENT), &ball(0.25), ([1e-11, 0.0, 0.0], IDENT));
    both(&mut m, &ball(0.5), ([0.0; 3], IDENT), &ball(0.5), ([0.3, -0.4, 0.5], IDENT));

    // ── a ball against a box: inside near each face, on an edge and a corner, outside each region ──
    let b1 = bx(1.0, 0.5, 0.25);
    for c in [
        [0.9, 0.0, 0.0],
        [0.0, -0.45, 0.0],
        [0.0, 0.0, 0.2],
        [0.0, 0.0, 0.0],
        [0.5, 0.5, 0.0],
        [1.0, 0.5, 0.25],
        [1.2, 0.0, 0.0],
        [1.1, 0.6, 0.0],
        [1.1, 0.6, 0.3],
        [0.0, 0.0, 0.25],
        [-0.25, 0.25, -0.125],
    ] {
        for &rot in &[IDENT, r45z, rxyz] {
            both(&mut m, &ball(0.3), (c, IDENT), &b1, ([0.0; 3], rot));
        }
    }

    // ── boxes: faces touching exactly, the AABB path's ties, face contacts with an offset incident face, edge–edge,
    // nearly parallel edges, and the identity tests at 1e-12 ──
    let cube1 = bx(0.5, 0.5, 0.5);
    both(&mut m, &cube1, ([0.0; 3], IDENT), &cube1, ([1.0, 0.0, 0.0], IDENT));
    both(&mut m, &cube1, ([0.0; 3], IDENT), &cube1, ([0.9, 0.9, 0.0], IDENT));
    both(&mut m, &cube1, ([0.0; 3], IDENT), &cube1, ([0.9, 0.9, 0.9], IDENT));
    both(&mut m, &cube1, ([0.0; 3], IDENT), &bx(2.0, 0.1, 2.0), ([0.3, 0.55, -0.2], IDENT));
    let tiny = rq(0.0, 0.0, 5e-13, 1.0);
    both(&mut m, &cube1, ([0.0; 3], tiny), &cube1, ([0.8, 0.1, 0.0], IDENT));
    both(&mut m, &cube1, ([0.0; 3], [0.0, 0.0, 0.0, -1.0]), &cube1, ([0.8, 0.1, 0.0], IDENT));
    // About 45° about x then y, and about z then x: a box edge leading.
    let edge = rq(0.353_553_39, 0.353_553_39, -0.146_446_61, 0.853_553_39);
    let edge2 = rq(0.353_553_39, 0.146_446_61, 0.353_553_39, 0.853_553_39);
    for &(pb, rb) in &[
        ([1.1, 0.0, 0.0], edge),
        ([0.0, 1.15, 0.0], edge),
        ([0.9, 0.9, 0.0], edge2),
        ([0.0, 0.0, 1.2], edge2),
        ([0.95, 0.0, 0.0], rq(0.0, 5e-8, 0.0, 1.0)),
        ([0.95, 0.2, 0.0], rq(0.0, 0.0, 0.025, 1.0)),
        ([0.0, 0.98, 0.0], r45z),
    ] {
        both(&mut m, &cube1, ([0.0; 3], IDENT), &cube1, (pb, rb));
        both(&mut m, &cube1, ([0.0; 3], r45z), &bx(1.0, 0.25, 0.5), (pb, rb));
    }

    // ── capsules: lying flat on a box (the midpoint tie), standing on it, crossing and parallel capsules, an
    // endpoint on a box edge ──
    let floor = bx(2.0, 0.5, 2.0);
    let flat = rq(0.0, 0.0, 1.0, 1.0);
    for &(pc, rc) in &[
        ([0.0, 0.7, 0.0], flat),
        ([0.3, 0.7, 0.2], flat),
        ([0.0, 1.2, 0.0], IDENT),
        ([1.9, 1.1, 0.0], rq(0.0, 0.0, 0.15, 1.0)),
        ([2.1, 0.6, 0.0], IDENT),
        ([0.0, 0.74, 0.0], rq(1.0, 0.0, 0.0, 1.0)),
    ] {
        both(&mut m, &capsule(0.5, 0.25), (pc, rc), &floor, ([0.0; 3], IDENT));
        both(&mut m, &capsule(0.5, 0.25), (pc, rc), &floor, ([0.0, 0.0, 0.0], r45z));
    }
    let cap = capsule(0.5, 0.25);
    for &(pb, rb) in &[
        ([0.4, 0.0, 0.0], IDENT),
        ([0.0, 0.0, 0.3], r90x),
        ([0.0, 0.9, 0.0], IDENT),
        ([0.3, 0.2, 0.1], rxy),
        ([0.0, 1.5, 0.0], IDENT),
    ] {
        both(&mut m, &cap, ([0.0; 3], IDENT), &cap, (pb, rb));
    }

    // ── segments against balls and boxes: through the box, grazing it, along a face, endpoints inside ──
    let seg = segment([-1.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
    for &(ps, rs) in &[
        ([0.0, 0.0, 0.0], IDENT),
        ([0.0, 0.45, 0.0], IDENT),
        ([0.0, 0.5, 0.0], IDENT),
        ([1.3, 0.0, 0.0], IDENT),
        ([0.0, 0.0, 0.0], rxyz),
        ([0.9, 0.3, 0.1], r45z),
    ] {
        both(&mut m, &seg, (ps, rs), &bx(0.5, 0.5, 0.5), ([0.0; 3], IDENT));
        both(&mut m, &seg, (ps, rs), &bx(0.5, 0.5, 0.5), ([0.0; 3], rxy));
        both(&mut m, &seg, (ps, rs), &ball(0.6), ([0.2, 0.1, 0.0], IDENT));
    }

    // ── hulls: degenerate point sets against each other and the primitives (GJK's degenerate simplex paths), the
    // origin on a face, larger point clouds (more EPA iterations) ──
    let mut rng = Rng::new(0xb3c0_0001);
    let big = hull(&blob(24, [1.0, 0.6, 0.8], &mut rng));
    let bigger = hull(&blob(64, [0.7, 1.2, 0.9], &mut rng));
    let flat_sq = hull(&[[-0.5, -0.5, 0.0], [0.5, -0.5, 0.0], [0.5, 0.5, 0.0], [-0.5, 0.5, 0.0]]);
    let line = hull(&[[-0.5, 0.0, 0.0], [0.5, 0.0, 0.0], [0.0, 0.0, 0.0]]);
    let dup = hull(&[[0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [-0.5, 0.5, 0.5], [0.5, -0.5, 0.5], [0.5, 0.5, -0.5], [0.0, 0.0, 0.0]]);
    let others = [ball(0.5), cube1.clone(), cap.clone(), hull(&cube(0.5)), flat_sq.clone(), line.clone(), dup.clone()];
    for h in [&big, &bigger, &flat_sq, &line, &dup] {
        for o in &others {
            for &(pb, rb) in &[
                ([0.0; 3], IDENT),
                ([0.5, 0.0, 0.0], IDENT),
                ([0.5, 0.5, 0.0], r45z),
                ([1.0, 0.0, 0.0], IDENT),
                ([0.3, -0.6, 0.2], rxyz),
                ([0.0, 0.0, 1.0], IDENT),
            ] {
                contact(&mut m, h, ([0.0; 3], IDENT), o, (pb, rb));
                contact(&mut m, o, (pb, rb), h, ([0.0; 3], rxy));
            }
        }
    }

    // ── large coordinates and non-finite poses ──
    for &off in &[1e6, -3.5e7, 1e12] {
        let pa = [off, off * 0.5, -off];
        let pb = [off + 0.6, off * 0.5 + 0.2, -off];
        for (a, bs) in [
            (ball(0.5), ball(0.5)),
            (cube1.clone(), cube1.clone()),
            (cube1.clone(), bx(0.5, 0.5, 0.5)),
            (cap.clone(), cube1.clone()),
            (hull(&cube(0.5)), ball(0.4)),
            (hull(&cube(0.5)), cube1.clone()),
        ] {
            both(&mut m, &a, (pa, IDENT), &bs, (pb, r45z));
        }
    }
    let nan = f64::NAN;
    let inf = f64::INFINITY;
    for pb in [[nan, 0.0, 0.0], [inf, 0.0, 0.0], [0.0, -inf, 0.0]] {
        for (a, bs) in [
            (ball(0.5), ball(0.5)),
            (ball(0.5), cube1.clone()),
            (cube1.clone(), cube1.clone()),
            (cap.clone(), cap.clone()),
            (cap.clone(), cube1.clone()),
            (seg.clone(), cube1.clone()),
            (hull(&cube(0.5)), cube1.clone()),
            (hull(&cube(0.5)), ball(0.5)),
        ] {
            both(&mut m, &a, ([0.0; 3], r45z), &bs, (pb, IDENT));
        }
    }

    // ── seeded shapes and poses, each pair placed to overlap about half the time ──
    let mut rng = Rng::new(0xb3c0_0002);
    for _ in 0..2000 {
        let a = random_shape(&mut rng);
        let bs = random_shape(&mut rng);
        let pa = [rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0)];
        let reach = rng.uniform(0.0, 2.5);
        let dir = DVec3::new(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)).normalize_or(DVec3::X);
        let pb = [pa[0] + dir.x * reach, pa[1] + dir.y * reach, pa[2] + dir.z * reach];
        let ra = if rng.int(0, 5) == 0 { IDENT } else { random_quat(&mut rng) };
        let rb = if rng.int(0, 5) == 0 { IDENT } else { random_quat(&mut rng) };
        contact(&mut m, &a, (pa, ra), &bs, (pb, rb));
    }
    m
}
