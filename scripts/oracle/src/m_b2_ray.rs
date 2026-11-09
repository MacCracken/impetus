//! `b2_ray.tsv` — the 2D raycast and overlap queries (M2d), through the public world API.
//!
//! Each row builds a world of static bodies, one collider each, and runs one query. Inputs: `v:<bodies>`,
//! then per body its x, y, rotation, its collider's shape (`flat::shape`), offset x, y and collision
//! layer; then the query:
//! - `b2::raycast` (class C): the ray's origin x, y, direction x, y, max distance and layer mask
//!   (`raycast_filtered`). Output: `n:`, or the hit's collider handle, point x, y, normal x, y and
//!   distance. Every transform is a `sin_cos` and every length a `hypot`, both correctly rounded in the
//!   port, so the reference is the corrected oracle (ADR 0011; `validate.py` recomputes every row).
//! - `b2::overlap_sphere` (class A): the centre x, y and radius. Output: the colliders found, `v:N`
//!   then their handles, in collider order.
//! - `b2::overlap_aabb` (class A): min x, y and max x, y. Output as for the sphere.
//!
//! The overlap rows also pin `Collider2d::world_aabb`, the queries' pre-filter (heightfields and
//! meshes are accepted on their boxes alone). Every box has finite, non-negative half extents: a
//! round probe against a box reaches Rust's `f64::clamp(-h, h)`, which panics on an inverted range.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use impetus::{
    BodyDesc, BodyType, ColliderDesc, ColliderShape, PhysicsMaterial, PhysicsWorld, WorldConfig,
};

use crate::flat;
use crate::grids::Rng;
use crate::shapes::{ball, bx, capsule, ellipse_hull, hull, random_shape, segment};
use crate::tsv::{Class, Module, Tok, f, i};

/// One body and its collider: position, rotation, shape, offset, collision layer.
#[derive(Clone)]
struct Body {
    pos: [f64; 2],
    rot: f64,
    shape: ColliderShape,
    offset: [f64; 2],
    layer: u32,
}

fn body(pos: [f64; 2], rot: f64, shape: &ColliderShape) -> Body {
    Body {
        pos,
        rot,
        shape: shape.clone(),
        offset: [0.0, 0.0],
        layer: u32::MAX,
    }
}

/// The world and the row's body tokens.
fn world(bodies: &[Body]) -> (PhysicsWorld, Vec<Tok>) {
    let mut w = PhysicsWorld::new(WorldConfig::default());
    let mut t = vec![Tok::V(bodies.len())];
    for bd in bodies {
        let h = w.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [bd.pos[0], bd.pos[1], 0.0],
            rotation: bd.rot,
            ..BodyDesc::default()
        });
        w.add_collider(
            h,
            ColliderDesc {
                shape: bd.shape.clone(),
                offset: [bd.offset[0], bd.offset[1], 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: bd.layer,
                collision_mask: u32::MAX,
            },
        );
        t.extend([f(bd.pos[0]), f(bd.pos[1]), f(bd.rot)]);
        t.extend(flat::shape(&bd.shape));
        t.extend([f(bd.offset[0]), f(bd.offset[1]), i(bd.layer)]);
    }
    (w, t)
}

fn handle(x: u64) -> Tok {
    Tok::I(x.cast_signed())
}

fn ray(m: &mut Module, bodies: &[Body], o: [f64; 2], d: [f64; 2], max: f64, mask: u32) {
    let (w, mut ins) = world(bodies);
    ins.extend([f(o[0]), f(o[1]), f(d[0]), f(d[1]), f(max), i(mask)]);
    let out = match w.raycast_filtered([o[0], o[1], 0.0], [d[0], d[1], 0.0], max, mask) {
        None => vec![Tok::None],
        Some(h) => vec![
            handle(h.collider.0),
            f(h.point[0]),
            f(h.point[1]),
            f(h.normal[0]),
            f(h.normal[1]),
            f(h.distance),
        ],
    };
    m.row("b2::raycast", Class::C, &ins, &out);
}

fn handles(hs: &[impetus::ColliderHandle]) -> Vec<Tok> {
    let mut t = vec![Tok::V(hs.len())];
    t.extend(hs.iter().map(|h| handle(h.0)));
    t
}

fn sphere(m: &mut Module, bodies: &[Body], c: [f64; 2], r: f64) {
    let (w, mut ins) = world(bodies);
    ins.extend([f(c[0]), f(c[1]), f(r)]);
    let found = w.overlap_sphere([c[0], c[1], 0.0], r);
    m.row("b2::overlap_sphere", Class::A, &ins, &handles(&found));
}

fn aabb(m: &mut Module, bodies: &[Body], lo: [f64; 2], hi: [f64; 2]) {
    let (w, mut ins) = world(bodies);
    ins.extend([f(lo[0]), f(lo[1]), f(hi[0]), f(hi[1])]);
    let found = w.overlap_aabb([lo[0], lo[1], 0.0], [hi[0], hi[1], 0.0]);
    m.row("b2::overlap_aabb", Class::A, &ins, &handles(&found));
}

/// The shapes every sweep runs over: round, box, capsule, hulls in both windings and a triangle, a
/// segment, the hulls a ray skips (two points), the field and the mesh, and the degenerate ones.
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
        ColliderShape::Heightfield {
            heights: vec![0.0, 1.0, 0.5],
            scale: [0.5, 1.0, 1.0],
        },
        ColliderShape::TriMesh {
            vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            indices: vec![[0, 1, 2]],
        },
        ball(0.0),
        bx(1.0, 0.0),
        capsule(0.0, 0.4),
    ]
}

#[allow(clippy::too_many_lines)] // the grid
pub fn emit() -> Module {
    let mut m = Module::new("b2_ray");
    let shapes = catalog();
    let rots = [0.0, FRAC_PI_4, FRAC_PI_2, 0.3];
    // Eight approach directions as rational unit vectors, and the ray's perpendicular offsets.
    let dirs: [[f64; 2]; 8] = [
        [1.0, 0.0],
        [0.0, 1.0],
        [-1.0, 0.0],
        [0.0, -1.0],
        [0.6, 0.8],
        [-0.8, 0.6],
        [-0.6, -0.8],
        [0.8, -0.6],
    ];
    let lateral = [-0.6, -0.2, 0.0, 0.3, 0.9, 2.0];

    for s in &shapes {
        for &rot in &rots {
            for off in [[0.0, 0.0], [0.5, -0.25]] {
                let mut b = body([1.0, -0.5], rot, s);
                b.offset = off;
                let bodies = [b];
                // From outside: start 5 back along -u, shifted sideways, aimed along u.
                for u in dirs {
                    for l in lateral {
                        let o = [1.0 - 5.0 * u[0] - l * u[1], -0.5 - 5.0 * u[1] + l * u[0]];
                        ray(&mut m, &bodies, o, u, 100.0, u32::MAX);
                    }
                }
                // From the body's position (inside most shapes).
                for u in &dirs[..4] {
                    ray(&mut m, &bodies, [1.0, -0.5], *u, 100.0, u32::MAX);
                }
                // The overlap probes around the body.
                for u in &dirs[..4] {
                    for dd in [0.0, 0.5, 1.0, 1.5, 2.5] {
                        let c = [1.0 + u[0] * dd, -0.5 + u[1] * dd];
                        sphere(&mut m, &bodies, c, 0.25);
                        sphere(&mut m, &bodies, c, 0.75);
                        aabb(
                            &mut m,
                            &bodies,
                            [c[0] - 0.25, c[1] - 0.25],
                            [c[0] + 0.25, c[1] + 0.4],
                        );
                    }
                }
            }
        }
    }

    // Ray directions the query refuses or normalises, and the max-distance cut.
    let one = [body([0.0, 0.0], 0.0, &ball(1.0))];
    let nan = f64::NAN;
    let inf = f64::INFINITY;
    for d in [
        [0.0, 0.0],
        [1e-11, 0.0],
        [1e-10, 0.0],
        [3.0, 4.0],
        [-1e-300, 0.0],
        [1e300, 1e300],
        [nan, 1.0],
        [inf, 0.0],
        [0.0, -2.5],
    ] {
        ray(&mut m, &one, [-5.0, 0.0], d, 100.0, u32::MAX);
        ray(&mut m, &one, [0.0, 5.0], d, 100.0, u32::MAX);
    }
    for max in [3.9, 4.0, 4.1, 0.0, -1.0, nan, inf] {
        ray(&mut m, &one, [-5.0, 0.0], [1.0, 0.0], max, u32::MAX);
    }
    // Inside a ball, at its centre, on its rim, behind it.
    for o in [[0.0, 0.0], [1.0, 0.0], [-1.0, 0.0], [2.0, 0.0], [0.5, 0.5]] {
        ray(&mut m, &one, o, [1.0, 0.0], 100.0, u32::MAX);
    }

    // Layers: the ray sees a collider only when its layer shares a bit with the mask.
    for (layer, mask) in [
        (u32::MAX, u32::MAX),
        (1, 2),
        (3, 2),
        (0, u32::MAX),
        (1 << 31, 1 << 31),
    ] {
        let mut b = body([0.0, 0.0], 0.0, &bx(1.0, 1.0));
        b.layer = layer;
        ray(&mut m, &[b], [-5.0, 0.0], [1.0, 0.0], 100.0, mask);
    }

    // Several colliders: the nearest wins, the first of equals, the cut applies before the choice.
    let row3 = [
        body([4.0, 0.0], 0.0, &ball(0.5)),
        body([2.0, 0.0], 0.0, &bx(0.5, 0.5)),
        body([6.0, 0.0], FRAC_PI_2, &capsule(0.5, 0.25)),
    ];
    for max in [100.0, 1.5, 1.4, 3.6] {
        ray(&mut m, &row3, [0.0, 0.0], [1.0, 0.0], max, u32::MAX);
        ray(&mut m, &row3, [8.0, 0.0], [-1.0, 0.0], max, u32::MAX);
    }
    let twins = [
        body([3.0, 0.0], 0.0, &ball(0.5)),
        body([3.0, 0.0], 0.0, &ball(0.5)),
    ];
    ray(&mut m, &twins, [0.0, 0.0], [1.0, 0.0], 100.0, u32::MAX);
    sphere(&mut m, &row3, [3.0, 0.0], 1.2);
    sphere(&mut m, &twins, [3.0, 0.0], 0.1);
    aabb(&mut m, &row3, [1.0, -1.0], [7.0, 1.0]);
    // Exactly touching boxes (closed intervals), a hair apart, and an inverted probe.
    let unit = [body([0.0, 0.0], 0.0, &bx(1.0, 1.0))];
    for e in [-1e-12, 0.0, 1e-12] {
        aabb(&mut m, &unit, [1.0 + e, -0.5], [2.0, 0.5]);
        sphere(&mut m, &unit, [2.0 + e, 0.0], 1.0);
    }
    aabb(&mut m, &unit, [0.5, 0.5], [-0.5, -0.5]);

    // The Rust tests' scenes: a box at (5, 0), a ball, a capsule.
    let scenes: Vec<Vec<Body>> = vec![
        vec![body([5.0, 0.0], 0.0, &bx(1.0, 1.0))],
        vec![body([5.0, 0.0], 0.0, &ball(1.0))],
        vec![body([0.0, 0.0], 0.0, &capsule(1.0, 0.5))],
        vec![body([0.0, 0.0], 0.0, &segment([-1.0, 0.0], [1.0, 0.0]))],
        vec![body(
            [0.0, 0.0],
            0.0,
            &hull(&[[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]]),
        )],
    ];
    for sc in &scenes {
        ray(&mut m, sc, [0.0, 0.0], [1.0, 0.0], 100.0, u32::MAX);
        ray(&mut m, sc, [0.0, 5.0], [0.0, -1.0], 100.0, u32::MAX);
        ray(&mut m, sc, [-5.0, 0.0], [1.0, 0.0], 100.0, u32::MAX);
        ray(&mut m, sc, [0.0, 0.0], [0.0, 1.0], 100.0, u32::MAX);
        sphere(&mut m, sc, [5.0, 0.5], 0.6);
        aabb(&mut m, sc, [4.0, -2.0], [6.0, 2.0]);
    }

    // Seeded worlds of one to three colliders, rays and probes through them.
    let mut rng = Rng::new(0x7a1c);
    let pi = std::f64::consts::PI;
    for k in 0..1800 {
        let n = rng.int(1, 3);
        let bodies: Vec<Body> = (0..n)
            .map(|_| {
                let shape = random_shape(&mut rng);
                let mut b = body(
                    [rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)],
                    rng.uniform(-pi, pi),
                    &shape,
                );
                b.offset = [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)];
                b
            })
            .collect();
        let o = [rng.uniform(-6.0, 6.0), rng.uniform(-6.0, 6.0)];
        let target = [rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)];
        let scale = rng.uniform(0.2, 3.0);
        let d = [(target[0] - o[0]) * scale, (target[1] - o[1]) * scale];
        ray(&mut m, &bodies, o, d, rng.uniform(1.0, 20.0), u32::MAX);
        if k % 3 == 0 {
            sphere(&mut m, &bodies, target, rng.uniform(0.05, 2.0));
        } else if k % 3 == 1 {
            let h = [rng.uniform(0.05, 1.5), rng.uniform(0.05, 1.5)];
            aabb(
                &mut m,
                &bodies,
                [target[0] - h[0], target[1] - h[1]],
                [target[0] + h[0], target[1] + h[1]],
            );
        }
    }
    // Big hulls (the port's polygon paths take freelist storage past 16 vertices).
    for h in [
        ellipse_hull(24, 1.2, 0.8, true),
        ellipse_hull(40, 0.9, 1.1, false),
    ] {
        let bodies = [body([0.5, 0.25], 0.3, &h)];
        for u in dirs {
            ray(
                &mut m,
                &bodies,
                [0.5 - 4.0 * u[0], 0.25 - 4.0 * u[1]],
                u,
                100.0,
                u32::MAX,
            );
        }
        ray(&mut m, &bodies, [0.5, 0.25], [1.0, 0.0], 100.0, u32::MAX);
        sphere(&mut m, &bodies, [1.5, 0.5], 0.4);
        aabb(&mut m, &bodies, [1.0, 0.0], [2.0, 1.0]);
    }
    m
}
