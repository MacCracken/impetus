//! `b3_ray.tsv` — the 3D raycast (M4 part 3), through the public world API (`PhysicsState3d::raycast_impl` and the
//! ray helpers of `backend_3d/raycast.rs`).
//!
//! Each row builds a 3D world of static bodies, one collider each, and casts one ray. Inputs: `v:<bodies>`, then per
//! body its position x, y, z, its rotation about z (`BodyDesc::rotation`), its collider's shape (`flat::shape`), offset
//! x, y, z and collision layer; then the ray's origin x, y, z, direction x, y, z, max distance, and the layer mask
//! (`raycast_filtered`) or `n:` (`raycast`, which filters nothing). Output: `n:`, or the hit's collider handle, point
//! x, y, z, normal x, y, z and distance.
//!
//! Class C: a body's rotation is `DQuat::from_rotation_z`, a `sin_cos`, correctly rounded in the port, so the reference
//! is the corrected oracle (ADR 0011; `validate.py` recomputes every row). The ray math itself is + − × ÷ √.
//!
//! Rust 1.4.0's 3D world has no overlap queries (`overlap_sphere` and `overlap_aabb` return nothing in 3D), so this
//! module is rays only.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use impetus::{BodyDesc, BodyType, ColliderDesc, ColliderShape, PhysicsMaterial, PhysicsWorld, WorldConfig};

use crate::flat;
use crate::grids::Rng;
use crate::tsv::{Class, Module, Tok, f, i};

/// One body and its collider.
#[derive(Clone)]
struct Body {
    pos: [f64; 3],
    rot: f64,
    shape: ColliderShape,
    offset: [f64; 3],
    layer: u32,
}

fn body(pos: [f64; 3], rot: f64, shape: &ColliderShape) -> Body {
    Body {
        pos,
        rot,
        shape: shape.clone(),
        offset: [0.0; 3],
        layer: u32::MAX,
    }
}

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

fn world(bodies: &[Body]) -> (PhysicsWorld, Vec<Tok>) {
    let mut w = PhysicsWorld::new(WorldConfig::default());
    let mut t = vec![Tok::V(bodies.len())];
    for bd in bodies {
        let h = w.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: bd.pos,
            rotation: bd.rot,
            ..BodyDesc::default()
        });
        w.add_collider(
            h,
            ColliderDesc {
                shape: bd.shape.clone(),
                offset: bd.offset,
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: bd.layer,
                collision_mask: u32::MAX,
            },
        );
        t.extend(bd.pos.iter().map(|&x| f(x)));
        t.push(f(bd.rot));
        t.extend(flat::shape(&bd.shape));
        t.extend(bd.offset.iter().map(|&x| f(x)));
        t.push(i(bd.layer));
    }
    (w, t)
}

fn ray(m: &mut Module, bodies: &[Body], o: [f64; 3], d: [f64; 3], max: f64, mask: Option<u32>) {
    let (w, mut ins) = world(bodies);
    ins.extend(o.iter().chain(d.iter()).map(|&x| f(x)));
    ins.push(f(max));
    ins.push(mask.map_or(Tok::None, i));
    let hit = match mask {
        Some(mk) => w.raycast_filtered(o, d, max, mk),
        None => w.raycast(o, d, max),
    };
    let out = match hit {
        None => vec![Tok::None],
        Some(h) => {
            let mut v = vec![Tok::I(h.collider.0.cast_signed())];
            v.extend(h.point.iter().chain(h.normal.iter()).map(|&x| f(x)));
            v.push(f(h.distance));
            v
        }
    };
    m.row("b3::raycast", Class::C, &ins, &out);
}

/// Rays at a body centred at `c`: from each axis at distance 5 toward the centre (unnormalized directions), obliques,
/// a tangent and a near-miss at `reach` off the axis, from inside, parallel to the faces inside and outside the slab,
/// and along y from above; each at max distances 100 and, for the axis rays, 3.9, 4.0 and infinity.
fn ray_fan(m: &mut Module, bodies: &[Body], c: [f64; 3], reach: f64) {
    let at = |dx: f64, dy: f64, dz: f64| [c[0] + dx, c[1] + dy, c[2] + dz];
    for (o, d) in [
        (at(5.0, 0.0, 0.0), [-5.0, 0.0, 0.0]),
        (at(-5.0, 0.0, 0.0), [1.0, 0.0, 0.0]),
        (at(0.0, 5.0, 0.0), [0.0, -2.0, 0.0]),
        (at(0.0, -5.0, 0.0), [0.0, 1.0, 0.0]),
        (at(0.0, 0.0, 5.0), [0.0, 0.0, -1.0]),
        (at(0.0, 0.0, -5.0), [0.0, 0.0, 3.0]),
    ] {
        for max in [100.0, 3.9, 4.0, f64::INFINITY] {
            ray(m, bodies, o, d, max, None);
        }
    }
    for (o, d) in [
        (at(5.0, 3.0, 1.0), [-4.9, -2.8, -1.0]),
        (at(-4.0, 2.0, -3.0), [4.0, -1.9, 3.1]),
        (at(3.0, -4.0, 2.5), [-3.0, 4.0, -2.4]),
        (at(5.0, reach, 0.0), [-1.0, 0.0, 0.0]),
        (at(5.0, reach * 1.000_001, 0.0), [-1.0, 0.0, 0.0]),
        (at(5.0, 0.0, reach * 0.999), [-1.0, 0.0, 0.0]),
        (at(0.0, 0.0, 0.0), [1.0, 0.0, 0.0]),
        (at(0.1, 0.05, -0.02), [0.3, 0.9, -0.2]),
        (at(-5.0, 0.1, 0.0), [1.0, 0.0, 0.0]),
        (at(-5.0, 0.0, 0.1), [1.0, 0.0, 1e-12]),
        (at(0.0, 0.1, 0.0), [0.0, 0.0, 1.0]),
        (at(0.0, 5.0, 0.0), [1e-12, -1.0, 0.0]),
        (at(0.0, -5.0, 0.0), [0.0, -1.0, 0.0]),
        (at(5.0, 0.0, 0.0), [1.0, 0.0, 0.0]),
    ] {
        ray(m, bodies, o, d, 100.0, None);
        ray(m, bodies, o, d, 100.0, Some(u32::MAX));
    }
}

/// A seeded shape: the three the raycast handles, and now and then one it skips.
fn random_shape(rng: &mut Rng) -> ColliderShape {
    let kind = rng.int(0, 7);
    let mut s = || rng.log_uniform(-1.3, 0.7);
    match kind {
        0 | 1 => ball(s()),
        2 | 3 => bx(s(), s(), s()),
        4 | 5 => capsule(s(), s()),
        6 => ColliderShape::Segment {
            a: [-s(), 0.0, 0.0],
            b: [s(), 0.0, 0.0],
        },
        _ => ColliderShape::ConvexHull {
            points: vec![[0.0, 0.0, 0.0], [s(), 0.0, 0.0], [0.0, s(), 0.0], [0.0, 0.0, s()]],
        },
    }
}

#[allow(clippy::too_many_lines)] // the grid
pub fn emit() -> Module {
    let mut m = Module::new("b3_ray");

    // ── one body: every shape kind (the three the raycast handles, degenerate ones, and four it skips) at three
    // poses ──
    let shapes: Vec<(ColliderShape, f64)> = vec![
        (ball(1.0), 1.0),
        (ball(0.0), 0.0),
        (bx(1.0, 0.5, 0.25), 0.5),
        (bx(1.0, 0.0, 1.0), 0.0),
        (capsule(1.0, 0.5), 0.5),
        (capsule(0.0, 0.5), 0.5),
        (capsule(1.0, 0.0), 0.0),
        (
            ColliderShape::ConvexHull {
                points: vec![[-1.0, -1.0, -1.0], [1.0, 1.0, 1.0], [1.0, -1.0, 0.0]],
            },
            1.0,
        ),
        (
            ColliderShape::Segment {
                a: [-1.0, 0.0, 0.0],
                b: [1.0, 0.0, 0.0],
            },
            0.0,
        ),
        (
            ColliderShape::Heightfield {
                heights: vec![0.0, 1.0],
                scale: [1.0, 1.0, 1.0],
            },
            1.0,
        ),
    ];
    let poses: [([f64; 3], f64, [f64; 3]); 4] = [
        ([0.0, 0.0, 0.0], 0.0, [0.0, 0.0, 0.0]),
        ([1.0, -0.5, 2.0], FRAC_PI_4, [0.0, 0.0, 0.0]),
        ([-2.0, 1.0, 0.5], 1.2, [0.3, 0.0, -0.2]),
        ([0.0, 3.0, 0.0], FRAC_PI_2, [0.0, 0.5, 0.0]),
    ];
    for (sh, reach) in &shapes {
        for (pos, rot, off) in &poses {
            let mut b = body(*pos, *rot, sh);
            b.offset = *off;
            ray_fan(&mut m, &[b], *pos, *reach);
        }
    }

    // ── degenerate rays: a zero, tiny, NaN or infinite direction, a non-finite origin, max distance 0 and negative ──
    let one = [body([0.0; 3], 0.0, &ball(1.0))];
    let nan = f64::NAN;
    let inf = f64::INFINITY;
    for (o, d, max) in [
        ([5.0, 0.0, 0.0], [0.0, 0.0, 0.0], 100.0),
        ([5.0, 0.0, 0.0], [-1e-300, 0.0, 0.0], 100.0),
        ([5.0, 0.0, 0.0], [-1e-160, 0.0, 0.0], 100.0),
        ([5.0, 0.0, 0.0], [nan, 0.0, 0.0], 100.0),
        ([5.0, 0.0, 0.0], [-inf, 0.0, 0.0], 100.0),
        ([5.0, 0.0, 0.0], [-1e300, 1e300, 0.0], 100.0),
        ([nan, 0.0, 0.0], [-1.0, 0.0, 0.0], 100.0),
        ([inf, 0.0, 0.0], [-1.0, 0.0, 0.0], 100.0),
        ([5.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 0.0),
        ([1.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 0.0),
        ([5.0, 0.0, 0.0], [-1.0, 0.0, 0.0], -1.0),
        ([5.0, 0.0, 0.0], [-1.0, 0.0, 0.0], nan),
    ] {
        ray(&mut m, &one, o, d, max, None);
    }

    // ── several bodies: the nearest hit, an exact tie (the first collider keeps it), hits behind the origin, and layer
    // masks, filtered and not ──
    let row3 = vec![
        body([3.0, 0.0, 0.0], 0.0, &ball(0.5)),
        body([6.0, 0.0, 0.0], 0.3, &bx(0.5, 0.5, 0.5)),
        body([9.0, 0.0, 0.0], 0.0, &capsule(0.5, 0.25)),
    ];
    let mut rev = row3.clone();
    rev.reverse();
    let twins = vec![body([3.0, 0.0, 0.0], 0.0, &ball(0.5)), body([3.0, 0.0, 0.0], 0.0, &ball(0.5))];
    let behind = vec![body([-3.0, 0.0, 0.0], 0.0, &ball(0.5)), body([4.0, 0.0, 0.0], 0.0, &bx(0.5, 1.0, 1.0))];
    for bodies in [&row3, &rev, &twins, &behind] {
        for max in [100.0, 2.6, 5.5, 8.6] {
            ray(&mut m, bodies, [0.0; 3], [1.0, 0.0, 0.0], max, None);
            ray(&mut m, bodies, [0.0, 0.1, 0.05], [1.0, 0.0, 0.0], max, Some(u32::MAX));
        }
    }
    let mut layered = row3.clone();
    layered[0].layer = 1;
    layered[1].layer = 2;
    layered[2].layer = 0;
    for mask in [Some(0), Some(1), Some(2), Some(3), Some(4), Some(u32::MAX), None] {
        ray(&mut m, &layered, [0.0; 3], [1.0, 0.0, 0.0], 100.0, mask);
        ray(&mut m, &layered, [12.0, 0.0, 0.0], [-1.0, 0.0, 0.0], 100.0, mask);
    }

    // ── seeded scenes: 1–6 bodies, rays mostly aimed near one of them ──
    let mut rng = Rng::new(0xb3d0_0001);
    for _ in 0..2000 {
        let n = rng.int(1, 6);
        let bodies: Vec<Body> = (0..n)
            .map(|_| {
                let shape = random_shape(&mut rng);
                let mut b = body(
                    [rng.uniform(-6.0, 6.0), rng.uniform(-6.0, 6.0), rng.uniform(-6.0, 6.0)],
                    rng.uniform(-4.0, 4.0),
                    &shape,
                );
                if rng.int(0, 2) == 0 {
                    b.offset = [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)];
                }
                if rng.int(0, 4) == 0 {
                    b.layer = 1 << rng.int(0, 3);
                }
                b
            })
            .collect();
        let target = bodies[rng.int(0, n - 1) as usize].pos;
        let o = [rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)];
        let d = [
            target[0] + rng.uniform(-1.0, 1.0) - o[0],
            target[1] + rng.uniform(-1.0, 1.0) - o[1],
            target[2] + rng.uniform(-1.0, 1.0) - o[2],
        ];
        let max = if rng.int(0, 3) == 0 { rng.uniform(0.0, 15.0) } else { 100.0 };
        let mask = match rng.int(0, 3) {
            0 => None,
            1 => Some(u32::MAX),
            _ => Some(1u32 << rng.int(0, 3)),
        };
        ray(&mut m, &bodies, o, d, max, mask);
    }
    m
}
