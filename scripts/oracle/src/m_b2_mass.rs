//! `b2_mass.tsv` — the 2D backend's mass properties (M2b), read from the oracle's world dump.
//!
//! Each row builds a world with one body (its type, `fixed_rotation`), attaches its colliders in
//! order through the public `add_collider`, and reads the body back from `oracle_dump()`: mass,
//! inertia (about the body origin, every collider shifted there by the parallel-axis theorem), and
//! their inverses. That pins `Collider2d::mass_properties` (`shape_props`, `polygon_mass_props`,
//! the MIN_MASS / MIN_INERTIA floors), state.rs's accumulation and `refresh_inverse_mass`.
//!
//! Inputs: `e:BodyType`, `fixed_rotation`, `v:<colliders>`, then per collider its shape's tokens
//! (`flat::shape`), offset x and y, density, and `n:` or the explicit mass. Outputs: mass, inertia,
//! inv_mass, inv_inertia. Class B: + − × ÷ √ only.

use impetus::{
    BodyDesc, BodyType, ColliderDesc, ColliderShape, PhysicsMaterial, PhysicsWorld, WorldConfig,
};

use crate::flat;
use crate::grids::Rng;
use crate::tsv::{Class, Module, Tok, b, e, f};

struct Coll {
    shape: ColliderShape,
    offset: [f64; 2],
    density: f64,
    mass: Option<f64>,
}

fn c(shape: ColliderShape) -> Coll {
    Coll {
        shape,
        offset: [0.0, 0.0],
        density: 1.0,
        mass: None,
    }
}

fn row(m: &mut Module, body_type: BodyType, fixed: bool, colls: &[Coll]) {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let body = world.add_body(BodyDesc {
        body_type,
        fixed_rotation: fixed,
        ..BodyDesc::default()
    });
    let mut ins = vec![e("BodyType", body_type), b(fixed), Tok::V(colls.len())];
    for k in colls {
        world.add_collider(
            body,
            ColliderDesc {
                shape: k.shape.clone(),
                offset: [k.offset[0], k.offset[1], 0.0],
                material: PhysicsMaterial {
                    density: k.density,
                    ..PhysicsMaterial::default()
                },
                is_sensor: false,
                mass: k.mass,
                collision_layer: u32::MAX,
                collision_mask: u32::MAX,
            },
        );
        ins.extend(flat::shape(&k.shape));
        ins.extend([f(k.offset[0]), f(k.offset[1]), f(k.density)]);
        ins.push(k.mass.map_or(Tok::None, f));
    }
    let dump = world.oracle_dump();
    let bd = &dump.backend.bodies[0];
    m.row(
        "b2::body_mass",
        Class::B,
        &ins,
        &[f(bd.mass), f(bd.inertia), f(bd.inv_mass), f(bd.inv_inertia)],
    );
}

fn hull(pts: &[[f64; 2]]) -> ColliderShape {
    ColliderShape::ConvexHull {
        points: pts.iter().map(|p| [p[0], p[1], 0.0]).collect(),
    }
}

/// A convex polygon with `n` vertices on an ellipse, counter-clockwise or clockwise, centred at c.
fn polygon(rng: &mut Rng, n: u32, ccw: bool) -> ColliderShape {
    let (cx, cy) = (rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0));
    let (rx, ry) = (rng.uniform(0.2, 3.0), rng.uniform(0.2, 3.0));
    // Vertices from exact rational points on the unit circle ((1 − t²)/(1 + t²), 2t/(1 + t²)),
    // so the grid needs no transcendental.
    let mut pts: Vec<[f64; 2]> = (0..n)
        .map(|k| {
            let t = -1.0 + 2.0 * f64::from(k) / f64::from(n);
            let d = 1.0 + t * t;
            [cx + rx * (1.0 - t * t) / d, cy + ry * (2.0 * t) / d]
        })
        .collect();
    if !ccw {
        pts.reverse();
    }
    hull(&pts)
}

#[allow(clippy::too_many_lines)] // the shape grid
pub fn emit() -> Module {
    let mut m = Module::new("b2_mass");
    let mut rng = Rng::new(0xb2a5);
    let d = BodyType::Dynamic;

    // One collider of each shape, the edge sizes, both signs.
    for r in [1.0, 0.5, 1e-3, 1e3, 0.0, -1.0, f64::NAN] {
        row(&mut m, d, false, &[c(ColliderShape::Ball { radius: r })]);
    }
    for he in [[1.0, 1.0], [2.0, 0.5], [0.0, 1.0], [-1.0, 2.0], [1e-4, 1e4]] {
        row(
            &mut m,
            d,
            false,
            &[c(ColliderShape::Box {
                half_extents: [he[0], he[1], 0.0],
            })],
        );
    }
    for (hh, r) in [
        (1.0, 0.5),
        (0.0, 1.0),
        (2.0, 0.0),
        (0.0, 0.0),
        (-1.0, -0.5),
        (1e3, 1e-3),
    ] {
        row(
            &mut m,
            d,
            false,
            &[c(ColliderShape::Capsule {
                half_height: hh,
                radius: r,
            })],
        );
    }
    for (a, bb) in [
        ([0.0, 0.0], [10.0, 0.0]),
        ([-1.0, 2.0], [3.0, -4.0]),
        ([1.0, 1.0], [1.0, 1.0]),
    ] {
        row(
            &mut m,
            d,
            false,
            &[c(ColliderShape::Segment {
                a: [a[0], a[1], 0.0],
                b: [bb[0], bb[1], 0.0],
            })],
        );
    }
    // Hulls: the Rust test's square in both windings, a triangle, an off-centre polygon, the
    // degenerate ones (collinear, two points, empty) that fall back to (1, 0, 1).
    let square = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]];
    let mut square_cw = square;
    square_cw.reverse();
    for pts in [
        &square[..],
        &square_cw[..],
        &[[0.0, 0.0], [3.0, 0.0], [0.0, 4.0]][..],
        &[[5.0, 5.0], [6.0, 5.0], [6.5, 6.0], [5.5, 7.0], [4.8, 6.0]][..],
        &[[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]][..],
        &[[0.0, 0.0], [1.0, 0.0]][..],
        &[][..],
    ] {
        row(&mut m, d, false, &[c(hull(pts))]);
    }
    for n in [3, 5, 8, 16] {
        for ccw in [true, false] {
            let shape = polygon(&mut rng, n, ccw);
            row(&mut m, d, false, &[c(shape)]);
        }
    }
    // Triangle meshes: one triangle, a quad of two, an out-of-range index, none.
    let quad = vec![
        [0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0],
        [2.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
    ];
    for (verts, idx) in [
        (quad.clone(), vec![[0, 1, 2]]),
        (quad.clone(), vec![[0, 1, 2], [0, 2, 3]]),
        (quad.clone(), vec![[0, 1, 2], [0, 2, 9]]),
        (quad.clone(), vec![]),
    ] {
        row(
            &mut m,
            d,
            false,
            &[c(ColliderShape::TriMesh {
                vertices: verts,
                indices: idx,
            })],
        );
    }
    row(
        &mut m,
        d,
        false,
        &[c(ColliderShape::Heightfield {
            heights: vec![0.0, 1.0, 0.5],
            scale: [1.0, 1.0, 1.0],
        })],
    );

    // Offsets (the parallel-axis shift), densities, explicit masses and their floors.
    for off in [[3.0, 0.0], [1.0, -2.0], [-0.5, 0.25]] {
        for shape in [
            ColliderShape::Ball { radius: 0.5 },
            ColliderShape::Box {
                half_extents: [1.0, 0.5, 0.0],
            },
            hull(&square),
        ] {
            row(
                &mut m,
                d,
                false,
                &[Coll {
                    shape,
                    offset: off,
                    density: 1.0,
                    mass: None,
                }],
            );
        }
    }
    for (density, mass) in [
        (0.5, None),
        (0.0, None),
        (1e6, None),
        (1.0, Some(42.0)),
        (1.0, Some(0.0)),
        (1.0, Some(-5.0)),
        (1.0, Some(f64::NAN)),
        (1.0, Some(1e-9)),
    ] {
        row(
            &mut m,
            d,
            false,
            &[Coll {
                shape: ColliderShape::Ball { radius: 1.0 },
                offset: [0.0, 0.0],
                density,
                mass,
            }],
        );
    }

    // Body types and fixed rotation: the inverses.
    for (bt, fixed) in [
        (BodyType::Static, false),
        (BodyType::Kinematic, false),
        (BodyType::Dynamic, true),
    ] {
        row(
            &mut m,
            bt,
            fixed,
            &[c(ColliderShape::Box {
                half_extents: [1.0, 1.0, 0.0],
            })],
        );
    }
    row(&mut m, d, false, &[]);

    // Several colliders: the sum, in attachment order.
    row(
        &mut m,
        d,
        false,
        &[
            c(ColliderShape::Ball { radius: 1.0 }),
            Coll {
                shape: ColliderShape::Box {
                    half_extents: [0.5, 0.5, 0.0],
                },
                offset: [2.0, 0.0],
                density: 2.0,
                mass: None,
            },
            Coll {
                shape: ColliderShape::Capsule {
                    half_height: 0.5,
                    radius: 0.25,
                },
                offset: [0.0, -1.5],
                density: 1.0,
                mass: Some(3.0),
            },
        ],
    );
    for _ in 0..12 {
        let n = u32::try_from(rng.int(2, 4)).unwrap_or(2);
        let colls: Vec<Coll> = (0..n)
            .map(|k| {
                let shape = match k % 3 {
                    0 => ColliderShape::Ball {
                        radius: rng.uniform(0.1, 2.0),
                    },
                    1 => ColliderShape::Box {
                        half_extents: [rng.uniform(0.1, 2.0), rng.uniform(0.1, 2.0), 0.0],
                    },
                    _ => polygon(&mut rng, 6, true),
                };
                Coll {
                    shape,
                    offset: [rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)],
                    density: rng.log_uniform(-1.0, 2.0),
                    mass: None,
                }
            })
            .collect();
        row(&mut m, d, false, &colls);
    }
    m
}
