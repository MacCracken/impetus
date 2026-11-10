//! `b3_body.tsv` — the 3D backend's bodies as `add_body` and `add_collider` build them (M4): `RigidBody3d::from_desc`
//! and the mass properties, read back from the oracle's world dump.
//!
//! Each row builds a 3D world, adds one body from a full `BodyDesc`, attaches its colliders in order through the
//! public `add_collider` (each recomputes the body's mass from all of them, as state.rs's
//! `recompute_mass_properties` does), and reads the body from `oracle_dump()`. That pins `from_desc` (the rotation
//! about z through `DQuat::from_rotation_z`), `Collider3d::compute_mass`, `compute_inertia` and `local_centroid`, the
//! parallel-axis sum, the inverses and the world inverse inertia R·I⁻¹·Rᵀ.
//!
//! Inputs: the body descriptor (`flat::body_desc`), `v:<colliders>`, then per collider its shape (`flat::shape`), the
//! offset x, y and z, the density, and `n:` or the explicit mass. Outputs: every stored field of the body but its
//! handle and type (`Body3dDump`), matrices by columns. Class C: crmath's `sin_cos` (ADR 0011) and + − × ÷ √.

use impetus::{
    BodyDesc, BodyType, ColliderDesc, ColliderShape, PhysicsMaterial, PhysicsWorld, WorldConfig,
};

use crate::flat;
use crate::grids::Rng;
use crate::tsv::{Class, Module, Tok, b, f};

struct Coll {
    shape: ColliderShape,
    offset: [f64; 3],
    density: f64,
    mass: Option<f64>,
}

fn c(shape: ColliderShape) -> Coll {
    Coll {
        shape,
        offset: [0.0; 3],
        density: 1.0,
        mass: None,
    }
}

fn at(shape: ColliderShape, offset: [f64; 3]) -> Coll {
    Coll { offset, ..c(shape) }
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

fn hull(points: &[[f64; 3]]) -> ColliderShape {
    ColliderShape::ConvexHull {
        points: points.to_vec(),
    }
}

/// The eight corners of the box [lo, hi].
fn corners(lo: [f64; 3], hi: [f64; 3]) -> Vec<[f64; 3]> {
    let mut v = Vec::new();
    for k in 0..8 {
        v.push([
            if k & 1 == 0 { lo[0] } else { hi[0] },
            if k & 2 == 0 { lo[1] } else { hi[1] },
            if k & 4 == 0 { lo[2] } else { hi[2] },
        ]);
    }
    v
}

fn mesh(vertices: Vec<[f64; 3]>) -> ColliderShape {
    let n = vertices.len() as u32;
    let indices = (0..n.saturating_sub(2)).map(|k| [0, k + 1, k + 2]).collect();
    ColliderShape::TriMesh { vertices, indices }
}

fn row(m: &mut Module, desc: &BodyDesc, colls: &[Coll]) {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let body = world.add_body(desc.clone());
    let mut ins = flat::body_desc(desc);
    ins.push(Tok::V(colls.len()));
    for k in colls {
        world.add_collider(
            body,
            ColliderDesc {
                shape: k.shape.clone(),
                offset: k.offset,
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
        ins.extend(k.offset.iter().map(|&x| f(x)));
        ins.extend([f(k.density), k.mass.map_or(Tok::None, f)]);
    }
    let dump = world.oracle_dump();
    let bd = &dump.backend.bodies[0];
    let mut out: Vec<Tok> = Vec::new();
    let v = |o: &mut Vec<Tok>, x: &[f64]| o.extend(x.iter().map(|&y| f(y)));
    v(&mut out, &bd.position);
    v(&mut out, &bd.rotation);
    v(&mut out, &bd.linear_velocity);
    v(&mut out, &bd.angular_velocity);
    out.extend([f(bd.linear_damping), f(bd.angular_damping), b(bd.fixed_rotation), f(bd.gravity_scale)]);
    v(&mut out, &bd.force_accumulator);
    v(&mut out, &bd.torque_accumulator);
    out.extend([f(bd.mass), f(bd.inv_mass)]);
    for mat in [&bd.inertia, &bd.inv_inertia, &bd.inv_inertia_world] {
        for col in mat.iter() {
            v(&mut out, col);
        }
    }
    v(&mut out, &bd.pseudo_velocity);
    v(&mut out, &bd.pseudo_angular_velocity);
    out.extend([b(bd.is_sleeping), f(bd.sleep_timer)]);
    m.row("b3::body", Class::C, &ins, &out);
}

fn dynamic() -> BodyDesc {
    BodyDesc {
        body_type: BodyType::Dynamic,
        ..BodyDesc::default()
    }
}

fn posed(rotation: f64, fixed: bool) -> BodyDesc {
    BodyDesc {
        rotation,
        fixed_rotation: fixed,
        ..dynamic()
    }
}

/// A seeded shape: one of the seven kinds, sizes log-uniform over about [0.03, 4).
fn random_shape(rng: &mut Rng) -> ColliderShape {
    let kind = rng.int(0, 6);
    let mut s = || rng.log_uniform(-1.3, 0.7);
    match kind {
        0 => ball(s()),
        1 => bx(s(), s(), s()),
        2 => capsule(s(), s()),
        3 => {
            let (a, bb, cc) = (s(), s(), s());
            hull(&corners([-a, -bb, -cc], [a * 0.5, bb, cc * 2.0]))
        }
        4 => {
            let (a, bb) = (s(), s());
            mesh(vec![[0.0, 0.0, 0.0], [a, 0.0, 0.0], [0.0, bb, 0.0], [0.0, 0.0, a + bb]])
        }
        5 => ColliderShape::Segment {
            a: [-s(), 0.0, 0.0],
            b: [s(), s(), 0.0],
        },
        _ => ColliderShape::Heightfield {
            heights: vec![0.0, s(), -s(), 1.0],
            scale: [s(), 1.0, s()],
        },
    }
}

pub fn emit() -> Module {
    let mut m = Module::new("b3_body");
    let d = dynamic();

    // ── every shape alone, at the body origin ──
    for r in [0.1, 0.5, 1.0, 2.5, 0.0] {
        row(&mut m, &d, &[c(ball(r))]);
    }
    for he in [[0.5, 0.5, 0.5], [1.0, 2.0, 3.0], [0.05, 4.0, 0.3], [1.0, 1.0, 0.0]] {
        row(&mut m, &d, &[c(bx(he[0], he[1], he[2]))]);
    }
    for (hh, r) in [(0.5, 0.25), (1.0, 1.0), (0.0, 0.5), (2.0, 0.0), (0.0, 0.0), (3.0, 0.1)] {
        row(&mut m, &d, &[c(capsule(hh, r))]);
    }
    let cube = corners([-0.5; 3], [0.5; 3]);
    row(&mut m, &d, &[c(hull(&cube))]);
    row(&mut m, &d, &[c(hull(&corners([0.0, 1.0, -2.0], [3.0, 1.5, 2.0])))]);
    // A tetrahedron off the origin (centroid = its AABB centre), a flat square (zero volume: the 1 m³ fallback, no
    // centroid shift), one point, and no points.
    row(&mut m, &d, &[c(hull(&[[1.0, 1.0, 1.0], [2.0, 1.0, 1.0], [1.0, 3.0, 1.0], [1.0, 1.0, 4.0]]))]);
    row(&mut m, &d, &[c(hull(&[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]]))]);
    row(&mut m, &d, &[c(hull(&[[2.0, 3.0, 4.0]]))]);
    row(&mut m, &d, &[c(hull(&[]))]);
    row(&mut m, &d, &[c(mesh(cube.clone()))]);
    row(&mut m, &d, &[c(mesh(vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]))]);
    row(
        &mut m,
        &d,
        &[c(ColliderShape::Segment {
            a: [-1.0, 0.0, 0.0],
            b: [1.0, 2.0, 0.5],
        })],
    );
    row(
        &mut m,
        &d,
        &[c(ColliderShape::Heightfield {
            heights: vec![0.0, 1.0, 0.5],
            scale: [1.0, 2.0, 1.0],
        })],
    );

    // ── density and explicit mass ──
    for density in [0.0, 0.25, 1000.0, 1e-300] {
        row(&mut m, &d, &[Coll { density, ..c(bx(0.5, 1.0, 1.5)) }]);
    }
    for mass in [2.0, 0.0, -1.0, 1e-9, 1e12] {
        row(&mut m, &d, &[Coll { mass: Some(mass), ..c(ball(0.5)) }]);
        row(&mut m, &d, &[Coll { mass: Some(mass), ..c(capsule(0.5, 0.25)) }]);
    }

    // ── offsets: the parallel-axis shift, one collider and several ──
    for off in [[1.0, 0.0, 0.0], [0.0, -2.0, 0.0], [1.0, 2.0, -3.0], [0.25, 0.5, 0.125]] {
        row(&mut m, &d, &[at(ball(0.5), off)]);
        row(&mut m, &d, &[at(bx(0.5, 0.25, 1.0), off)]);
        row(&mut m, &d, &[at(capsule(0.5, 0.2), off)]);
        row(&mut m, &d, &[at(hull(&[[1.0, 1.0, 1.0], [2.0, 1.0, 1.0], [1.0, 3.0, 1.0], [1.0, 1.0, 4.0]]), off)]);
    }
    row(&mut m, &d, &[at(ball(0.5), [-1.0, 0.0, 0.0]), at(ball(0.5), [1.0, 0.0, 0.0])]);
    row(
        &mut m,
        &d,
        &[at(bx(1.0, 0.1, 1.0), [0.0, -1.0, 0.0]), at(capsule(0.5, 0.3), [0.0, 1.0, 0.5]), at(ball(0.2), [2.0, 2.0, 2.0])],
    );
    // Symmetric about the origin: the products of inertia cancel to the bit, or not, in the sum's order.
    row(
        &mut m,
        &d,
        &[at(ball(0.3), [1.0, 1.0, 0.0]), at(ball(0.3), [-1.0, -1.0, 0.0]), at(ball(0.3), [1.0, -1.0, 0.0])],
    );

    // ── body types, fixed rotation, poses and the other descriptor fields ──
    for bt in [BodyType::Static, BodyType::Kinematic] {
        row(&mut m, &BodyDesc { body_type: bt, ..d.clone() }, &[c(ball(1.0))]);
        row(&mut m, &BodyDesc { body_type: bt, ..d.clone() }, &[]);
    }
    row(&mut m, &d, &[]);
    row(&mut m, &posed(0.0, true), &[c(bx(1.0, 2.0, 3.0))]);
    for rot in [0.3, std::f64::consts::FRAC_PI_2, -2.0, std::f64::consts::PI, 10.0, -0.0] {
        row(&mut m, &posed(rot, false), &[c(bx(1.0, 2.0, 3.0))]);
        row(&mut m, &posed(rot, false), &[at(capsule(0.5, 0.25), [1.0, 0.5, -0.25])]);
        row(&mut m, &posed(rot, true), &[c(bx(1.0, 2.0, 3.0))]);
    }
    row(
        &mut m,
        &BodyDesc {
            body_type: BodyType::Dynamic,
            position: [1.0, -2.0, 3.5],
            rotation: 0.75,
            linear_velocity: [3.0, 0.5, -1.0],
            angular_velocity: -2.0,
            linear_damping: 0.1,
            angular_damping: 0.05,
            fixed_rotation: false,
            gravity_scale: Some(0.5),
        },
        &[c(bx(0.5, 0.5, 0.5))],
    );
    row(&mut m, &BodyDesc { gravity_scale: Some(-1.0), ..d.clone() }, &[c(ball(1.0))]);

    // ── seeded bodies: 1–3 random colliders with random offsets on a random pose ──
    let mut rng = Rng::new(0xb3b0_0001);
    for _ in 0..150 {
        let desc = BodyDesc {
            body_type: BodyType::Dynamic,
            position: [rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)],
            rotation: rng.uniform(-4.0, 4.0),
            linear_velocity: [rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0)],
            angular_velocity: rng.uniform(-5.0, 5.0),
            linear_damping: rng.uniform(0.0, 1.0),
            angular_damping: rng.uniform(0.0, 1.0),
            fixed_rotation: rng.int(0, 9) == 0,
            gravity_scale: if rng.int(0, 1) == 0 { None } else { Some(rng.uniform(-2.0, 2.0)) },
        };
        let n = rng.int(1, 3);
        let colls: Vec<Coll> = (0..n)
            .map(|_| {
                let shape = random_shape(&mut rng);
                let offset = [rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0)];
                let density = rng.log_uniform(-1.0, 3.0);
                let mass = if rng.int(0, 4) == 0 { Some(rng.log_uniform(-1.0, 2.0)) } else { None };
                Coll { shape, offset, density, mass }
            })
            .collect();
        row(&mut m, &desc, &colls);
    }
    m
}
