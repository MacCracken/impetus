//! `b3_contact.tsv` — single-step rows (class S, port plan §5) of the 3D backend with contacts (M5 part 3): the
//! broadphase (both kinds, swept boxes), the narrowphase on every shape family, waking on contact, the shock-propagation
//! order, the contact solver (normal impulses with restitution, friction in two tangent directions with the static
//! cone, rolling resistance) and the split-impulse position pass, with joints where a scene has them.
//!
//! The row format is `b3_step.tsv`'s (`m_b3_step::rows`): the config and the 3D backend dump at frame k in, the dump at
//! k + 1 and the frame's collision events out, fn `b3::step`. Class C (friction's slip and impulse lengths are `hypot`,
//! a body's rotation a `sin_cos`): the corrected oracle (`validate.py` transcribes the step, contacts included, and
//! writes `b3_contact.cr.tsv` where glibc misrounds a call).

use std::f64::consts::FRAC_PI_2;

use impetus::{
    BodyDesc, BodyHandle, BodyType, BroadphaseKind, ColliderDesc, ColliderShape, CombineRule, Force, Impulse,
    JointDesc, JointType, PhysicsMaterial, PhysicsWorld, WorldConfig,
};

use crate::grids::Rng;
use crate::m_b3_step::{Scene, rows, scene};
use crate::tsv::Module;

const S: BodyType = BodyType::Static;
const D: BodyType = BodyType::Dynamic;
const K: BodyType = BodyType::Kinematic;

/// Every layer.
const ALL: u32 = u32::MAX;

/// A collider's settings beyond its shape.
#[derive(Clone)]
struct Coll {
    material: PhysicsMaterial,
    offset: [f64; 3],
    sensor: bool,
    layer: u32,
    mask: u32,
}

impl Coll {
    fn plain() -> Self {
        Self {
            material: PhysicsMaterial::default(),
            offset: [0.0, 0.0, 0.0],
            sensor: false,
            layer: 1,
            mask: ALL,
        }
    }

    fn with(material: PhysicsMaterial) -> Self {
        Self {
            material,
            ..Self::plain()
        }
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

/// A cube hull of half size h (GJK and EPA against everything but a ball).
fn cube_hull(h: f64) -> ColliderShape {
    let mut points = Vec::new();
    for x in [-h, h] {
        for y in [-h, h] {
            for z in [-h, h] {
                points.push([x, y, z]);
            }
        }
    }
    ColliderShape::ConvexHull { points }
}

/// A body of type t at `pos`, rotated by `rot` about z, moving at `vel`, with one collider.
fn body(
    w: &mut PhysicsWorld,
    t: BodyType,
    pos: [f64; 3],
    rot: f64,
    vel: [f64; 3],
    shape: ColliderShape,
    c: &Coll,
) -> BodyHandle {
    let h = w.add_body(BodyDesc {
        body_type: t,
        position: pos,
        rotation: rot,
        linear_velocity: vel,
        ..BodyDesc::default()
    });
    attach(w, h, shape, c);
    h
}

fn attach(w: &mut PhysicsWorld, h: BodyHandle, shape: ColliderShape, c: &Coll) {
    w.add_collider(
        h,
        ColliderDesc {
            shape,
            offset: c.offset,
            material: c.material.clone(),
            is_sensor: c.sensor,
            mass: None,
            collision_layer: c.layer,
            collision_mask: c.mask,
        },
    );
}

/// A static ground box, its top at y = 0, `half` wide in x and z.
fn ground(w: &mut PhysicsWorld, half: f64, c: &Coll) -> BodyHandle {
    body(w, S, [0.0, -0.5, 0.0], 0.0, [0.0, 0.0, 0.0], bx(half, 0.5, half), c)
}

/// The frames a contact scene is sampled at: the first contacts, impact, settling, rest and sleep.
const FRAMES: [u64; 9] = [0, 1, 2, 5, 10, 30, 60, 120, 240];

fn material(friction: f64, restitution: f64) -> PhysicsMaterial {
    PhysicsMaterial {
        friction,
        restitution,
        ..PhysicsMaterial::default()
    }
}

fn world(cfg: &WorldConfig) -> PhysicsWorld {
    PhysicsWorld::new(cfg.clone())
}

#[allow(clippy::too_many_lines)] // the scenes
pub fn emit() -> Module {
    let mut m = Module::new("b3_contact");
    let base = WorldConfig::default();
    let p = Coll::plain();
    let z = [0.0, 0.0, 0.0];

    // For each broadphase: a ball dropped on the ground, a box, a box set down at an angle (one edge, then a face),
    // a cube hull (GJK + EPA), and a capsule lying down.
    for kind in [BroadphaseKind::SpatialHash, BroadphaseKind::AabbTree] {
        let mut cfg = base.clone();
        cfg.broadphase = kind;
        for (shape, pos, rot) in [
            (ball(0.5), [0.0, 2.0, 0.0], 0.0),
            (bx(0.5, 0.5, 0.5), [0.0, 1.0, 0.0], 0.0),
            (bx(0.6, 0.3, 0.4), [0.0, 1.2, 0.0], 0.3),
            (cube_hull(0.4), [0.2, 1.0, -0.1], 0.2),
            (capsule(0.6, 0.25), [0.0, 0.6, 0.0], FRAC_PI_2),
        ] {
            let mut w = world(&cfg);
            ground(&mut w, 10.0, &p);
            body(&mut w, D, pos, rot, z, shape, &p);
            rows(&mut m, scene(w), &FRAMES);
        }
    }

    // A stack of five boxes and a pyramid of six (shock propagation's levels).
    {
        let mut w = world(&base);
        ground(&mut w, 10.0, &p);
        for i in 0..5 {
            let y = 0.5 + 1.0 * f64::from(i);
            body(&mut w, D, [0.0, y, 0.0], 0.0, z, bx(0.5, 0.5, 0.5), &p);
        }
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = world(&base);
        ground(&mut w, 10.0, &p);
        for (row, n) in [(0, 3), (1, 2), (2, 1)] {
            for i in 0..n {
                let x = -f64::from(n - 1) * 0.55 + 1.1 * f64::from(i);
                let y = 0.5 + 1.0 * f64::from(row);
                body(&mut w, D, [x, y, 0.0], 0.0, z, bx(0.5, 0.5, 0.5), &p);
            }
        }
        rows(&mut m, scene(w), &FRAMES);
    }

    // Restitution: bouncy, rubber and dead balls dropped fast; the Max, Multiply and Min combine rules.
    {
        let mut w = world(&base);
        ground(&mut w, 10.0, &p);
        for (i, e) in [0.9, 0.6, 0.0].into_iter().enumerate() {
            let x = -3.0 + 3.0 * f64::from(u8::try_from(i).unwrap_or(0));
            body(&mut w, D, [x, 1.5, 0.0], 0.0, [0.0, -8.0, 0.0], ball(0.5), &Coll::with(material(0.5, e)));
        }
        rows(&mut m, scene(w), &[0, 1, 2, 3, 5, 10, 30]);
    }
    for rule in [CombineRule::Max, CombineRule::Multiply, CombineRule::Min] {
        let mut w = world(&base);
        let mut g = material(0.8, 0.2);
        g.restitution_combine = rule;
        g.friction_combine = rule;
        ground(&mut w, 10.0, &Coll::with(g));
        let mut b = material(0.3, 0.8);
        b.restitution_combine = CombineRule::Average;
        body(&mut w, D, [0.0, 1.5, 0.0], 0.0, [2.0, -6.0, 0.5], bx(0.4, 0.4, 0.4), &Coll::with(b));
        rows(&mut m, scene(w), &[0, 1, 2, 3, 10, 30]);
    }

    // Friction: a box sliding down a slope (a static box tilted about z), on ice, with a static friction override; a
    // box shoved along flat ground, a ball spun across it; rolling resistance on a rolling ball.
    for (friction, sf) in [(0.4, None), (0.02, None), (0.2, Some(0.9))] {
        let mut w = world(&base);
        let mut mat = material(friction, 0.0);
        mat.static_friction = sf;
        let c = Coll::with(mat);
        body(&mut w, S, [0.0, 0.0, 0.0], 0.35, z, bx(6.0, 0.5, 3.0), &c);
        body(&mut w, D, [0.0, 1.05, 0.0], 0.35, z, bx(0.4, 0.4, 0.4), &c);
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = world(&base);
        ground(&mut w, 20.0, &Coll::with(material(0.6, 0.0)));
        body(&mut w, D, [-3.0, 0.5, 0.0], 0.0, [6.0, 0.0, 1.5], bx(0.5, 0.5, 0.5), &Coll::with(material(0.6, 0.0)));
        let b = w.add_body(BodyDesc {
            position: [3.0, 0.5, 2.0],
            linear_velocity: [0.0, 0.0, -2.0],
            angular_velocity: 6.0,
            ..BodyDesc::default()
        });
        attach(&mut w, b, ball(0.5), &Coll::with(material(0.6, 0.0)));
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = world(&base);
        let mut g = material(0.6, 0.0);
        g.rolling_friction = 0.1;
        ground(&mut w, 20.0, &Coll::with(g.clone()));
        let b = body(&mut w, D, [0.0, 0.5, 0.0], 0.0, [4.0, 0.0, 0.0], ball(0.5), &Coll::with(g));
        w.apply_impulse(b, Impulse::at_point(0.0, 0.0, 0.5, 0.0, 0.5, 0.0));
        rows(&mut m, scene(w), &FRAMES);
    }

    // Shapes on the ground: a capsule standing, two capsules crossing, a ball on a segment ground, a ball on a hull,
    // a box against a capsule, a collider offset from its body, a tumbling box (an edge and a corner first).
    {
        let mut w = world(&base);
        ground(&mut w, 10.0, &p);
        body(&mut w, D, [-4.0, 0.9, 0.0], 0.0, z, capsule(0.5, 0.3), &p);
        body(&mut w, D, [0.0, 0.3, 0.0], FRAC_PI_2, z, capsule(0.8, 0.3), &p);
        let cross = w.add_body(BodyDesc {
            position: [0.0, 1.0, 0.0],
            ..BodyDesc::default()
        });
        attach(&mut w, cross, capsule(0.8, 0.25), &p);
        w.apply_impulse(cross, Impulse::at_point(0.0, 0.0, 0.3, 0.6, 0.0, 0.0));
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = world(&base);
        body(
            &mut w,
            S,
            [0.0, 0.0, 0.0],
            0.0,
            z,
            ColliderShape::Segment {
                a: [-3.0, 0.0, 0.0],
                b: [3.0, 0.0, 0.0],
            },
            &p,
        );
        body(&mut w, D, [0.5, 0.8, 0.0], 0.0, z, ball(0.4), &p);
        body(&mut w, S, [5.0, 0.0, 0.0], 0.1, z, cube_hull(0.5), &p);
        body(&mut w, D, [5.1, 1.2, 0.05], 0.0, z, ball(0.3), &p);
        body(&mut w, S, [-5.0, 0.3, 0.0], FRAC_PI_2, z, capsule(1.0, 0.3), &p);
        body(&mut w, D, [-5.0, 1.0, 0.0], 0.2, z, bx(0.4, 0.3, 0.3), &p);
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = world(&base);
        ground(&mut w, 10.0, &p);
        let off = Coll {
            offset: [0.5, 0.0, 0.3],
            ..p.clone()
        };
        body(&mut w, D, [0.0, 1.0, 0.0], 0.4, z, bx(0.3, 0.3, 0.3), &off);
        let t = body(&mut w, D, [3.0, 1.5, 0.0], 0.5, z, bx(0.6, 0.3, 0.4), &p);
        w.apply_impulse(t, Impulse::at_point(0.0, 0.0, 2.0, 0.3, 0.2, 0.0));
        w.apply_impulse(t, Impulse::at_point(1.0, 0.0, 0.0, 0.0, 0.0, 0.3));
        rows(&mut m, scene(w), &FRAMES);
    }

    // Sensors, layers and kinematic bodies: a ball falling through a sensor onto the ground, a ball on layer 2 with
    // mask 2 falling through a ground on layer 1 with mask 1 (neither layer meets the other's mask), a kinematic paddle
    // sweeping boxes, two sensors overlapping (no pair).
    {
        let mut w = world(&base);
        let own = Coll {
            mask: 1,
            ..p.clone()
        };
        ground(&mut w, 10.0, &own);
        let sensor = Coll {
            sensor: true,
            ..p.clone()
        };
        body(&mut w, S, [0.0, 1.5, 0.0], 0.0, z, bx(1.0, 0.25, 1.0), &sensor);
        body(&mut w, D, [0.0, 3.0, 0.0], 0.0, z, ball(0.3), &p);
        let miss = Coll {
            layer: 2,
            mask: 2,
            ..p.clone()
        };
        body(&mut w, D, [3.0, 1.0, 0.0], 0.0, z, ball(0.3), &miss);
        body(&mut w, D, [-3.0, 3.0, 0.0], 0.0, z, ball(0.3), &sensor);
        body(&mut w, D, [-3.0, 3.4, 0.0], 0.0, z, ball(0.3), &sensor);
        rows(&mut m, scene(w), &[0, 1, 2, 10, 20, 30, 40, 60]);
    }
    {
        let mut w = world(&base);
        ground(&mut w, 10.0, &p);
        let k = w.add_body(BodyDesc {
            body_type: K,
            position: [-3.0, 0.5, 0.0],
            linear_velocity: [2.0, 0.0, 0.0],
            angular_velocity: 0.5,
            ..BodyDesc::default()
        });
        attach(&mut w, k, bx(0.2, 0.5, 1.0), &p);
        for i in 0..3 {
            let x = -1.0 + 1.0 * f64::from(i);
            body(&mut w, D, [x, 0.3, 0.0], 0.0, z, bx(0.3, 0.3, 0.3), &p);
        }
        rows(&mut m, scene(w), &FRAMES);
    }

    // Sleep and wake on contact: a box comes to rest and sleeps; a ball dropped later wakes it.
    {
        let mut w = world(&base);
        ground(&mut w, 10.0, &p);
        body(&mut w, D, [0.0, 0.5, 0.0], 0.0, z, bx(0.5, 0.5, 0.5), &p);
        body(&mut w, D, [0.2, 40.0, 0.1], 0.0, z, ball(0.3), &p);
        rows(&mut m, scene(w), &[0, 1, 10, 29, 30, 31, 60, 120, 160, 170, 175, 180, 240]);
    }

    // The solver settings: Baumgarte (no constraint frequency) at two correction factors, a stiff soft constraint,
    // more slop, more position iterations; four sub-steps; a push every frame.
    for (freq, corr, slop, pos_it, sub) in [
        (0.0, 0.2, 0.01, 1, 1),
        (0.0, 0.8, 0.01, 1, 1),
        (120.0, 0.2, 0.01, 1, 1),
        (30.0, 0.2, 0.05, 1, 1),
        (30.0, 0.2, 0.01, 4, 1),
        (30.0, 0.2, 0.01, 1, 4),
    ] {
        let mut cfg = base.clone();
        cfg.constraint_frequency = freq;
        cfg.position_correction = corr;
        cfg.position_slop = slop;
        cfg.position_iterations = pos_it;
        cfg.sub_steps = sub;
        let mut w = world(&cfg);
        ground(&mut w, 10.0, &p);
        body(&mut w, D, [0.0, 0.45, 0.0], 0.0, z, bx(0.5, 0.5, 0.5), &p);
        body(&mut w, D, [0.0, 1.4, 0.0], 0.1, z, bx(0.5, 0.5, 0.5), &p);
        rows(&mut m, scene(w), &[0, 1, 2, 10, 30]);
    }
    {
        let mut w = world(&base);
        ground(&mut w, 10.0, &p);
        let b = body(&mut w, D, [0.0, 0.5, 0.0], 0.0, z, bx(0.5, 0.5, 0.5), &p);
        rows(
            &mut m,
            Scene {
                world: w,
                push: Some((b, Force::at_point(30.0, 0.0, 5.0, 0.0, 0.5, 0.0))),
                twist: None,
            },
            &FRAMES,
        );
    }

    // Fast bodies (the swept AABB): a ball shot at a wall, a box falling from high.
    {
        let mut w = world(&base);
        ground(&mut w, 20.0, &p);
        body(&mut w, S, [8.0, 2.0, 0.0], 0.0, z, bx(0.5, 2.0, 2.0), &p);
        body(&mut w, D, [0.0, 1.0, 0.0], 0.0, [90.0, 0.0, 0.0], ball(0.3), &p);
        body(&mut w, D, [-4.0, 30.0, 0.0], 0.3, [0.0, -40.0, 0.0], bx(0.5, 0.5, 0.5), &p);
        rows(&mut m, scene(w), &[0, 1, 2, 3, 4, 5, 10, 30]);
    }

    // Joints and contacts together: a chain of boxes on distance and fixed joints draped over the ground.
    {
        let mut w = world(&base);
        ground(&mut w, 10.0, &p);
        let mut prev = body(&mut w, S, [-3.0, 3.0, 0.0], 0.0, z, ball(0.1), &p);
        for i in 0..6 {
            let x = -2.5 + 1.0 * f64::from(i);
            let b = body(&mut w, D, [x, 3.0, 0.0], 0.0, z, bx(0.45, 0.1, 0.2), &p);
            let jt = if i % 2 == 0 {
                JointType::Distance { length: 1.0 }
            } else {
                JointType::Fixed
            };
            w.add_joint(JointDesc {
                body_a: prev,
                body_b: b,
                joint_type: jt,
                local_anchor_a: [0.0, 0.0],
                local_anchor_b: [-0.5, 0.0],
                motor: None,
                damping: 0.0,
                break_force: None,
            });
            prev = b;
        }
        rows(&mut m, scene(w), &FRAMES);
    }

    // Seeded scenes: a box of static walls, three to eight random dynamic shapes (sensors, rule and material
    // variety), random velocities and spins, either broadphase, sampled at random frames.
    let mut rng = Rng::new(0xc0_7ac7_0003);
    for _ in 0..40 {
        let mut cfg = base.clone();
        cfg.sub_steps = u32::try_from(rng.int(1, 2)).unwrap_or(1);
        if rng.int(0, 1) == 1 {
            cfg.broadphase = BroadphaseKind::AabbTree;
        }
        if rng.int(0, 3) == 0 {
            cfg.constraint_frequency = 0.0;
        }
        let mut w = world(&cfg);
        ground(&mut w, 6.0, &p);
        body(&mut w, S, [-6.5, 5.0, 0.0], 0.0, z, bx(0.5, 6.0, 6.0), &p);
        body(&mut w, S, [6.5, 5.0, 0.0], 0.0, z, bx(0.5, 6.0, 6.0), &p);
        body(&mut w, S, [0.0, 5.0, -6.5], 0.0, z, bx(6.0, 6.0, 0.5), &p);
        body(&mut w, S, [0.0, 5.0, 6.5], 0.0, z, bx(6.0, 6.0, 0.5), &p);
        let n = rng.int(3, 8);
        for _ in 0..n {
            let shape = match rng.int(0, 3) {
                0 => ball(rng.uniform(0.2, 0.7)),
                1 => bx(rng.uniform(0.2, 0.7), rng.uniform(0.2, 0.7), rng.uniform(0.2, 0.7)),
                2 => capsule(rng.uniform(0.2, 0.6), rng.uniform(0.15, 0.4)),
                _ => cube_hull(rng.uniform(0.2, 0.6)),
            };
            let mut mat = material(rng.uniform(0.0, 1.0), rng.uniform(0.0, 0.9));
            mat.rolling_friction = if rng.int(0, 3) == 0 {
                rng.uniform(0.0, 0.2)
            } else {
                0.0
            };
            mat.friction_combine = match rng.int(0, 3) {
                0 => CombineRule::Min,
                1 => CombineRule::Average,
                2 => CombineRule::Multiply,
                _ => CombineRule::Max,
            };
            let c = Coll {
                material: mat,
                sensor: rng.int(0, 9) == 0,
                ..p.clone()
            };
            let pos = [rng.uniform(-4.5, 4.5), rng.uniform(1.0, 8.0), rng.uniform(-4.5, 4.5)];
            let vel = [rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 2.0), rng.uniform(-5.0, 5.0)];
            let b = body(&mut w, D, pos, rng.uniform(-3.0, 3.0), vel, shape, &c);
            if rng.int(0, 1) == 0 {
                let q = [rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4), rng.uniform(-0.4, 0.4)];
                w.apply_impulse(b, Impulse::at_point(rng.uniform(-1.0, 1.0), 0.0, rng.uniform(-1.0, 1.0), q[0], q[1], q[2]));
            }
        }
        let k = u64::try_from(rng.int(0, 150)).unwrap_or(0);
        rows(&mut m, scene(w), &[0, 1, k, k + 1]);
    }
    m
}
