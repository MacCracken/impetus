//! `b2_contact.tsv` — single-step rows (class S, port plan §5) of the 2D backend with contacts (M3c):
//! the broadphase (both kinds), the narrowphase and the manifold rebuild with warm-start matching, waking
//! on contact, warm starting, the contact solver (block and sequential normal impulses, friction, rolling
//! resistance, restitution) and the split-impulse position solver, with joints where a scene has them.
//!
//! The row format is `b2_step.tsv`'s (`m_b2_step::rows`): the config and the backend dump at frame k in,
//! the dump at k + 1 and the frame's collision events out, fn `b2::step`. Class C: the corrected oracle
//! (`validate.py` transcribes the step, contacts included, and writes `b2_contact.cr.tsv` where glibc
//! misrounds a call).

use impetus::{
    BodyDesc, BodyHandle, BodyType, BroadphaseKind, ColliderDesc, ColliderShape, CombineRule,
    JointDesc, JointType, PhysicsMaterial, PhysicsWorld, WorldConfig,
};

use crate::grids::Rng;
use crate::m_b2_step::{Scene, rows};
use crate::shapes::{ball, bx, capsule, ellipse_hull, random_shape, segment};
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
    offset: [f64; 2],
    sensor: bool,
    layer: u32,
    mask: u32,
}

impl Coll {
    fn plain() -> Self {
        Self {
            material: PhysicsMaterial::default(),
            offset: [0.0, 0.0],
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

/// A body of type t at `pos`, rotated by `rot`, moving at `vel`, with one collider.
fn body(
    w: &mut PhysicsWorld,
    t: BodyType,
    pos: [f64; 2],
    rot: f64,
    vel: [f64; 2],
    shape: ColliderShape,
    c: &Coll,
) -> BodyHandle {
    let h = w.add_body(BodyDesc {
        body_type: t,
        position: [pos[0], pos[1], 0.0],
        rotation: rot,
        linear_velocity: [vel[0], vel[1], 0.0],
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
            offset: [c.offset[0], c.offset[1], 0.0],
            material: c.material.clone(),
            is_sensor: c.sensor,
            mass: None,
            collision_layer: c.layer,
            collision_mask: c.mask,
        },
    );
}

/// A static ground box, its top at y = 0, `half` wide.
fn ground(w: &mut PhysicsWorld, half: f64, c: &Coll) -> BodyHandle {
    body(w, S, [0.0, -0.5], 0.0, [0.0, 0.0], bx(half, 0.5), c)
}

fn scene(world: PhysicsWorld) -> Scene {
    Scene { world, push: None }
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

#[allow(clippy::too_many_lines)] // the scenes
pub fn emit() -> Module {
    let mut m = Module::new("b2_contact");
    let base = WorldConfig::default();
    let p = Coll::plain();

    // A ball dropped on the ground, a box dropped on it (two points: the block solver), and a box set
    // down at an angle (one corner, then two), for each broadphase.
    for kind in [BroadphaseKind::SpatialHash, BroadphaseKind::AabbTree] {
        let mut cfg = base.clone();
        cfg.broadphase = kind;
        let mut w = PhysicsWorld::new(cfg.clone());
        ground(&mut w, 10.0, &p);
        body(&mut w, D, [0.0, 2.0], 0.0, [0.0, 0.0], ball(0.5), &p);
        rows(&mut m, scene(w), &FRAMES);

        let mut w = PhysicsWorld::new(cfg.clone());
        ground(&mut w, 10.0, &p);
        body(&mut w, D, [0.0, 1.0], 0.0, [0.0, 0.0], bx(0.5, 0.5), &p);
        rows(&mut m, scene(w), &FRAMES);

        let mut w = PhysicsWorld::new(cfg);
        ground(&mut w, 10.0, &p);
        body(&mut w, D, [0.0, 1.2], 0.3, [0.0, 0.0], bx(0.6, 0.3), &p);
        rows(&mut m, scene(w), &FRAMES);
    }

    // A stack of five boxes and a pyramid of six (shock propagation's levels).
    {
        let mut w = PhysicsWorld::new(base.clone());
        ground(&mut w, 10.0, &p);
        for i in 0..5 {
            let y = 0.5 + 1.0 * f64::from(i);
            body(&mut w, D, [0.0, y], 0.0, [0.0, 0.0], bx(0.5, 0.5), &p);
        }
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = PhysicsWorld::new(base.clone());
        ground(&mut w, 10.0, &p);
        for row in 0..3_i32 {
            for col in 0..(3 - row) {
                let x = -1.05 + 1.05 * f64::from(col) + 0.525 * f64::from(row);
                let y = 0.5 + 1.0 * f64::from(row);
                body(&mut w, D, [x, y], 0.0, [0.0, 0.0], bx(0.5, 0.5), &p);
            }
        }
        rows(&mut m, scene(w), &[0, 1, 2, 10, 60, 240]);
    }

    // Restitution: bouncy, rubber and dead balls dropped fast; Max, Multiply and Min combine rules.
    for (rest, rule) in [
        (0.9, CombineRule::Max),
        (0.6, CombineRule::Multiply),
        (0.5, CombineRule::Min),
        (0.0, CombineRule::Average),
    ] {
        let mut w = PhysicsWorld::new(base.clone());
        ground(&mut w, 10.0, &Coll::with(material(0.5, 0.7)));
        let mut mat = material(0.4, rest);
        mat.restitution_combine = rule;
        body(
            &mut w,
            D,
            [0.0, 0.6],
            0.0,
            [0.0, -8.0],
            ball(0.5),
            &Coll::with(mat),
        );
        rows(&mut m, scene(w), &[0, 1, 2, 3, 10, 30]);
    }

    // Friction: a box sliding on a slope (wood), on ice, with a static friction override; a box shoved
    // along flat ground; rolling resistance on a rolling ball.
    for (mat, rule) in [
        (PhysicsMaterial::wood(), CombineRule::Average),
        (PhysicsMaterial::ice(), CombineRule::Min),
        (
            PhysicsMaterial {
                static_friction: Some(1.2),
                ..material(0.3, 0.0)
            },
            CombineRule::Max,
        ),
    ] {
        let mut w = PhysicsWorld::new(base.clone());
        let mut gm = mat.clone();
        gm.friction_combine = rule;
        body(
            &mut w,
            S,
            [0.0, 0.0],
            -0.35,
            [0.0, 0.0],
            bx(8.0, 0.5),
            &Coll::with(gm),
        );
        body(
            &mut w,
            D,
            [0.0, 1.1],
            -0.35,
            [0.0, 0.0],
            bx(0.4, 0.4),
            &Coll::with(mat),
        );
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = PhysicsWorld::new(base.clone());
        ground(&mut w, 20.0, &Coll::with(material(0.6, 0.0)));
        body(
            &mut w,
            D,
            [0.0, 0.5],
            0.0,
            [6.0, 0.0],
            bx(0.5, 0.5),
            &Coll::with(material(0.6, 0.0)),
        );
        rows(&mut m, scene(w), &FRAMES);
    }
    for roll in [0.0, 0.05, 0.5] {
        let mut w = PhysicsWorld::new(base.clone());
        let mut gm = material(0.8, 0.0);
        gm.rolling_friction = roll;
        ground(&mut w, 30.0, &Coll::with(gm.clone()));
        let h = w.add_body(BodyDesc {
            body_type: D,
            position: [0.0, 0.5, 0.0],
            linear_velocity: [4.0, 0.0, 0.0],
            angular_velocity: -8.0,
            ..BodyDesc::default()
        });
        attach(&mut w, h, ball(0.5), &Coll::with(gm));
        rows(&mut m, scene(w), &FRAMES);
    }

    // Shapes on the ground: a capsule lying and standing, a hull, a ball on a segment ground, two
    // capsules crossing, a collider offset from its body.
    {
        let mut w = PhysicsWorld::new(base.clone());
        ground(&mut w, 10.0, &p);
        body(
            &mut w,
            D,
            [-3.0, 0.5],
            std::f64::consts::FRAC_PI_2,
            [0.0, 0.0],
            capsule(0.6, 0.4),
            &p,
        );
        body(
            &mut w,
            D,
            [0.0, 1.0],
            0.0,
            [0.0, 0.0],
            capsule(0.6, 0.4),
            &p,
        );
        body(
            &mut w,
            D,
            [3.0, 1.2],
            0.2,
            [0.0, 0.0],
            ellipse_hull(6, 0.8, 0.6, true),
            &p,
        );
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = PhysicsWorld::new(base.clone());
        body(
            &mut w,
            S,
            [0.0, 0.0],
            0.0,
            [0.0, 0.0],
            segment([-5.0, 0.0], [5.0, 0.5]),
            &p,
        );
        body(&mut w, D, [-2.0, 1.0], 0.0, [1.0, 0.0], ball(0.3), &p);
        body(
            &mut w,
            D,
            [1.0, 2.0],
            0.4,
            [0.0, 0.0],
            capsule(0.5, 0.2),
            &p,
        );
        body(
            &mut w,
            D,
            [1.2, 3.5],
            -0.9,
            [0.0, 0.0],
            capsule(0.5, 0.2),
            &p,
        );
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = PhysicsWorld::new(base.clone());
        ground(&mut w, 10.0, &p);
        let off = Coll {
            offset: [0.6, 0.2],
            ..p.clone()
        };
        body(&mut w, D, [0.0, 1.0], 0.0, [0.0, 0.0], bx(0.3, 0.3), &off);
        rows(&mut m, scene(w), &FRAMES);
    }

    // Sensors, layers and kinematic bodies: a ball falling through a sensor onto the ground, a ball on
    // layer 4 with mask 4 (it still lands: the ground's all-layers mask meets its layer, and a pair is
    // dropped only when neither layer meets the other's mask), a kinematic paddle sweeping boxes.
    {
        let mut w = PhysicsWorld::new(base.clone());
        ground(&mut w, 10.0, &p);
        let sensor = Coll {
            sensor: true,
            ..p.clone()
        };
        body(
            &mut w,
            S,
            [0.0, 2.0],
            0.0,
            [0.0, 0.0],
            bx(1.0, 0.5),
            &sensor,
        );
        body(&mut w, D, [0.0, 4.0], 0.0, [0.0, 0.0], ball(0.3), &p);
        body(&mut w, D, [0.2, 2.0], 0.0, [0.0, 0.0], ball(0.2), &sensor);
        rows(&mut m, scene(w), &[0, 1, 10, 20, 25, 30, 35, 40, 60, 120]);
    }
    {
        let mut w = PhysicsWorld::new(base.clone());
        let gl = Coll {
            layer: 2,
            mask: 1,
            ..p.clone()
        };
        ground(&mut w, 10.0, &gl);
        let miss = Coll {
            layer: 4,
            mask: 4,
            ..p.clone()
        };
        body(&mut w, D, [-1.0, 0.6], 0.0, [0.0, 0.0], ball(0.5), &miss);
        body(&mut w, D, [1.0, 0.6], 0.0, [0.0, 0.0], ball(0.5), &p);
        rows(&mut m, scene(w), &[0, 1, 2, 10, 30]);
    }
    {
        let mut w = PhysicsWorld::new(base.clone());
        ground(&mut w, 10.0, &p);
        let k = w.add_body(BodyDesc {
            body_type: K,
            position: [-4.0, 0.6, 0.0],
            linear_velocity: [2.0, 0.0, 0.0],
            angular_velocity: 0.3,
            ..BodyDesc::default()
        });
        attach(&mut w, k, bx(0.2, 0.5), &p);
        for i in 0..3 {
            let x = -2.0 + 1.1 * f64::from(i);
            body(&mut w, D, [x, 0.5], 0.0, [0.0, 0.0], bx(0.5, 0.5), &p);
        }
        rows(&mut m, scene(w), &FRAMES);
    }

    // Sleep and wake on contact: a box comes to rest and sleeps; a ball dropped later wakes it.
    {
        let mut w = PhysicsWorld::new(base.clone());
        ground(&mut w, 10.0, &p);
        body(&mut w, D, [0.0, 0.5], 0.0, [0.0, 0.0], bx(0.5, 0.5), &p);
        body(&mut w, D, [0.1, 30.0], 0.0, [0.0, 0.0], ball(0.3), &p);
        rows(
            &mut m,
            scene(w),
            &[0, 30, 40, 50, 100, 140, 145, 150, 155, 160],
        );
    }

    // The position solver's settings: Baumgarte (no constraint frequency) at two correction factors,
    // a stiff soft constraint, more slop, more position iterations; four sub-steps.
    for (freq, corr, slop, piters, sub) in [
        (0.0, 0.2, 0.01, 3, 1),
        (0.0, 0.8, 0.0, 1, 1),
        (120.0, 0.2, 0.005, 3, 1),
        (30.0, 0.2, 0.05, 8, 1),
        (30.0, 0.2, 0.005, 3, 4),
    ] {
        let mut cfg = base.clone();
        cfg.constraint_frequency = freq;
        cfg.position_correction = corr;
        cfg.position_slop = slop;
        cfg.position_iterations = piters;
        cfg.sub_steps = sub;
        let mut w = PhysicsWorld::new(cfg);
        ground(&mut w, 10.0, &p);
        // Start overlapping, so the position solver has work at once.
        body(&mut w, D, [0.0, 0.3], 0.0, [0.0, 0.0], bx(0.5, 0.5), &p);
        body(&mut w, D, [1.5, 0.35], 0.0, [0.0, 0.0], ball(0.5), &p);
        rows(&mut m, scene(w), &[0, 1, 2, 10, 60]);
    }

    // Fast bodies (the swept AABB): a ball shot at a wall, a box falling from high.
    {
        let mut w = PhysicsWorld::new(base.clone());
        ground(&mut w, 10.0, &p);
        body(&mut w, S, [5.0, 2.0], 0.0, [0.0, 0.0], bx(0.5, 2.0), &p);
        body(&mut w, D, [2.0, 1.0], 0.0, [50.0, 0.0], ball(0.2), &p);
        body(&mut w, D, [-2.0, 10.0], 0.0, [0.0, -40.0], bx(0.3, 0.3), &p);
        rows(&mut m, scene(w), &[0, 1, 2, 3, 4, 5, 10]);
    }

    // Joints and contacts together: a chain of boxes on revolute joints draped over the ground.
    {
        let mut w = PhysicsWorld::new(base.clone());
        ground(&mut w, 10.0, &p);
        let anchor = body(&mut w, S, [-3.0, 3.0], 0.0, [0.0, 0.0], ball(0.1), &p);
        let mut prev = anchor;
        for i in 0..6 {
            let x = -2.5 + 1.0 * f64::from(i);
            let b = body(&mut w, D, [x, 3.0], 0.0, [0.0, 0.0], bx(0.45, 0.1), &p);
            w.add_joint(JointDesc {
                body_a: prev,
                body_b: b,
                joint_type: JointType::Revolute {
                    anchor: [0.0, 0.0],
                    limits: None,
                },
                local_anchor_a: if prev == anchor {
                    [0.0, 0.0]
                } else {
                    [0.5, 0.0]
                },
                local_anchor_b: [-0.5, 0.0],
                motor: None,
                damping: 0.0,
                break_force: None,
            });
            prev = b;
        }
        rows(&mut m, scene(w), &FRAMES);
    }

    // Seeded scenes: a box of static walls, three to eight random dynamic shapes (sensors, rule and
    // material variety), random velocities, either broadphase, sampled at random frames.
    let mut rng = Rng::new(0xc0_7ac7);
    for _ in 0..40 {
        let mut cfg = base.clone();
        cfg.sub_steps = u32::try_from(rng.int(1, 2)).unwrap_or(1);
        if rng.int(0, 1) == 1 {
            cfg.broadphase = BroadphaseKind::AabbTree;
        }
        if rng.int(0, 3) == 0 {
            cfg.constraint_frequency = 0.0;
        }
        let mut w = PhysicsWorld::new(cfg);
        ground(&mut w, 6.0, &p);
        body(&mut w, S, [-6.5, 5.0], 0.0, [0.0, 0.0], bx(0.5, 6.0), &p);
        body(&mut w, S, [6.5, 5.0], 0.0, [0.0, 0.0], bx(0.5, 6.0), &p);
        let n = rng.int(3, 8);
        for _ in 0..n {
            let shape = random_shape(&mut rng);
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
            let pos = [rng.uniform(-4.5, 4.5), rng.uniform(1.0, 8.0)];
            let vel = [rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 2.0)];
            body(&mut w, D, pos, rng.uniform(-3.0, 3.0), vel, shape, &c);
        }
        let k = u64::try_from(rng.int(0, 150)).unwrap_or(0);
        rows(&mut m, scene(w), &[0, 1, k, k + 1]);
    }
    m
}
