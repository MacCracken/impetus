//! `b3_step.tsv` — single-step rows (class S, port plan §5) of the 3D backend (M5 part 2: velocity and position
//! integration with the gyroscopic step, the joints, sleep).
//!
//! A row is one `PhysicsWorld::step` from a recorded state, as `b2_step.tsv`'s. Inputs: the world config
//! (`flat::config`), then the 3D backend dump at frame k (`dump3d::backend`), taken after that frame's applied forces.
//! Outputs: the dump at frame k + 1, then the frame's collision events (`v:N` and each event).
//!
//! Every collider has collision mask 0, so the broadphase pairs nothing (it drops a pair when neither layer meets the
//! other's mask) and every contact stage of the step is empty; colliders are there for mass. A body's rotation is a
//! `sin_cos` (`from_rotation_z`) and a Wheel's perpendicular impulses meet in a `hypot` when it breaks, so the rows are
//! class C: the reference is the corrected oracle (`validate.py` transcribes this part of the step and writes
//! `b3_step.cr.tsv` where glibc misrounds a call).

use std::f64::consts::FRAC_PI_2;

use impetus::{
    BodyDesc, BodyHandle, BodyType, ColliderDesc, ColliderShape, Force, Impulse, JointDesc, JointMotor, JointType,
    PhysicsMaterial, PhysicsWorld, Torque, WorldConfig,
};

use crate::dump3d;
use crate::flat;
use crate::grids::Rng;
use crate::tsv::{Class, Module, Tok};

/// A scene: a built world, and what it applies before each frame: a force (with or without a point) and a torque.
struct Scene {
    world: PhysicsWorld,
    push: Option<(BodyHandle, Force)>,
    twist: Option<(BodyHandle, f64)>,
}

fn scene(world: PhysicsWorld) -> Scene {
    Scene {
        world,
        push: None,
        twist: None,
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

/// A body of type `t` at `pos`, rotated `rot` about z, with one collider of `shape` (layer 1, mask 0) if given.
fn body(w: &mut PhysicsWorld, t: BodyType, pos: [f64; 3], rot: f64, shape: Option<ColliderShape>) -> BodyHandle {
    let h = w.add_body(BodyDesc {
        body_type: t,
        position: pos,
        rotation: rot,
        ..BodyDesc::default()
    });
    if let Some(s) = shape {
        collider(w, h, s, [0.0, 0.0, 0.0]);
    }
    h
}

fn collider(w: &mut PhysicsWorld, h: BodyHandle, shape: ColliderShape, offset: [f64; 3]) {
    w.add_collider(
        h,
        ColliderDesc {
            shape,
            offset,
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 1,
            collision_mask: 0,
        },
    );
}

fn jd(a: BodyHandle, b: BodyHandle, jt: JointType, la: [f64; 2], lb: [f64; 2]) -> JointDesc {
    JointDesc {
        body_a: a,
        body_b: b,
        joint_type: jt,
        local_anchor_a: la,
        local_anchor_b: lb,
        motor: None,
        damping: 0.0,
        break_force: None,
    }
}

fn joint(w: &mut PhysicsWorld, a: BodyHandle, b: BodyHandle, jt: JointType, la: [f64; 2], lb: [f64; 2]) {
    w.add_joint(jd(a, b, jt, la, lb));
}

/// An off-centre impulse: a linear kick and a spin about an axis off every coordinate axis.
fn spin(w: &mut PhysicsWorld, h: BodyHandle, v: [f64; 3], p: [f64; 3]) {
    w.apply_impulse(h, Impulse::at_point(v[0], v[1], v[2], p[0], p[1], p[2]));
}

const S: BodyType = BodyType::Static;
const D: BodyType = BodyType::Dynamic;
const K: BodyType = BodyType::Kinematic;

/// Runs a scene to the last frame asked, emitting a row for each frame k in `frames`.
fn rows(m: &mut Module, mut sc: Scene, frames: &[u64]) {
    let last = frames.iter().copied().max().unwrap_or(0);
    let cfg = sc.world.config().clone();
    for k in 0..=last {
        if let Some((h, f)) = &sc.push {
            sc.world.apply_force(*h, f.clone());
        }
        if let Some((h, v)) = sc.twist {
            sc.world.apply_torque(h, Torque { value: v });
        }
        let take = frames.contains(&k);
        let mut ins = Vec::new();
        if take {
            let mut c = cfg.clone();
            c.step = k;
            ins.extend(flat::config(&c));
            ins.extend(dump3d::backend(&sc.world.oracle_dump().backend));
        }
        sc.world.step();
        if take {
            let mut outs = dump3d::backend(&sc.world.oracle_dump().backend);
            let ev = sc.world.collision_events();
            outs.push(Tok::V(ev.len()));
            for x in ev {
                outs.extend(flat::event(x));
            }
            m.row("b3::step", Class::C, &ins, &outs);
        }
    }
}

const FRAMES: [u64; 6] = [0, 1, 2, 10, 60, 300];

fn world(cfg: &WorldConfig) -> PhysicsWorld {
    PhysicsWorld::new(cfg.clone())
}

#[allow(clippy::too_many_lines)] // the scenes
pub fn emit() -> Module {
    let mut m = Module::new("b3_step");
    let base = WorldConfig::default();

    // Free fall at 1, 4 and 8 sub-steps; a damped body with half gravity; the speed cap.
    for sub in [1, 4, 8] {
        let mut cfg = base.clone();
        cfg.sub_steps = sub;
        let mut w = world(&cfg);
        body(&mut w, D, [0.0, 10.0, 0.0], 0.3, Some(ball(0.5)));
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = world(&base);
        let h = w.add_body(BodyDesc {
            position: [1.0, 2.0, 3.0],
            linear_velocity: [3.0, 1.0, -2.0],
            angular_velocity: 4.0,
            linear_damping: 0.3,
            angular_damping: 0.7,
            gravity_scale: Some(0.5),
            ..BodyDesc::default()
        });
        collider(&mut w, h, bx(0.5, 0.25, 0.75), [0.0, 0.0, 0.0]);
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut cfg = base.clone();
        cfg.max_velocity = 5.0;
        let mut w = world(&cfg);
        let h = w.add_body(BodyDesc {
            linear_velocity: [20.0, 5.0, -3.0],
            ..BodyDesc::default()
        });
        collider(&mut w, h, ball(0.3), [0.0, 0.0, 0.0]);
        rows(&mut m, scene(w), &[0, 1, 2, 10]);
    }

    // Free spins, gravity off: an asymmetric box tumbling about an axis off its principal axes (the gyroscopic step),
    // a symmetric ball, a box with fixed rotation, a capsule with an offset collider, and a fast spin.
    {
        let mut cfg = base.clone();
        cfg.gravity = [0.0, 0.0, 0.0];
        let mut w = world(&cfg);
        let a = body(&mut w, D, [0.0, 0.0, 0.0], 0.2, Some(bx(1.0, 0.5, 0.25)));
        spin(&mut w, a, [0.0, 0.0, 2.0], [0.3, 0.7, 0.0]);
        spin(&mut w, a, [1.0, 0.0, 0.0], [0.0, 0.2, 0.4]);
        let b = body(&mut w, D, [5.0, 0.0, 0.0], 0.0, Some(ball(0.5)));
        spin(&mut w, b, [0.0, 1.0, 1.0], [0.5, 0.0, 0.0]);
        let c = w.add_body(BodyDesc {
            position: [-5.0, 0.0, 0.0],
            angular_velocity: 2.0,
            fixed_rotation: true,
            ..BodyDesc::default()
        });
        collider(&mut w, c, bx(0.5, 0.5, 0.5), [0.0, 0.0, 0.0]);
        let d = body(&mut w, D, [0.0, 5.0, 0.0], -0.4, None);
        collider(&mut w, d, capsule(0.6, 0.2), [0.3, 0.1, -0.2]);
        spin(&mut w, d, [0.0, 0.0, 1.5], [0.2, -0.4, 0.0]);
        let e = body(&mut w, D, [0.0, -5.0, 0.0], 0.0, Some(bx(0.2, 0.6, 0.1)));
        spin(&mut w, e, [0.0, 0.0, 3.0], [1.0, 0.5, 0.0]);
        spin(&mut w, e, [2.0, 0.0, 0.0], [0.0, 0.0, 0.8]);
        rows(&mut m, scene(w), &FRAMES);
    }
    // A torque every frame, and a force at a point (its torque p × f) every frame.
    {
        let mut cfg = base.clone();
        cfg.gravity = [0.0, 0.0, 0.0];
        let mut w = world(&cfg);
        let a = body(&mut w, D, [0.0, 0.0, 0.0], 0.5, Some(bx(0.8, 0.3, 0.4)));
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
                twist: Some((a, 1.5)),
            },
            &FRAMES,
        );
    }
    {
        let mut w = world(&base);
        let a = body(&mut w, D, [0.0, 0.0, 0.0], 0.0, Some(capsule(0.5, 0.3)));
        rows(
            &mut m,
            Scene {
                world: w,
                push: Some((a, Force::at_point(0.0, 12.0, 1.0, 0.4, 0.0, 0.3))),
                twist: None,
            },
            &FRAMES,
        );
    }

    // A pendulum on a distance joint at 1 and 4 sub-steps; a hanging ball; a damped distance joint.
    for sub in [1, 4] {
        let mut cfg = base.clone();
        cfg.sub_steps = sub;
        let mut w = world(&cfg);
        let anchor = body(&mut w, S, [0.0, 10.0, 0.0], 0.0, None);
        let bob = body(&mut w, D, [2.0, 10.0, 0.5], 0.0, Some(ball(0.25)));
        joint(&mut w, anchor, bob, JointType::Distance { length: 2.0 }, [0.0, 0.0], [0.0, 0.0]);
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = world(&base);
        let anchor = body(&mut w, S, [0.0, 10.0, 0.0], 0.0, None);
        let bob = body(&mut w, D, [0.5, 8.0, 0.0], 0.0, Some(bx(0.2, 0.2, 0.2)));
        let mut d = jd(anchor, bob, JointType::Distance { length: 2.0 }, [0.0, 0.0], [0.2, 0.1]);
        d.damping = 0.8;
        w.add_joint(d);
        rows(&mut m, scene(w), &FRAMES);
    }

    // Fixed joints: the cantilever, a pair rotated against each other, a joint to a rotated static body, and a body
    // with fixed rotation (the point block alone).
    {
        let mut w = world(&base);
        let a = body(&mut w, S, [0.0, 0.0, 0.0], 0.0, None);
        let b = body(&mut w, D, [2.0, 0.0, 0.0], 0.0, Some(bx(0.5, 0.1, 0.1)));
        joint(&mut w, a, b, JointType::Fixed, [0.0, 0.0], [-2.0, 0.0]);
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = world(&base);
        let pin = body(&mut w, S, [0.0, 2.0, 0.0], 0.7, None);
        let a = body(&mut w, D, [1.0, 2.0, 0.0], 0.3, Some(bx(0.5, 0.25, 0.2)));
        let b = body(&mut w, D, [2.0, 2.2, 0.1], -0.2, Some(bx(0.5, 0.25, 0.3)));
        joint(&mut w, pin, a, JointType::Fixed, [0.5, 0.0], [-0.5, 0.0]);
        joint(&mut w, a, b, JointType::Fixed, [0.5, 0.0], [-0.5, 0.0]);
        spin(&mut w, b, [0.0, 0.0, 1.0], [0.4, 0.2, 0.0]);
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = world(&base);
        let a = body(&mut w, S, [0.0, 0.0, 0.0], 0.0, None);
        let b = w.add_body(BodyDesc {
            position: [1.0, 0.0, 0.0],
            fixed_rotation: true,
            ..BodyDesc::default()
        });
        collider(&mut w, b, ball(0.2), [0.0, 0.0, 0.0]);
        joint(&mut w, a, b, JointType::Fixed, [1.0, 0.0], [0.0, 0.0]);
        rows(&mut m, scene(w), &FRAMES);
    }

    // A wheel on a suspension under a chassis pinned to the world; a wheel with a zero axis (no rows, no force).
    {
        let mut w = world(&base);
        let chassis = body(&mut w, D, [0.0, 1.0, 0.0], 0.0, Some(bx(1.0, 0.25, 0.5)));
        let pin = body(&mut w, S, [0.0, 1.0, 0.0], 0.0, None);
        let wheel = body(&mut w, D, [0.8, 0.4, 0.1], 0.0, Some(ball(0.3)));
        joint(&mut w, pin, chassis, JointType::Fixed, [0.0, 0.0], [0.0, 0.0]);
        joint(
            &mut w,
            chassis,
            wheel,
            JointType::Wheel {
                axis: [0.0, 1.0],
                stiffness: 50.0,
                damping: 2.0,
            },
            [0.8, -0.6],
            [0.0, 0.0],
        );
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = world(&base);
        let a = body(&mut w, S, [0.0, 5.0, 0.0], 0.0, None);
        let b = body(&mut w, D, [0.3, 4.0, 0.2], 0.0, Some(ball(0.3)));
        let c = body(&mut w, D, [-0.3, 4.0, 0.2], 0.0, Some(ball(0.3)));
        joint(
            &mut w,
            a,
            b,
            JointType::Wheel {
                axis: [0.0, 0.0],
                stiffness: 50.0,
                damping: 2.0,
            },
            [0.0, 0.0],
            [0.0, 0.0],
        );
        joint(
            &mut w,
            a,
            c,
            JointType::Wheel {
                axis: [1.0, 1.0],
                stiffness: 30.0,
                damping: 1.0,
            },
            [0.0, 0.0],
            [0.0, 0.0],
        );
        rows(&mut m, scene(w), &[0, 1, 2, 10, 60]);
    }

    // Spring (and one at zero length), rope (slack, then taut), mouse (capped, uncapped, NaN cap).
    {
        let mut w = world(&base);
        let a = body(&mut w, D, [0.0, 0.0, 0.0], 0.0, Some(ball(0.25)));
        let b = body(&mut w, D, [2.5, 0.5, -0.5], 0.0, Some(ball(0.25)));
        joint(
            &mut w,
            a,
            b,
            JointType::Spring {
                rest_length: 1.5,
                stiffness: 20.0,
                damping: 0.5,
            },
            [0.0, 0.0],
            [0.0, 0.0],
        );
        let c = body(&mut w, D, [5.0, 0.0, 0.0], 0.0, Some(ball(0.25)));
        let d = body(&mut w, D, [5.0, 0.0, 0.0], 0.0, Some(ball(0.25)));
        joint(
            &mut w,
            c,
            d,
            JointType::Spring {
                rest_length: 1.0,
                stiffness: 20.0,
                damping: 0.5,
            },
            [0.0, 0.0],
            [0.0, 0.0],
        );
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = world(&base);
        let anchor = body(&mut w, S, [0.0, 10.0, 0.0], 0.0, None);
        let bob = body(&mut w, D, [0.5, 9.0, 0.3], 0.0, Some(ball(0.25)));
        joint(&mut w, anchor, bob, JointType::Rope { max_length: 2.0 }, [0.0, 0.0], [0.0, 0.0]);
        rows(&mut m, scene(w), &[0, 1, 2, 10, 15, 20, 25, 30, 60]);
    }
    for (stiffness, damping, max_force) in [(40.0, 3.0, 1000.0), (40.0, 3.0, 5.0), (40.0, 3.0, f64::NAN)] {
        let mut w = world(&base);
        let b = body(&mut w, D, [0.0, 0.0, 0.0], 0.4, Some(bx(0.5, 0.25, 0.25)));
        joint(
            &mut w,
            b,
            b,
            JointType::Mouse {
                target: [2.0, 1.0, -1.0],
                stiffness,
                damping,
                max_force,
            },
            [0.4, 0.1],
            [0.0, 0.0],
        );
        rows(&mut m, scene(w), &FRAMES);
    }

    // Revolute and Prismatic are no-ops in 3D apart from the per-joint damping (motors ignored); a static pair.
    {
        let mut w = world(&base);
        let hub = body(&mut w, S, [0.0, 0.0, 0.0], 0.0, None);
        let arm = body(&mut w, D, [1.0, 0.0, 0.0], 0.0, Some(bx(1.0, 0.1, 0.1)));
        let mut d = jd(
            hub,
            arm,
            JointType::Revolute {
                anchor: [0.0, 0.0],
                limits: Some([-0.5, 0.5]),
            },
            [0.0, 0.0],
            [-1.0, 0.0],
        );
        d.motor = Some(JointMotor {
            target_velocity: 2.0,
            max_force: 50.0,
        });
        d.damping = 0.5;
        w.add_joint(d);
        let block = body(&mut w, D, [0.0, 3.0, 0.0], 0.0, Some(bx(0.25, 0.25, 0.25)));
        let mut p = jd(
            hub,
            block,
            JointType::Prismatic {
                axis: [1.0, 0.0],
                limits: Some([-1.0, 1.0]),
            },
            [0.0, 0.0],
            [0.0, 0.0],
        );
        p.damping = 2.0;
        w.add_joint(p);
        let s2 = body(&mut w, S, [4.0, 0.0, 0.0], 0.0, Some(ball(0.5)));
        joint(&mut w, hub, s2, JointType::Distance { length: 1.0 }, [0.0, 0.0], [0.0, 0.0]);
        rows(&mut m, scene(w), &FRAMES);
    }

    // Breaking: a distance joint by its row impulse, a spring and a mouse by their explicit force, a wheel by its two
    // perpendicular impulses (their hypot), a fixed joint; and ones that hold.
    {
        let mut w = world(&base);
        let anchor = body(&mut w, S, [0.0, 10.0, 0.0], 0.0, None);
        for (i, (jt, bf)) in [
            (JointType::Distance { length: 2.0 }, 0.5),
            (JointType::Distance { length: 2.0 }, 5.0),
            (
                JointType::Spring {
                    rest_length: 0.5,
                    stiffness: 30.0,
                    damping: 0.0,
                },
                20.0,
            ),
            (
                JointType::Wheel {
                    axis: [1.0, 0.2],
                    stiffness: 20.0,
                    damping: 1.0,
                },
                0.8,
            ),
            (JointType::Fixed, 3.0),
            (JointType::Rope { max_length: 1.5 }, 2.0),
        ]
        .into_iter()
        .enumerate()
        {
            let x = 3.0 * f64::from(u8::try_from(i).unwrap_or(0));
            let bob = body(&mut w, D, [x + 0.4, 8.0, 0.1], 0.0, Some(ball(0.25)));
            let mut d = jd(anchor, bob, jt, [x, 0.0], [0.0, 0.0]);
            d.break_force = Some(bf);
            w.add_joint(d);
        }
        let grab = body(&mut w, D, [-3.0, 8.0, 0.0], 0.0, Some(ball(0.25)));
        let mut d = jd(
            grab,
            grab,
            JointType::Mouse {
                target: [-3.0, 12.0, 0.0],
                stiffness: 100.0,
                damping: 0.0,
                max_force: 1000.0,
            },
            [0.0, 0.0],
            [0.0, 0.0],
        );
        d.break_force = Some(50.0);
        w.add_joint(d);
        rows(&mut m, scene(w), &[0, 1, 2, 3, 10]);
    }

    // A kinematic driver; a push every frame; a chain of mixed joints; a body falling asleep.
    {
        let mut w = world(&base);
        let k = w.add_body(BodyDesc {
            body_type: K,
            linear_velocity: [1.0, 0.0, 0.5],
            angular_velocity: 0.5,
            ..BodyDesc::default()
        });
        let a = body(&mut w, D, [1.0, 0.0, 0.0], 0.0, Some(ball(0.3)));
        joint(&mut w, k, a, JointType::Distance { length: 1.0 }, [0.0, 0.0], [0.0, 0.0]);
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut w = world(&base);
        let anchor = body(&mut w, S, [0.0, 0.0, 0.0], 0.0, None);
        let bob = body(&mut w, D, [0.0, -1.0, 0.0], 0.0, Some(ball(0.25)));
        joint(&mut w, anchor, bob, JointType::Distance { length: 1.0 }, [0.0, 0.0], [0.0, 0.0]);
        rows(
            &mut m,
            Scene {
                world: w,
                push: Some((bob, Force::new(2.0, 0.0, 1.0))),
                twist: None,
            },
            &FRAMES,
        );
    }
    {
        let mut w = world(&base);
        let mut prev = body(&mut w, S, [0.0, 0.0, 0.0], 0.0, None);
        for i in 0..10 {
            let x = 0.5 + f64::from(i);
            let link = body(&mut w, D, [x, 0.0, 0.0], 0.0, Some(bx(0.5, 0.1, 0.1)));
            let jt = match i % 4 {
                0 => JointType::Fixed,
                1 => JointType::Distance { length: 1.0 },
                2 => JointType::Rope { max_length: 1.05 },
                _ => JointType::Spring {
                    rest_length: 1.0,
                    stiffness: 200.0,
                    damping: 4.0,
                },
            };
            let la = if i == 0 { [0.0, 0.0] } else { [0.5, 0.0] };
            joint(&mut w, prev, link, jt, la, [-0.5, 0.0]);
            prev = link;
        }
        rows(&mut m, scene(w), &FRAMES);
    }
    {
        let mut cfg = base.clone();
        cfg.gravity = [0.0, 0.0, 0.0];
        let mut w = world(&cfg);
        let a = w.add_body(BodyDesc {
            linear_velocity: [0.005, 0.0, 0.002],
            ..BodyDesc::default()
        });
        collider(&mut w, a, ball(0.5), [0.0, 0.0, 0.0]);
        let b = body(&mut w, D, [3.0, 0.0, 0.0], 0.0, Some(bx(0.3, 0.3, 0.3)));
        spin(&mut w, b, [0.0, 0.0, 0.001], [0.2, 0.0, 0.0]);
        rows(&mut m, scene(w), &[0, 1, 10, 28, 29, 30, 31, 60]);
    }

    // The solver settings: Baumgarte (no soft constraint), no position iterations, 1 and 8 velocity iterations, a
    // long timestep.
    for (freq, pos_it, vel_it, ts) in [(0.0, 1, 4, 1.0 / 60.0), (30.0, 0, 4, 1.0 / 60.0), (30.0, 3, 1, 1.0 / 60.0),
                                      (10.0, 2, 8, 1.0 / 30.0)] {
        let mut cfg = base.clone();
        cfg.constraint_frequency = freq;
        cfg.position_iterations = pos_it;
        cfg.velocity_iterations = vel_it;
        cfg.timestep = ts;
        let mut w = world(&cfg);
        let anchor = body(&mut w, S, [0.0, 0.0, 0.0], FRAC_PI_2, None);
        let a = body(&mut w, D, [1.0, 0.0, 0.0], 0.0, Some(bx(0.4, 0.2, 0.2)));
        let b = body(&mut w, D, [2.0, -1.0, 0.0], 0.0, Some(ball(0.3)));
        joint(&mut w, anchor, a, JointType::Fixed, [0.0, -1.0], [-0.4, 0.0]);
        joint(&mut w, a, b, JointType::Distance { length: 1.2 }, [0.4, 0.0], [0.0, 0.0]);
        rows(&mut m, scene(w), &[0, 1, 2, 10, 60]);
    }

    // Seeded scenes: two to six bodies (every collider kind), spins, random joints between them and to a static anchor.
    let mut rng = Rng::new(0x57e9_0003);
    for _ in 0..50 {
        let mut cfg = base.clone();
        cfg.sub_steps = u32::try_from(rng.int(1, 3)).unwrap_or(1);
        if rng.int(0, 3) == 0 {
            cfg.constraint_frequency = 0.0;
        }
        cfg.velocity_iterations = u32::try_from(rng.int(1, 8)).unwrap_or(4);
        cfg.position_iterations = u32::try_from(rng.int(0, 3)).unwrap_or(1);
        let mut w = world(&cfg);
        let mut hs = vec![body(&mut w, S, [0.0, 0.0, 0.0], rng.uniform(-3.0, 3.0), None)];
        let n = rng.int(2, 6);
        for _ in 0..n {
            let shape = match rng.int(0, 4) {
                0 => ball(rng.uniform(0.1, 0.6)),
                1 => bx(rng.uniform(0.1, 0.6), rng.uniform(0.1, 0.6), rng.uniform(0.1, 0.6)),
                2 => capsule(rng.uniform(0.1, 0.6), rng.uniform(0.1, 0.4)),
                3 => ColliderShape::ConvexHull {
                    points: vec![
                        [0.0, 0.0, 0.0],
                        [rng.uniform(0.2, 0.8), 0.0, 0.0],
                        [0.0, rng.uniform(0.2, 0.8), 0.0],
                        [0.0, 0.0, rng.uniform(0.2, 0.8)],
                    ],
                },
                _ => ColliderShape::Segment {
                    a: [-0.5, 0.0, 0.0],
                    b: [0.5, 0.0, 0.0],
                },
            };
            let t = if rng.int(0, 7) == 0 { K } else { D };
            let p = [rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)];
            let h = w.add_body(BodyDesc {
                body_type: t,
                position: p,
                rotation: rng.uniform(-3.0, 3.0),
                linear_velocity: [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)],
                angular_velocity: rng.uniform(-2.0, 2.0),
                linear_damping: if rng.int(0, 2) == 0 { rng.uniform(0.0, 1.0) } else { 0.0 },
                angular_damping: if rng.int(0, 2) == 0 { rng.uniform(0.0, 1.0) } else { 0.0 },
                fixed_rotation: rng.int(0, 6) == 0,
                ..BodyDesc::default()
            });
            let off = if rng.int(0, 2) == 0 {
                [rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3)]
            } else {
                [0.0, 0.0, 0.0]
            };
            collider(&mut w, h, shape, off);
            if rng.int(0, 1) == 0 {
                let v = [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)];
                let q = [rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5)];
                spin(&mut w, h, v, q);
            }
            hs.push(h);
        }
        let pick = |rng: &mut Rng, n: usize| hs[usize::try_from(rng.int(0, i64::try_from(n).unwrap_or(1) - 1)).unwrap_or(0)];
        for (i, &b) in hs.iter().enumerate().skip(1) {
            let a = pick(&mut rng, i);
            let jt = match rng.int(0, 8) {
                0 => JointType::Fixed,
                1 => JointType::Revolute {
                    anchor: [0.0, 0.0],
                    limits: None,
                },
                2 => JointType::Prismatic {
                    axis: [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)],
                    limits: Some([-1.0, 1.0]),
                },
                3 => JointType::Spring {
                    rest_length: rng.uniform(0.5, 2.0),
                    stiffness: rng.uniform(5.0, 80.0),
                    damping: rng.uniform(0.0, 3.0),
                },
                4 => JointType::Distance {
                    length: rng.uniform(0.5, 3.0),
                },
                5 => JointType::Wheel {
                    axis: [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)],
                    stiffness: rng.uniform(5.0, 80.0),
                    damping: rng.uniform(0.0, 3.0),
                },
                6 => JointType::Rope {
                    max_length: rng.uniform(1.0, 4.0),
                },
                7 => JointType::Fixed,
                _ => JointType::Mouse {
                    target: [rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)],
                    stiffness: rng.uniform(5.0, 80.0),
                    damping: rng.uniform(0.0, 3.0),
                    max_force: rng.uniform(1.0, 200.0),
                },
            };
            let mut d = jd(
                a,
                b,
                jt,
                [rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5)],
                [rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5)],
            );
            if rng.int(0, 2) == 0 {
                d.damping = rng.uniform(0.0, 1.0);
            }
            if rng.int(0, 4) == 0 {
                d.break_force = Some(rng.uniform(1.0, 200.0));
            }
            w.add_joint(d);
        }
        let k = u64::try_from(rng.int(0, 90)).unwrap_or(0);
        rows(&mut m, scene(w), &[0, k, k + 1]);
    }
    m
}
