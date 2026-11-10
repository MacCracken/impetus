//! `b2_step.tsv` — single-step rows (class S, port plan §5) of the 2D backend (M3b: joints, integration,
//! islands and sleep).
//!
//! A row is one `PhysicsWorld::step` from a recorded state. Inputs: the world config (`flat::config`),
//! then the backend dump at frame k (`dump2d::backend`), taken after that frame's applied forces.
//! Outputs: the dump at frame k + 1, then the frame's collision events (`v:N` and each event).
//!
//! M3b's scenes give every collider collision mask 0, so the broadphase pairs nothing (it drops a pair
//! when neither layer meets the other's mask) and every contact stage of the step is empty; colliders
//! are there for mass. Rotations go through `sin_cos` and lengths through `hypot`, so the rows are
//! class C: the reference is the corrected oracle (`validate.py` transcribes this part of the step and
//! writes `b2_step.cr.tsv` where glibc misrounds a call).

use impetus::{
    BodyDesc, BodyHandle, BodyType, ColliderDesc, ColliderShape, Force, JointDesc, JointMotor,
    JointType, PhysicsMaterial, PhysicsWorld, WorldConfig,
};

use crate::dump2d;
use crate::flat;
use crate::grids::Rng;
use crate::shapes::{ball, bx};
use crate::tsv::{Class, Module, Tok};

/// A scene: a built world and the force it applies to a body before each frame (if any).
pub struct Scene {
    pub world: PhysicsWorld,
    pub push: Option<(BodyHandle, [f64; 2])>,
}

fn body(
    w: &mut PhysicsWorld,
    t: BodyType,
    pos: [f64; 2],
    rot: f64,
    shape: Option<ColliderShape>,
) -> BodyHandle {
    let h = w.add_body(BodyDesc {
        body_type: t,
        position: [pos[0], pos[1], 0.0],
        rotation: rot,
        ..BodyDesc::default()
    });
    if let Some(s) = shape {
        w.add_collider(
            h,
            ColliderDesc {
                shape: s,
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 1,
                collision_mask: 0,
            },
        );
    }
    h
}

fn joint(
    w: &mut PhysicsWorld,
    a: BodyHandle,
    b: BodyHandle,
    jt: JointType,
    la: [f64; 2],
    lb: [f64; 2],
) -> JointDesc {
    let d = JointDesc {
        body_a: a,
        body_b: b,
        joint_type: jt,
        local_anchor_a: la,
        local_anchor_b: lb,
        motor: None,
        damping: 0.0,
        break_force: None,
    };
    w.add_joint(d.clone());
    d
}

fn add(w: &mut PhysicsWorld, d: JointDesc) {
    w.add_joint(d);
}

fn world(cfg: &WorldConfig) -> PhysicsWorld {
    PhysicsWorld::new(cfg.clone())
}

const S: BodyType = BodyType::Static;
const D: BodyType = BodyType::Dynamic;
const K: BodyType = BodyType::Kinematic;

/// Runs a scene to the last frame asked, emitting a row for each frame k in `frames` (shared with
/// `m_b2_contact`).
pub fn rows(m: &mut Module, mut sc: Scene, frames: &[u64]) {
    let last = frames.iter().copied().max().unwrap_or(0);
    let cfg = sc.world.config().clone();
    for k in 0..=last {
        if let Some((h, f)) = sc.push {
            sc.world.apply_force(h, Force::new(f[0], f[1], 0.0));
        }
        let take = frames.contains(&k);
        let mut ins = Vec::new();
        if take {
            let mut c = cfg.clone();
            c.step = k;
            ins.extend(flat::config(&c));
            ins.extend(dump2d::backend(&sc.world.oracle_dump().backend));
        }
        sc.world.step();
        if take {
            let mut outs = dump2d::backend(&sc.world.oracle_dump().backend);
            let ev = sc.world.collision_events();
            outs.push(Tok::V(ev.len()));
            for x in ev {
                outs.extend(flat::event(x));
            }
            m.row("b2::step", Class::C, &ins, &outs);
        }
    }
}

const FRAMES: [u64; 6] = [0, 1, 2, 10, 60, 300];

#[allow(clippy::too_many_lines)] // the scenes
pub fn emit() -> Module {
    let mut m = Module::new("b2_step");
    let base = WorldConfig::default();

    // A pendulum: a ball on a revolute joint to a static anchor, at 1, 4 and 8 sub-steps.
    for sub in [1, 4, 8] {
        let mut cfg = base.clone();
        cfg.sub_steps = sub;
        let mut w = world(&cfg);
        let anchor = body(&mut w, S, [0.0, 0.0], 0.0, None);
        let bob = body(&mut w, D, [1.5, 0.0], 0.0, Some(ball(0.25)));
        joint(
            &mut w,
            anchor,
            bob,
            JointType::Revolute {
                anchor: [0.0, 0.0],
                limits: None,
            },
            [0.0, 0.0],
            [-1.5, 0.0],
        );
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &FRAMES,
        );
    }

    // A revolute chain of 20 links hanging from a static anchor.
    {
        let mut w = world(&base);
        let mut prev = body(&mut w, S, [0.0, 0.0], 0.0, None);
        let mut prev_dx = 0.0;
        for i in 0..20 {
            let x = 0.5 + f64::from(i);
            let link = body(&mut w, D, [x, 0.0], 0.0, Some(bx(0.5, 0.1)));
            joint(
                &mut w,
                prev,
                link,
                JointType::Revolute {
                    anchor: [0.0, 0.0],
                    limits: None,
                },
                [prev_dx, 0.0],
                [-0.5, 0.0],
            );
            prev = link;
            prev_dx = 0.5;
        }
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &FRAMES,
        );
    }

    // A fixed pair pinned to the world, and a revolute with limits and a motor.
    {
        let mut w = world(&base);
        let pin = body(&mut w, S, [0.0, 2.0], 0.0, None);
        let a = body(&mut w, D, [1.0, 2.0], 0.3, Some(bx(0.5, 0.25)));
        let b = body(&mut w, D, [2.0, 2.0], -0.2, Some(bx(0.5, 0.25)));
        joint(
            &mut w,
            pin,
            a,
            JointType::Revolute {
                anchor: [0.0, 0.0],
                limits: None,
            },
            [0.0, 0.0],
            [-1.0, 0.0],
        );
        joint(&mut w, a, b, JointType::Fixed, [0.5, 0.0], [-0.5, 0.0]);
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &FRAMES,
        );
    }
    {
        let mut w = world(&base);
        let hub = body(&mut w, S, [0.0, 0.0], 0.0, None);
        let arm = body(&mut w, D, [1.0, 0.0], 0.0, Some(bx(1.0, 0.1)));
        let mut d = joint(
            &mut w,
            hub,
            arm,
            JointType::Revolute {
                anchor: [0.0, 0.0],
                limits: Some([-0.5, 0.5]),
            },
            [0.0, 0.0],
            [-1.0, 0.0],
        );
        let _ = w.remove_joint(impetus::JointHandle(0));
        d.motor = Some(JointMotor {
            target_velocity: 2.0,
            max_force: 50.0,
        });
        d.damping = 0.5;
        add(&mut w, d);
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &FRAMES,
        );
    }

    // A prismatic slider with limits and a motor; a wheel on a suspension.
    {
        let mut w = world(&base);
        let rail = body(&mut w, S, [0.0, 0.0], 0.0, None);
        let block = body(&mut w, D, [0.0, 0.0], 0.0, Some(bx(0.25, 0.25)));
        let d = JointDesc {
            body_a: rail,
            body_b: block,
            joint_type: JointType::Prismatic {
                axis: [1.0, 0.0],
                limits: Some([-1.0, 1.0]),
            },
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: Some(JointMotor {
                target_velocity: 1.5,
                max_force: 20.0,
            }),
            damping: 0.25,
            break_force: None,
        };
        add(&mut w, d);
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &FRAMES,
        );
    }
    {
        let mut w = world(&base);
        let chassis = body(&mut w, D, [0.0, 1.0], 0.0, Some(bx(1.0, 0.25)));
        let pin = body(&mut w, S, [0.0, 1.0], 0.0, None);
        let wheel = body(&mut w, D, [0.8, 0.4], 0.0, Some(ball(0.3)));
        joint(
            &mut w,
            pin,
            chassis,
            JointType::Fixed,
            [0.0, 0.0],
            [0.0, 0.0],
        );
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
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &FRAMES,
        );
    }

    // Spring, distance (with damping), rope (slack, then taut), mouse.
    {
        let mut w = world(&base);
        let a = body(&mut w, D, [0.0, 0.0], 0.0, Some(ball(0.25)));
        let b = body(&mut w, D, [2.5, 0.5], 0.0, Some(ball(0.25)));
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
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &FRAMES,
        );
    }
    {
        let mut w = world(&base);
        let top = body(&mut w, S, [0.0, 3.0], 0.0, None);
        let a = body(&mut w, D, [1.0, 1.5], 0.0, Some(ball(0.25)));
        let b = body(&mut w, D, [2.0, 0.5], 0.0, Some(bx(0.3, 0.2)));
        let mut d = joint(
            &mut w,
            top,
            a,
            JointType::Distance { length: 2.0 },
            [0.0, 0.0],
            [0.0, 0.0],
        );
        let _ = w.remove_joint(impetus::JointHandle(0));
        d.damping = 0.75;
        add(&mut w, d);
        joint(
            &mut w,
            a,
            b,
            JointType::Rope { max_length: 2.0 },
            [0.0, 0.0],
            [0.2, 0.0],
        );
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &FRAMES,
        );
    }
    {
        let mut w = world(&base);
        let ground = body(&mut w, S, [0.0, 0.0], 0.0, None);
        let a = body(&mut w, D, [0.0, 0.0], 0.0, Some(bx(0.4, 0.2)));
        joint(
            &mut w,
            a,
            ground,
            JointType::Mouse {
                target: [3.0, 2.0, 0.0],
                stiffness: 50.0,
                damping: 5.0,
                max_force: 100.0,
            },
            [0.2, 0.1],
            [0.0, 0.0],
        );
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &FRAMES,
        );
    }

    // Breaking: a fixed joint whose limit the first step exceeds, and one it doesn't.
    for limit in [0.001, 1e5] {
        let mut w = world(&base);
        let top = body(&mut w, S, [0.0, 0.0], 0.0, None);
        let a = body(&mut w, D, [0.0, -3.0], 0.0, Some(ball(0.5)));
        let d = JointDesc {
            body_a: top,
            body_b: a,
            joint_type: JointType::Fixed,
            local_anchor_a: [0.0, -3.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
            damping: 0.0,
            break_force: Some(limit),
        };
        add(&mut w, d);
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &[0, 1, 2, 10],
        );
    }

    // A kinematic driver; a constant push every frame; a body at rest that falls asleep.
    {
        let mut w = world(&base);
        let k = w.add_body(BodyDesc {
            body_type: K,
            position: [0.0, 0.0, 0.0],
            linear_velocity: [1.0, 0.0, 0.0],
            angular_velocity: 0.5,
            ..BodyDesc::default()
        });
        let a = body(&mut w, D, [1.0, 0.0], 0.0, Some(ball(0.3)));
        joint(
            &mut w,
            k,
            a,
            JointType::Distance { length: 1.0 },
            [0.0, 0.0],
            [0.0, 0.0],
        );
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &FRAMES,
        );
    }
    {
        let mut w = world(&base);
        let anchor = body(&mut w, S, [0.0, 0.0], 0.0, None);
        let bob = body(&mut w, D, [0.0, -1.0], 0.0, Some(ball(0.25)));
        joint(
            &mut w,
            anchor,
            bob,
            JointType::Revolute {
                anchor: [0.0, 0.0],
                limits: None,
            },
            [0.0, 0.0],
            [0.0, 1.0],
        );
        rows(
            &mut m,
            Scene {
                world: w,
                push: Some((bob, [2.0, 0.0])),
            },
            &FRAMES,
        );
    }
    {
        let mut cfg = base.clone();
        cfg.gravity = [0.0, 0.0, 0.0];
        let mut w = world(&cfg);
        let anchor = body(&mut w, S, [0.0, 0.0], 0.0, None);
        let a = body(&mut w, D, [0.0, -1.0], 0.0, Some(ball(0.25)));
        let b = body(&mut w, D, [3.0, 0.0], 0.0, Some(ball(0.25)));
        joint(
            &mut w,
            anchor,
            a,
            JointType::Distance { length: 1.0 },
            [0.0, 0.0],
            [0.0, 0.0],
        );
        joint(
            &mut w,
            a,
            b,
            JointType::Distance { length: 3.16 },
            [0.0, 0.0],
            [0.0, 0.0],
        );
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &[0, 1, 10, 25, 29, 30, 31, 60],
        );
    }

    // Seeded scenes: two to five bodies, random joints between them and to a static anchor.
    let mut rng = Rng::new(0x57e9);
    for _ in 0..40 {
        let mut cfg = base.clone();
        cfg.sub_steps = u32::try_from(rng.int(1, 3)).unwrap_or(1);
        let mut w = world(&cfg);
        let mut hs = vec![body(&mut w, S, [0.0, 0.0], 0.0, None)];
        let n = rng.int(2, 5);
        for _ in 0..n {
            let shape = if rng.int(0, 1) == 0 {
                ball(rng.uniform(0.1, 0.6))
            } else {
                bx(rng.uniform(0.1, 0.6), rng.uniform(0.1, 0.6))
            };
            let p = [rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)];
            hs.push(body(&mut w, D, p, rng.uniform(-3.0, 3.0), Some(shape)));
        }
        let pick = |rng: &mut Rng, n: usize| {
            hs[usize::try_from(rng.int(0, i64::try_from(n).unwrap_or(1) - 1)).unwrap_or(0)]
        };
        for (i, &b) in hs.iter().enumerate().skip(1) {
            let a = pick(&mut rng, i);
            let jt = match rng.int(0, 6) {
                0 => JointType::Fixed,
                1 => JointType::Revolute {
                    anchor: [0.0, 0.0],
                    limits: if rng.int(0, 1) == 0 {
                        None
                    } else {
                        Some([rng.uniform(-1.0, 0.0), rng.uniform(0.0, 1.0)])
                    },
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
                    axis: [0.0, 1.0],
                    stiffness: rng.uniform(5.0, 80.0),
                    damping: rng.uniform(0.0, 3.0),
                },
                _ => JointType::Rope {
                    max_length: rng.uniform(1.0, 4.0),
                },
            };
            let mut d = JointDesc {
                body_a: a,
                body_b: b,
                joint_type: jt,
                local_anchor_a: [rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5)],
                local_anchor_b: [rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5)],
                motor: None,
                damping: if rng.int(0, 2) == 0 {
                    rng.uniform(0.0, 1.0)
                } else {
                    0.0
                },
                break_force: if rng.int(0, 4) == 0 {
                    Some(rng.uniform(1.0, 200.0))
                } else {
                    None
                },
            };
            if rng.int(0, 3) == 0 {
                d.motor = Some(JointMotor {
                    target_velocity: rng.uniform(-2.0, 2.0),
                    max_force: rng.uniform(1.0, 50.0),
                });
            }
            add(&mut w, d);
        }
        let k = u64::try_from(rng.int(0, 90)).unwrap_or(0);
        rows(
            &mut m,
            Scene {
                world: w,
                push: None,
            },
            &[0, k, k + 1],
        );
    }
    m
}
