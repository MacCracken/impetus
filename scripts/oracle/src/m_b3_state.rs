//! `b3_state.tsv` — the 3D backend's state (M5 part 1): add, remove, get and set through the public world API, then
//! the full backend dump (`dump3d::backend`).
//!
//! A row is a script, as `b2_state.tsv`'s. Inputs: `v:<ops>`, then each op as `e:Op.<Name>` and its arguments.
//! Outputs: each op's result, in order, then the dump. The ops and their results:
//! - `AddBody` + `BodyDesc` → the body handle; `AddCollider` + body handle + `ColliderDesc` → the collider handle;
//!   `AddJoint` + `JointDesc` → the joint handle;
//! - `ApplyForce` / `ApplyImpulse` + body handle + vector and point; `ApplyTorque` + handle + value → nothing;
//! - `RemoveBody`, `RemoveCollider`, `RemoveJoint` + handle → `i:0`, or the error (`E:`);
//! - `GetBodyState` + handle → the `BodyState` (its rotation 2·atan2(q.z, q.w)), or `E:BodyNotFound`; `SetBodyState`
//!   + handle + state and `SetBodyType` + handle + type → `i:0` or the error; `BodyCount` → the count.
//!
//! Class C: a body's rotation is `from_rotation_z` (a `sin_cos`) and a state's is an `atan2`, both correctly rounded
//! in the port (ADR 0011), so the reference is the corrected oracle; the rest is copies and the mass sums. Rows whose
//! script adds a collider or joint for a missing body (an orphan, ADR 0008 #1) are `b3::state_script_orphan`; rows
//! adding a Revolute or Prismatic joint or a motor, which the 3D API refuses (ADR 0008 #5), are
//! `b3::state_script_unsupported`. No other script adds one.

use std::f64::consts::FRAC_PI_2;

use impetus::{
    BodyDesc, BodyHandle, BodyState, BodyType, ColliderDesc, ColliderHandle, Force, Impulse,
    JointDesc, JointHandle, JointMotor, JointType, PhysicsMaterial, PhysicsWorld, Torque,
    WorldConfig,
};

use crate::dump3d;
use crate::flat::{self, handle};
use crate::grids::Rng;
use crate::tsv::{Class, Module, Tok, e, f};

#[derive(Debug, Clone, Copy)]
enum Op {
    AddBody,
    AddCollider,
    AddJoint,
    ApplyForce,
    ApplyImpulse,
    ApplyTorque,
    RemoveBody,
    RemoveCollider,
    RemoveJoint,
    GetBodyState,
    SetBodyState,
    SetBodyType,
    BodyCount,
}

/// One script step with its arguments.
enum Step {
    AddBody(BodyDesc),
    AddCollider(BodyHandle, ColliderDesc),
    AddJoint(JointDesc),
    ApplyForce(BodyHandle, Force),
    ApplyImpulse(BodyHandle, Impulse),
    ApplyTorque(BodyHandle, f64),
    RemoveBody(BodyHandle),
    RemoveCollider(ColliderHandle),
    RemoveJoint(JointHandle),
    GetBodyState(BodyHandle),
    SetBodyState(BodyHandle, BodyState),
    SetBodyType(BodyHandle, BodyType),
    BodyCount,
}

fn unit<E: std::fmt::Debug>(r: Result<(), E>) -> Vec<Tok> {
    match r {
        Ok(()) => vec![Tok::I(0)],
        Err(err) => {
            let s = format!("{err:?}");
            vec![Tok::Err(s.split('(').next().unwrap_or("").to_string())]
        }
    }
}

/// Runs a script on a fresh world: (inputs, outputs).
fn run(steps: &[Step]) -> (Vec<Tok>, Vec<Tok>) {
    let mut w = PhysicsWorld::new(WorldConfig::default());
    let mut ins = vec![Tok::V(steps.len())];
    let mut outs = Vec::new();
    for s in steps {
        match s {
            Step::AddBody(d) => {
                ins.push(e("Op", Op::AddBody));
                ins.extend(flat::body_desc(d));
                outs.push(handle(w.add_body(d.clone()).0));
            }
            Step::AddCollider(bh, d) => {
                ins.extend([e("Op", Op::AddCollider), handle(bh.0)]);
                ins.extend(flat::collider_desc(d));
                outs.push(handle(w.add_collider(*bh, d.clone()).0));
            }
            Step::AddJoint(d) => {
                ins.push(e("Op", Op::AddJoint));
                ins.extend(flat::joint_desc(d));
                outs.push(handle(w.add_joint(d.clone()).0));
            }
            Step::ApplyForce(bh, x) => {
                ins.extend([e("Op", Op::ApplyForce), handle(bh.0)]);
                ins.extend(flat::force(x));
                w.apply_force(*bh, x.clone());
            }
            Step::ApplyImpulse(bh, x) => {
                ins.extend([e("Op", Op::ApplyImpulse), handle(bh.0)]);
                ins.extend(flat::impulse(x));
                w.apply_impulse(*bh, x.clone());
            }
            Step::ApplyTorque(bh, v) => {
                ins.extend([e("Op", Op::ApplyTorque), handle(bh.0), f(*v)]);
                w.apply_torque(*bh, Torque { value: *v });
            }
            Step::RemoveBody(h) => {
                ins.extend([e("Op", Op::RemoveBody), handle(h.0)]);
                outs.extend(unit(w.remove_body(*h)));
            }
            Step::RemoveCollider(h) => {
                ins.extend([e("Op", Op::RemoveCollider), handle(h.0)]);
                outs.extend(unit(w.remove_collider(*h)));
            }
            Step::RemoveJoint(h) => {
                ins.extend([e("Op", Op::RemoveJoint), handle(h.0)]);
                outs.extend(unit(w.remove_joint(*h)));
            }
            Step::GetBodyState(h) => {
                ins.extend([e("Op", Op::GetBodyState), handle(h.0)]);
                match w.get_body_state(*h) {
                    Ok(st) => outs.extend(flat::body_state(&st)),
                    Err(err) => outs.extend(unit::<impetus::ImpetusError>(Err(err))),
                }
            }
            Step::SetBodyState(h, st) => {
                ins.extend([e("Op", Op::SetBodyState), handle(h.0)]);
                ins.extend(flat::body_state(st));
                outs.extend(unit(w.set_body_state(*h, st)));
            }
            Step::SetBodyType(h, t) => {
                ins.extend([e("Op", Op::SetBodyType), handle(h.0), e("BodyType", *t)]);
                outs.extend(unit(w.set_body_type(*h, *t)));
            }
            Step::BodyCount => {
                ins.push(e("Op", Op::BodyCount));
                outs.push(Tok::I(i64::try_from(w.body_count()).unwrap_or(-1)));
            }
        }
    }
    outs.extend(dump3d::backend(&w.oracle_dump().backend));
    (ins, outs)
}

fn row(m: &mut Module, name: &str, steps: &[Step]) {
    let unsupported = steps.iter().any(|s| match s {
        Step::AddJoint(d) => {
            d.motor.is_some() || matches!(d.joint_type, JointType::Revolute { .. } | JointType::Prismatic { .. })
        }
        _ => false,
    });
    assert_eq!(unsupported, name == "b3::state_script_unsupported", "a script's name must say whether ADR 0008 #5 maps it");
    let (ins, outs) = run(steps);
    m.row(name, Class::C, &ins, &outs);
}

fn desc(body_type: BodyType, pos: [f64; 3], rot: f64) -> BodyDesc {
    BodyDesc {
        body_type,
        position: pos,
        rotation: rot,
        ..BodyDesc::default()
    }
}

fn coll(shape: impetus::ColliderShape, offset: [f64; 3], density: f64) -> ColliderDesc {
    ColliderDesc {
        shape,
        offset,
        material: PhysicsMaterial {
            density,
            ..PhysicsMaterial::default()
        },
        is_sensor: false,
        mass: None,
        collision_layer: u32::MAX,
        collision_mask: u32::MAX,
    }
}

fn joint(a: BodyHandle, b: BodyHandle, jt: JointType) -> JointDesc {
    JointDesc {
        body_a: a,
        body_b: b,
        joint_type: jt,
        local_anchor_a: [0.5, 0.0],
        local_anchor_b: [-0.5, 0.0],
        motor: None,
        damping: 0.0,
        break_force: None,
    }
}

fn ball(r: f64) -> impetus::ColliderShape {
    impetus::ColliderShape::Ball { radius: r }
}

fn bx(hx: f64, hy: f64, hz: f64) -> impetus::ColliderShape {
    impetus::ColliderShape::Box {
        half_extents: [hx, hy, hz],
    }
}

fn capsule(hh: f64, r: f64) -> impetus::ColliderShape {
    impetus::ColliderShape::Capsule {
        half_height: hh,
        radius: r,
    }
}

/// A seeded 3D shape: every kind, sizes log-uniform over about [0.03, 4).
fn random_shape(rng: &mut Rng) -> impetus::ColliderShape {
    let kind = rng.int(0, 6);
    let mut s = || rng.log_uniform(-1.3, 0.7);
    match kind {
        0 => ball(s()),
        1 => bx(s(), s(), s()),
        2 => capsule(s(), s()),
        3 => impetus::ColliderShape::ConvexHull {
            points: vec![[0.0, 0.0, 0.0], [s(), 0.0, 0.0], [0.0, s(), 0.0], [0.0, 0.0, s()], [s(), s(), s()]],
        },
        4 => impetus::ColliderShape::Segment {
            a: [-s(), 0.0, s()],
            b: [s(), s(), 0.0],
        },
        5 => impetus::ColliderShape::TriMesh {
            vertices: vec![[0.0, 0.0, 0.0], [s(), 0.0, 0.0], [0.0, s(), 0.0], [0.0, 0.0, s()]],
            indices: vec![[0, 1, 2], [0, 2, 3]],
        },
        _ => impetus::ColliderShape::Heightfield {
            heights: vec![0.0, s(), s()],
            scale: [s(), 1.0, s()],
        },
    }
}

/// A handle as the world will hand it out: slot `index` at `generation`.
fn bh(index: u64, generation: u64) -> BodyHandle {
    BodyHandle((generation << 32) | index)
}

fn ch(index: u64, generation: u64) -> ColliderHandle {
    ColliderHandle((generation << 32) | index)
}

fn jh(index: u64, generation: u64) -> JointHandle {
    JointHandle((generation << 32) | index)
}

/// A seeded joint type the 3D API accepts: no Revolute or Prismatic (ADR 0008 #5; the unsupported scripts cover them).
fn random_joint_type(rng: &mut Rng) -> JointType {
    match rng.int(0, 5) {
        0 => JointType::Fixed,
        1 => JointType::Spring {
            rest_length: rng.uniform(0.5, 3.0),
            stiffness: rng.uniform(1.0, 100.0),
            damping: rng.uniform(0.0, 5.0),
        },
        2 => JointType::Distance {
            length: rng.uniform(0.5, 3.0),
        },
        3 => JointType::Wheel {
            axis: [0.0, 1.0],
            stiffness: rng.uniform(1.0, 100.0),
            damping: rng.uniform(0.0, 5.0),
        },
        4 => JointType::Rope {
            max_length: rng.uniform(0.5, 4.0),
        },
        _ => JointType::Mouse {
            target: [rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0), 0.0],
            stiffness: rng.uniform(1.0, 100.0),
            damping: rng.uniform(0.0, 5.0),
            max_force: rng.uniform(1.0, 1000.0),
        },
    }
}

fn random_type(rng: &mut Rng) -> BodyType {
    match rng.int(0, 5) {
        0 => BodyType::Static,
        1 => BodyType::Kinematic,
        _ => BodyType::Dynamic,
    }
}

fn random_point(rng: &mut Rng) -> Option<[f64; 3]> {
    if rng.int(0, 1) == 1 {
        Some([rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)])
    } else {
        None
    }
}

/// A seeded script of up to `len` ops over live and stale handles; every collider and joint gets live
/// bodies.
#[allow(clippy::too_many_lines)] // one arm per op
fn random_script(rng: &mut Rng, len: usize) -> Vec<Step> {
    let mut w = PhysicsWorld::new(WorldConfig::default());
    let mut bodies: Vec<BodyHandle> = Vec::new();
    let mut colls: Vec<ColliderHandle> = Vec::new();
    let mut joints: Vec<JointHandle> = Vec::new();
    let mut gone_b: Vec<BodyHandle> = vec![bh(7, 0)];
    let mut gone_c: Vec<ColliderHandle> = vec![ch(9, 0)];
    let mut gone_j: Vec<JointHandle> = vec![jh(3, 0)];
    let mut steps = Vec::new();
    let pick = |rng: &mut Rng, n: usize| {
        usize::try_from(rng.int(0, i64::try_from(n).unwrap_or(1) - 1)).unwrap_or(0)
    };
    while steps.len() < len {
        let r = rng.int(0, 99);
        let step = if r < 18 || bodies.is_empty() {
            let mut d = desc(
                random_type(rng),
                [rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0)],
                rng.uniform(-3.0, 3.0),
            );
            d.linear_velocity = [rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0)];
            d.angular_velocity = rng.uniform(-1.0, 1.0);
            d.linear_damping = rng.uniform(0.0, 0.5);
            d.angular_damping = rng.uniform(0.0, 0.5);
            d.fixed_rotation = rng.int(0, 4) == 0;
            d.gravity_scale = if rng.int(0, 2) == 0 {
                Some(rng.uniform(-1.0, 2.0))
            } else {
                None
            };
            let h = w.add_body(d.clone());
            bodies.push(h);
            Step::AddBody(d)
        } else if r < 42 {
            let b = bodies[pick(rng, bodies.len())];
            let mut d = coll(
                random_shape(rng),
                [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)],
                rng.log_uniform(-1.0, 1.0),
            );
            d.is_sensor = rng.int(0, 5) == 0;
            d.mass = if rng.int(0, 4) == 0 {
                Some(rng.uniform(0.0, 5.0))
            } else {
                None
            };
            d.collision_layer = u32::try_from(rng.int(1, 255)).unwrap_or(1);
            colls.push(w.add_collider(b, d.clone()));
            Step::AddCollider(b, d)
        } else if r < 50 && bodies.len() >= 2 {
            let a = bodies[pick(rng, bodies.len())];
            let b = bodies[pick(rng, bodies.len())];
            let mut d = joint(a, b, random_joint_type(rng));
            d.local_anchor_a = [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)];
            d.local_anchor_b = [rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0)];
            d.damping = rng.uniform(0.0, 1.0);
            d.break_force = if rng.int(0, 2) == 0 {
                Some(rng.uniform(10.0, 100.0))
            } else {
                None
            };
            joints.push(w.add_joint(d.clone()));
            Step::AddJoint(d)
        } else if r < 58 {
            let b = bodies[pick(rng, bodies.len())];
            let x = Force {
                vector: [rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0), rng.uniform(-10.0, 10.0)],
                point: random_point(rng),
            };
            w.apply_force(b, x.clone());
            Step::ApplyForce(b, x)
        } else if r < 65 {
            let b = bodies[pick(rng, bodies.len())];
            let x = Impulse {
                vector: [rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0)],
                point: random_point(rng),
            };
            w.apply_impulse(b, x.clone());
            Step::ApplyImpulse(b, x)
        } else if r < 69 {
            let b = bodies[pick(rng, bodies.len())];
            let v = rng.uniform(-5.0, 5.0);
            w.apply_torque(b, Torque { value: v });
            Step::ApplyTorque(b, v)
        } else if r < 74 {
            if rng.int(0, 4) == 0 {
                Step::RemoveBody(gone_b[pick(rng, gone_b.len())])
            } else {
                let i = pick(rng, bodies.len());
                let h = bodies.remove(i);
                let _ = w.remove_body(h);
                gone_b.push(h);
                // Its colliders and joints go with it: forget them so later ops pick live ones.
                let live_c = w
                    .oracle_dump()
                    .backend
                    .colliders
                    .iter()
                    .map(|c| c.handle)
                    .collect::<Vec<_>>();
                colls.retain(|c| live_c.contains(&c.0));
                let live_j = w
                    .oracle_dump()
                    .backend
                    .joints
                    .iter()
                    .map(|j| j.handle)
                    .collect::<Vec<_>>();
                joints.retain(|j| live_j.contains(&j.0));
                Step::RemoveBody(h)
            }
        } else if r < 79 {
            if colls.is_empty() || rng.int(0, 4) == 0 {
                Step::RemoveCollider(gone_c[pick(rng, gone_c.len())])
            } else {
                let i = pick(rng, colls.len());
                let h = colls.remove(i);
                let _ = w.remove_collider(h);
                gone_c.push(h);
                Step::RemoveCollider(h)
            }
        } else if r < 83 {
            if joints.is_empty() || rng.int(0, 4) == 0 {
                Step::RemoveJoint(gone_j[pick(rng, gone_j.len())])
            } else {
                let i = pick(rng, joints.len());
                let h = joints.remove(i);
                let _ = w.remove_joint(h);
                gone_j.push(h);
                Step::RemoveJoint(h)
            }
        } else if r < 89 {
            let b = if rng.int(0, 5) == 0 {
                gone_b[pick(rng, gone_b.len())]
            } else {
                bodies[pick(rng, bodies.len())]
            };
            Step::GetBodyState(b)
        } else if r < 94 {
            let b = if rng.int(0, 5) == 0 {
                gone_b[pick(rng, gone_b.len())]
            } else {
                bodies[pick(rng, bodies.len())]
            };
            let st = BodyState {
                handle: b,
                body_type: BodyType::Dynamic,
                position: [rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0), rng.uniform(-5.0, 5.0)],
                rotation: rng.uniform(-3.0, 3.0),
                linear_velocity: [rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0), rng.uniform(-2.0, 2.0)],
                angular_velocity: rng.uniform(-1.0, 1.0),
                is_sleeping: rng.int(0, 1) == 1,
            };
            let _ = w.set_body_state(b, &st);
            Step::SetBodyState(b, st)
        } else if r < 98 {
            let b = if rng.int(0, 5) == 0 {
                gone_b[pick(rng, gone_b.len())]
            } else {
                bodies[pick(rng, bodies.len())]
            };
            let t = random_type(rng);
            let _ = w.set_body_type(b, t);
            Step::SetBodyType(b, t)
        } else {
            Step::BodyCount
        };
        steps.push(step);
    }
    steps
}

#[allow(clippy::too_many_lines)] // the hand scripts
pub fn emit() -> Module {
    let mut m = Module::new("b3_state");
    let s = "b3::state_script";
    let dynamic = BodyType::Dynamic;

    // An empty world; one body; slot reuse after a removal (generation 1), the free list LIFO.
    row(&mut m, s, &[Step::BodyCount]);
    row(
        &mut m,
        s,
        &[
            Step::AddBody(desc(dynamic, [1.0, 2.0, 0.5], 0.5)),
            Step::BodyCount,
        ],
    );
    row(
        &mut m,
        s,
        &[
            Step::AddBody(desc(dynamic, [0.0, 0.0, 0.5], 0.0)),
            Step::AddBody(desc(dynamic, [1.0, 0.0, 0.5], 0.0)),
            Step::AddBody(desc(dynamic, [2.0, 0.0, 0.5], 0.0)),
            Step::RemoveBody(bh(1, 0)),
            Step::RemoveBody(bh(0, 0)),
            Step::AddBody(desc(BodyType::Static, [5.0, 5.0, 0.5], 1.0)),
            Step::AddBody(desc(BodyType::Kinematic, [6.0, 5.0, 0.5], 2.0)),
            Step::AddBody(desc(dynamic, [7.0, 5.0, 0.5], 3.0)),
            Step::RemoveBody(bh(1, 0)),
            Step::BodyCount,
        ],
    );

    // Mass: colliders add up (the parallel-axis shift), a removal recomputes, set_body_type gives a
    // static body its mass and zeroes kinematic and static velocities.
    let mut moving = desc(dynamic, [0.0, 0.0, 0.5], 0.0);
    moving.linear_velocity = [3.0, -1.0, 0.0];
    moving.angular_velocity = 2.0;
    row(
        &mut m,
        s,
        &[
            Step::AddBody(moving.clone()),
            Step::AddCollider(bh(0, 0), coll(ball(0.5), [0.0, 0.0, -0.25], 1.0)),
            Step::AddCollider(bh(0, 0), coll(bx(1.0, 0.5, 0.4), [2.0, 0.0, -0.25], 2.0)),
            Step::AddCollider(bh(0, 0), coll(capsule(0.5, 0.25), [0.0, -1.5, -0.25], 1.0)),
            Step::RemoveCollider(ch(1, 0)),
            Step::RemoveCollider(ch(1, 0)),
            Step::GetBodyState(bh(0, 0)),
            Step::SetBodyType(bh(0, 0), BodyType::Kinematic),
            Step::GetBodyState(bh(0, 0)),
            Step::SetBodyType(bh(0, 0), BodyType::Static),
            Step::SetBodyType(bh(0, 0), dynamic),
            Step::AddCollider(bh(0, 0), coll(ball(1.0), [0.5, 0.5, -0.25], 0.5)),
        ],
    );
    row(
        &mut m,
        s,
        &[
            Step::AddBody(desc(BodyType::Static, [0.0, -1.0, 0.5], 0.0)),
            Step::AddCollider(bh(0, 0), coll(bx(5.0, 0.5, 0.4), [0.0, 0.0, -0.25], 1.0)),
            Step::SetBodyType(bh(0, 0), dynamic),
            Step::SetBodyType(bh(7, 0), dynamic),
            Step::GetBodyState(bh(7, 0)),
        ],
    );

    // Forces, impulses and torques: points make torque, impulses need a dynamic body with mass, every
    // one of them wakes the body.
    let mut sleeper = desc(dynamic, [0.0, 0.0, 0.5], 0.0);
    sleeper.fixed_rotation = false;
    row(
        &mut m,
        s,
        &[
            Step::AddBody(sleeper.clone()),
            Step::AddBody(desc(BodyType::Static, [0.0, 0.0, 0.5], 0.0)),
            Step::AddBody(desc(dynamic, [3.0, 0.0, 0.5], 0.0)),
            Step::AddCollider(bh(0, 0), coll(bx(0.5, 0.5, 0.4), [0.0, 0.0, -0.25], 1.0)),
            Step::AddCollider(bh(1, 0), coll(bx(0.5, 0.5, 0.4), [0.0, 0.0, -0.25], 1.0)),
            Step::ApplyForce(
                bh(0, 0),
                Force {
                    vector: [1.0, 2.0, 0.0],
                    point: Some([0.5, -0.5, 0.25]),
                },
            ),
            Step::ApplyForce(
                bh(0, 0),
                Force {
                    vector: [-3.0, 0.5, 9.0],
                    point: None,
                },
            ),
            Step::ApplyImpulse(
                bh(0, 0),
                Impulse {
                    vector: [2.0, 0.0, 0.0],
                    point: Some([0.0, 0.5, -0.75]),
                },
            ),
            Step::ApplyImpulse(
                bh(1, 0),
                Impulse {
                    vector: [2.0, 0.0, 0.0],
                    point: None,
                },
            ),
            Step::ApplyImpulse(
                bh(2, 0),
                Impulse {
                    vector: [2.0, 0.0, 0.0],
                    point: None,
                },
            ),
            Step::ApplyTorque(bh(0, 0), 4.5),
            Step::ApplyTorque(bh(1, 0), 1.0),
            Step::ApplyForce(
                bh(9, 0),
                Force {
                    vector: [1.0, 1.0, 0.0],
                    point: None,
                },
            ),
            Step::GetBodyState(bh(0, 0)),
        ],
    );

    // Joints: the reference angle from the bodies' rotations; removals wake both bodies; a body's
    // removal takes its joints and colliders with it.
    let mut first = joint(
        bh(0, 0),
        bh(1, 0),
        JointType::Spring {
            rest_length: 1.0,
            stiffness: 20.0,
            damping: 0.5,
        },
    );
    first.break_force = Some(50.0);
    first.damping = 0.25;
    row(
        &mut m,
        s,
        &[
            Step::AddBody(desc(dynamic, [0.0, 0.0, 0.5], 0.3)),
            Step::AddBody(desc(dynamic, [1.0, 0.0, 0.5], -FRAC_PI_2)),
            Step::AddBody(desc(dynamic, [2.0, 0.0, 0.5], 1.25)),
            Step::AddCollider(bh(0, 0), coll(ball(0.5), [0.0, 0.0, -0.25], 1.0)),
            Step::AddCollider(bh(1, 0), coll(ball(0.5), [0.0, 0.0, -0.25], 1.0)),
            Step::AddCollider(bh(2, 0), coll(ball(0.5), [0.0, 0.0, -0.25], 1.0)),
            Step::AddJoint(first),
            Step::AddJoint(joint(
                bh(1, 0),
                bh(2, 0),
                JointType::Distance { length: 1.0 },
            )),
            Step::AddJoint(joint(
                bh(2, 0),
                bh(0, 0),
                JointType::Rope { max_length: 3.0 },
            )),
            Step::RemoveJoint(jh(0, 0)),
            Step::RemoveJoint(jh(0, 0)),
            Step::RemoveBody(bh(1, 0)),
            Step::AddJoint(joint(bh(0, 0), bh(2, 0), JointType::Fixed)),
            Step::RemoveCollider(ch(1, 0)),
            Step::BodyCount,
        ],
    );

    // Teleports wake; a missing body is an error for get, set and set_body_type alike.
    row(
        &mut m,
        s,
        &[
            Step::AddBody(desc(dynamic, [0.0, 0.0, 0.5], 0.0)),
            Step::SetBodyState(
                bh(0, 0),
                BodyState {
                    handle: bh(5, 5),
                    body_type: BodyType::Static,
                    position: [4.0, -2.0, 7.0],
                    rotation: 1.0,
                    linear_velocity: [0.5, 0.25, 9.0],
                    angular_velocity: -0.75,
                    is_sleeping: true,
                },
            ),
            Step::GetBodyState(bh(0, 0)),
            Step::RemoveBody(bh(0, 0)),
            Step::GetBodyState(bh(0, 0)),
            Step::SetBodyState(
                bh(0, 0),
                BodyState {
                    handle: bh(0, 0),
                    body_type: dynamic,
                    position: [0.0; 3],
                    rotation: 0.0,
                    linear_velocity: [0.0; 3],
                    angular_velocity: 0.0,
                    is_sleeping: false,
                },
            ),
            Step::SetBodyType(bh(0, 0), BodyType::Static),
            Step::RemoveBody(bh(0, 0)),
            Step::RemoveCollider(ch(0, 0)),
            Step::RemoveJoint(jh(0, 0)),
        ],
    );

    // Orphans (ADR 0008 #1): a collider or a joint for a body that isn't there.
    let o = "b3::state_script_orphan";
    row(
        &mut m,
        o,
        &[Step::AddCollider(
            bh(0, 0),
            coll(ball(0.5), [0.0, 0.0, -0.25], 1.0),
        )],
    );
    row(
        &mut m,
        o,
        &[
            Step::AddBody(desc(dynamic, [0.0, 0.0, 0.5], 0.0)),
            Step::RemoveBody(bh(0, 0)),
            Step::AddCollider(bh(0, 0), coll(ball(0.5), [0.0, 0.0, -0.25], 1.0)),
            Step::AddBody(desc(dynamic, [0.0, 0.0, 0.5], 0.0)),
        ],
    );
    row(
        &mut m,
        o,
        &[
            Step::AddBody(desc(dynamic, [0.0, 0.0, 0.5], 0.0)),
            Step::AddJoint(joint(bh(0, 0), bh(4, 0), JointType::Fixed)),
        ],
    );

    // Joints the 3D API refuses (ADR 0008 #5; Rust accepts them and does nothing with them but their damping): a
    // Revolute and a Prismatic joint, a motor on a Distance joint, then a Fixed joint, whose handle shows the refused
    // ones took no slot.
    let u = "b3::state_script_unsupported";
    let mut on_distance = joint(bh(0, 0), bh(1, 0), JointType::Distance { length: 1.0 });
    on_distance.motor = Some(JointMotor {
        target_velocity: 1.5,
        max_force: 20.0,
    });
    for unsupported in [
        joint(
            bh(0, 0),
            bh(1, 0),
            JointType::Revolute {
                anchor: [0.0, 0.0],
                limits: Some([-0.5, 0.5]),
            },
        ),
        joint(
            bh(0, 0),
            bh(1, 0),
            JointType::Prismatic {
                axis: [1.0, 0.0],
                limits: None,
            },
        ),
        on_distance,
    ] {
        row(
            &mut m,
            u,
            &[
                Step::AddBody(desc(dynamic, [0.0, 0.0, 0.5], 0.3)),
                Step::AddBody(desc(dynamic, [1.0, 0.0, 0.5], -0.4)),
                Step::AddJoint(unsupported),
                Step::AddJoint(joint(bh(0, 0), bh(1, 0), JointType::Fixed)),
                Step::RemoveJoint(jh(0, 0)),
            ],
        );
    }

    // Seeded scripts.
    let mut rng = Rng::new(0x5a7e_0003);
    for k in 0..400 {
        let len = 5 + (k % 26);
        let steps = random_script(&mut rng, len);
        row(&mut m, s, &steps);
    }
    m
}
