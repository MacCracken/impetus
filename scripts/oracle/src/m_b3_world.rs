//! `b3_world.tsv` — single-step rows (class S, port plan §5) of `PhysicsWorld::step` in 3D (M5 part 4): the frame
//! counter, the 3D backend's sub-steps and events, then the particles (emitter spawns with the R3 spread, gravity,
//! radial and directional fields, drag, damping, collisions with the 3D colliders, sub-emitters, removal of the
//! dead).
//!
//! A row is `world_step.tsv`'s with the 3D backend: the world config (`flat::config`, its `step` the frame count so
//! far), the world's own state (`dumpcore::world`) and the 3D backend dump (`dump3d::backend`), taken after that
//! frame's applied forces; its outputs are the same three after `step()`, then the frame's collision events. Rust's
//! 3D world builds bodies rotated about z only, so the scenes that need other orientations spin dynamic bodies with
//! an off-centre impulse before the first frame (with gravity scale 0, so they tumble in place under the falling
//! particles). Class C: the radial field divides by a `powf`, the sub-emitters spread by `cos` and `sin`, and the
//! backend rotates and normalises; the reference is the corrected oracle (`validate.py` transcribes the world step
//! and writes `b3_world.cr.tsv` where glibc misrounds a call).

use impetus::{
    BodyDesc, BodyHandle, BodyType, BroadphaseKind, ColliderDesc, ColliderShape, Force, ForceField, Impulse,
    JointDesc, JointType, Particle, ParticleEmitter, PhysicsMaterial, PhysicsWorld, SubEmitter, WorldConfig,
};

use crate::dump3d;
use crate::dumpcore;
use crate::flat;
use crate::grids::Rng;
use crate::tsv::{Class, Module, Tok};

const S: BodyType = BodyType::Static;
const D: BodyType = BodyType::Dynamic;
const K: BodyType = BodyType::Kinematic;

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

/// An octahedron of half-diagonal r: particles have no contact with a hull.
fn octahedron(r: f64) -> ColliderShape {
    ColliderShape::ConvexHull {
        points: vec![[r, 0.0, 0.0], [-r, 0.0, 0.0], [0.0, r, 0.0], [0.0, -r, 0.0], [0.0, 0.0, r], [0.0, 0.0, -r]],
    }
}

/// Adds a collider of `shape` at `offset` (layer 1, every mask bit) to body h.
fn coll(w: &mut PhysicsWorld, h: BodyHandle, shape: ColliderShape, offset: [f64; 3], sensor: bool) {
    w.add_collider(
        h,
        ColliderDesc {
            shape,
            offset,
            material: PhysicsMaterial::default(),
            is_sensor: sensor,
            mass: None,
            collision_layer: 1,
            collision_mask: u32::MAX,
        },
    );
}

/// A body of type `t` at `pos`, rotated `rot` about z, with one centred collider of `shape`.
fn body(w: &mut PhysicsWorld, t: BodyType, pos: [f64; 3], rot: f64, shape: ColliderShape, sensor: bool) -> BodyHandle {
    let h = w.add_body(BodyDesc {
        body_type: t,
        position: pos,
        rotation: rot,
        ..BodyDesc::default()
    });
    coll(w, h, shape, [0.0, 0.0, 0.0], sensor);
    h
}

/// A dynamic body at `pos` that ignores gravity, with a collider of `shape` at `offset`, set tumbling by an impulse
/// `v` at the world point `pos + at` (a linear kick and a spin about an axis off every coordinate axis).
fn tumbler(w: &mut PhysicsWorld, pos: [f64; 3], shape: ColliderShape, offset: [f64; 3], v: [f64; 3], at: [f64; 3]) -> BodyHandle {
    let h = w.add_body(BodyDesc {
        body_type: D,
        position: pos,
        gravity_scale: Some(0.0),
        ..BodyDesc::default()
    });
    coll(w, h, shape, offset, false);
    let p = [pos[0] + at[0], pos[1] + at[1], pos[2] + at[2]];
    w.apply_impulse(h, Impulse::at_point(v[0], v[1], v[2], p[0], p[1], p[2]));
    h
}

/// A scene: a built world, and a force applied to a body before each frame (if any).
struct Scene {
    world: PhysicsWorld,
    push: Option<(BodyHandle, [f64; 3])>,
}

fn scene(world: PhysicsWorld) -> Scene {
    Scene { world, push: None }
}

/// The world's tokens: the config, its own state, the 3D backend.
fn tokens(w: &PhysicsWorld) -> Vec<Tok> {
    let d = w.oracle_dump();
    let mut t = flat::config(&d.config);
    t.extend(dumpcore::world(&d));
    t.extend(dump3d::backend(&d.backend));
    t
}

/// Runs a scene to the last frame asked, emitting a row for each frame k in `frames`.
fn rows(m: &mut Module, mut sc: Scene, frames: &[u64]) {
    let last = frames.iter().copied().max().unwrap_or(0);
    for k in 0..=last {
        if let Some((h, f)) = sc.push {
            sc.world.apply_force(h, Force::new(f[0], f[1], f[2]));
        }
        let take = frames.contains(&k);
        let ins = if take { tokens(&sc.world) } else { Vec::new() };
        sc.world.step();
        if take {
            let mut outs = tokens(&sc.world);
            let ev = sc.world.collision_events();
            outs.push(Tok::V(ev.len()));
            for x in ev {
                outs.extend(flat::event(x));
            }
            m.row("b3::world_step", Class::C, &ins, &outs);
        }
    }
}

const FRAMES: [u64; 7] = [0, 1, 2, 5, 10, 30, 60];

fn radial(c: [f64; 3], strength: f64, falloff: f64, radius: f64) -> ForceField {
    ForceField::Radial {
        center: c,
        strength,
        falloff,
        radius,
    }
}

/// `n` particles at rest on the circle of radius `r` about `c` in the plane of axes `i` and `j`, lifetime `life`.
fn ring(w: &mut PhysicsWorld, n: u32, c: [f64; 3], r: f64, (i, j): (usize, usize), life: f64) {
    for k in 0..n {
        // Rational points on the circle (no transcendental).
        let t = -2.0 + 4.0 * f64::from(k) / f64::from(n);
        let d = 1.0 + t * t;
        let mut p = c;
        p[i] += r * (1.0 - t * t) / d;
        p[j] += r * (2.0 * t) / d;
        w.spawn_particle(Particle::new(p, [0.0, 0.0, 0.0], life));
    }
}

/// Three rings about `c`, one in each coordinate plane.
fn shell(w: &mut PhysicsWorld, n: u32, c: [f64; 3], r: f64, life: f64) {
    ring(w, n, c, r, (0, 1), life);
    ring(w, n, c, r, (0, 2), life);
    ring(w, n, c, r, (1, 2), life);
}

#[allow(clippy::too_many_lines)] // the scenes
pub fn emit() -> Module {
    let mut m = Module::new("b3_world");
    let base = WorldConfig::default();
    let zero_g = WorldConfig {
        gravity: [0.0, 0.0, 0.0],
        ..base.clone()
    };

    // An emitter over a floor: spawns at 60/s with spread on every axis, collisions, expiry; an inactive emitter and
    // two with no valid rate.
    {
        let mut w = PhysicsWorld::new(base.clone());
        body(&mut w, S, [0.0, -0.5, 0.0], 0.0, bx(10.0, 0.5, 10.0), false);
        w.add_emitter(
            ParticleEmitter::new([0.0, 2.0, 0.0], [0.5, 3.0, -0.4], 60.0)
                .with_spread([2.0, 1.0, 1.5])
                .with_lifetime(1.5)
                .with_radius(0.08)
                .with_damping(0.2),
        );
        let mut off = ParticleEmitter::new([1.0, 1.0, 0.0], [0.0, 1.0, 0.0], 100.0);
        off.active = false;
        w.add_emitter(off);
        w.add_emitter(ParticleEmitter::new([0.0, 1.0, 0.0], [0.0, 1.0, 0.0], f64::NAN));
        w.add_emitter(ParticleEmitter::new([0.0, 1.0, 0.0], [0.0, 1.0, 0.0], -3.0));
        rows(&mut m, scene(w), &[0, 1, 2, 10, 60, 100, 120]);
    }
    // A fast emitter (several spawns a frame) with gravity scale 0 and two sub-steps.
    {
        let mut cfg = base.clone();
        cfg.sub_steps = 2;
        let mut w = PhysicsWorld::new(cfg);
        w.add_emitter(
            ParticleEmitter::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.5], 330.0)
                .with_spread([0.5, 0.5, 0.5])
                .with_lifetime(0.2)
                .with_gravity_scale(0.0),
        );
        rows(&mut m, scene(w), &[0, 1, 2, 5, 11, 12, 13]);
    }

    // Radial fields on particles off the z = 0 plane: attraction at falloffs 1 and 2, repulsion, a reach limit.
    for (falloff, strength, radius) in [(1.0, 4.0, 0.0), (2.0, 4.0, 0.0), (2.0, -6.0, 0.0), (1.0, 4.0, 1.2)] {
        let mut w = PhysicsWorld::new(zero_g.clone());
        w.add_force_field(radial([0.0, 0.0, 0.3], strength, falloff, radius));
        shell(&mut w, 8, [0.0, 0.0, 0.0], 1.5, 10.0);
        ring(&mut w, 6, [0.3, -0.2, 0.5], 0.4, (0, 2), 10.0);
        // One exactly at the centre: the field skips it.
        w.spawn_particle(Particle::new([0.0, 0.0, 0.3], [0.0, 0.0, 0.0], 10.0));
        rows(&mut m, scene(w), &FRAMES);
    }
    // Directional fields: a wind box thin in z (particles in and out of it along z), with gravity and a radial field.
    {
        let mut w = PhysicsWorld::new(base.clone());
        w.add_force_field(ForceField::Directional {
            force: [3.0, 1.0, -2.0],
            min: [-1.0, -1.0, -0.5],
            max: [1.0, 1.0, 0.5],
        });
        w.add_force_field(radial([2.0, 0.0, -1.0], 2.0, 2.0, 0.0));
        ring(&mut w, 10, [0.0, 0.0, 0.0], 0.9, (0, 2), 10.0);
        ring(&mut w, 6, [0.0, 0.0, 0.0], 0.9, (1, 2), 10.0);
        w.spawn_particle(Particle::new([0.0, 0.0, 0.5], [0.0, 0.0, 0.0], 10.0));
        rows(&mut m, scene(w), &FRAMES);
    }

    // Collisions with every 3D shape: particles falling onto a box, a box turned about z, a ball, a turned capsule, a
    // segment, a hull (no contact for hulls), a sensor (ignored) and an offset box on a turned body; across z and
    // with restitutions 0, 0.3 and 1. One starts inside a box, one has radius 0 (skipped), one meets a box's z face.
    {
        let mut w = PhysicsWorld::new(base.clone());
        body(&mut w, S, [-6.0, 0.0, 0.0], 0.0, bx(1.0, 0.5, 0.6), false);
        body(&mut w, S, [-3.0, 0.0, 0.0], 0.4, bx(1.0, 0.3, 0.8), false);
        body(&mut w, S, [0.0, 0.0, 0.0], 0.0, ball(0.7), false);
        body(&mut w, S, [3.0, 0.0, 0.0], 1.2, capsule(0.8, 0.3), false);
        body(&mut w, S, [6.0, 0.0, 0.0], 0.1, segment([-1.0, 0.0, -0.5], [1.0, 0.2, 0.5]), false);
        body(&mut w, S, [9.0, 0.0, 0.0], 0.0, octahedron(0.8), false);
        body(&mut w, S, [12.0, 0.0, 0.0], 0.0, bx(1.0, 0.5, 1.0), true);
        let turned = w.add_body(BodyDesc {
            body_type: S,
            position: [15.0, 0.0, 0.0],
            rotation: -0.7,
            ..BodyDesc::default()
        });
        coll(&mut w, turned, bx(0.8, 0.25, 0.5), [0.3, 0.1, 0.2], false);
        for (k, rest) in [0.0, 0.3, 1.0].into_iter().enumerate() {
            let dx = 0.25 * f64::from(u32::try_from(k).unwrap_or(0)) - 0.25;
            let dz = 0.2 * f64::from(u32::try_from(k).unwrap_or(0)) - 0.2;
            for cx in [-6.0, -3.0, 0.0, 3.0, 6.0, 9.0, 12.0, 15.0] {
                w.spawn_particle(
                    Particle::new([cx + dx, 1.5, dz], [0.0, -1.0, 0.0], 10.0)
                        .with_radius(0.1)
                        .with_restitution(rest),
                );
            }
        }
        w.spawn_particle(Particle::new([-6.2, 0.1, 0.3], [0.0, 0.0, 0.0], 10.0).with_radius(0.05));
        w.spawn_particle(Particle::new([0.0, 0.69, 0.0], [0.0, -2.0, 0.0], 10.0).with_radius(0.0));
        w.spawn_particle(
            Particle::new([-6.0, 0.0, 1.2], [0.0, 0.0, -3.0], 10.0)
                .with_radius(0.08)
                .with_gravity_scale(0.0)
                .with_restitution(0.5),
        );
        rows(&mut m, scene(w), &[0, 1, 2, 5, 10, 20, 30, 40, 60]);
    }
    // Streams along z: weightless particles shot at a box's z face, at a ball, at a box turned about z and at a
    // tumbling box, so that every frame has hits whose normals have a z component.
    {
        let mut w = PhysicsWorld::new(base.clone());
        body(&mut w, S, [-4.0, 1.0, 0.0], 0.0, bx(1.0, 1.0, 0.2), false);
        body(&mut w, S, [-1.0, 1.0, 0.0], 0.0, ball(0.6), false);
        body(&mut w, S, [2.0, 1.0, 0.0], 0.6, bx(0.7, 0.5, 0.3), false);
        tumbler(&mut w, [5.0, 1.0, 0.0], bx(0.6, 0.4, 0.5), [0.0, 0.0, 0.0], [0.0, 0.3, 0.0], [0.4, 0.0, 0.3]);
        for (x, vx, rest) in [(-4.0, 0.0, 0.5), (-1.0, 0.4, 1.0), (2.0, -0.3, 0.0), (5.0, 0.2, 0.7)] {
            let mut e = ParticleEmitter::new([x, 1.0, 1.2], [vx, 0.0, -4.0], 50.0)
                .with_spread([0.8, 0.8, 0.3])
                .with_lifetime(1.5)
                .with_radius(0.06)
                .with_gravity_scale(0.0);
            e.particle_restitution = rest;
            w.add_emitter(e);
        }
        rows(&mut m, scene(w), &[0, 15, 20, 25, 30, 40, 50, 60]);
    }
    // Tumbling bodies: a box, a capsule, a ball on an offset collider and a segment, each spun about a skew axis,
    // under an emitter's stream of particles.
    {
        let mut w = PhysicsWorld::new(base.clone());
        tumbler(&mut w, [-3.0, 0.0, 0.0], bx(0.6, 0.3, 0.4), [0.0, 0.0, 0.0], [0.0, 0.4, 0.3], [0.3, 0.2, -0.4]);
        tumbler(&mut w, [0.0, 0.0, 0.0], capsule(0.5, 0.25), [0.0, 0.0, 0.0], [0.2, 0.0, 0.5], [-0.2, 0.4, 0.1]);
        tumbler(&mut w, [3.0, 0.0, 0.0], ball(0.4), [0.3, 0.0, 0.2], [0.0, 0.0, -0.6], [0.5, 0.3, 0.0]);
        let mut stream = ParticleEmitter::new([0.0, 3.0, 0.0], [0.0, -2.0, 0.0], 90.0)
            .with_spread([4.0, 0.5, 0.6])
            .with_lifetime(2.0)
            .with_radius(0.07);
        stream.particle_restitution = 0.4;
        w.add_emitter(stream);
        rows(&mut m, scene(w), &[0, 1, 2, 10, 30, 60, 90]);
    }
    // Particles on a moving kinematic paddle (its collider offset in x and z) and around a dynamic box on the ground.
    {
        let mut w = PhysicsWorld::new(base.clone());
        body(&mut w, S, [0.0, -0.5, 0.0], 0.0, bx(10.0, 0.5, 10.0), false);
        let k = w.add_body(BodyDesc {
            body_type: K,
            position: [-3.0, 0.5, 0.0],
            linear_velocity: [1.5, 0.0, 0.3],
            angular_velocity: 0.4,
            ..BodyDesc::default()
        });
        coll(&mut w, k, bx(0.6, 0.15, 0.4), [0.2, 0.0, 0.1], false);
        body(&mut w, D, [2.0, 0.5, 0.0], 0.0, bx(0.5, 0.5, 0.5), false);
        w.add_emitter(
            ParticleEmitter::new([0.0, 3.0, 0.0], [0.0, -1.0, 0.0], 40.0)
                .with_spread([4.0, 0.5, 1.0])
                .with_lifetime(2.5)
                .with_radius(0.06),
        );
        rows(&mut m, scene(w), &[0, 1, 2, 30, 60, 90]);
    }

    // Sub-emitters: particles that die and burst into 1, 6 and 13 children (in the x–y plane), some at the frame the
    // parent spawns them.
    {
        let mut w = PhysicsWorld::new(base.clone());
        body(&mut w, S, [0.0, -0.5, 0.0], 0.0, bx(10.0, 0.5, 10.0), false);
        for (k, (count, life)) in [(1_u32, 0.05), (6, 0.1), (13, 0.25), (6, 0.0), (4, 0.016)]
            .into_iter()
            .enumerate()
        {
            let x = 2.0 * f64::from(u32::try_from(k).unwrap_or(0)) - 4.0;
            w.spawn_particle(
                Particle::new([x, 1.0, 0.5 - 0.25 * x], [0.0, 2.0, 0.3], life).with_sub_emitter(SubEmitter {
                    count,
                    speed: 3.0,
                    lifetime: 0.5,
                    radius: 0.04,
                    gravity_scale: 0.5,
                    restitution: 0.6,
                }),
            );
        }
        rows(&mut m, scene(w), &[0, 1, 2, 3, 5, 10, 20, 30, 40]);
    }

    // Particles beside bodies that touch: a ball dropped on a box (contact events) while a field pulls a shell of
    // particles to the box, and a pushed box.
    {
        let mut w = PhysicsWorld::new(base.clone());
        body(&mut w, S, [0.0, -0.5, 0.0], 0.0, bx(5.0, 0.5, 5.0), false);
        body(&mut w, D, [0.0, 2.0, 0.0], 0.0, ball(0.4), false);
        let pushed = body(&mut w, D, [2.5, 0.5, 1.0], 0.3, bx(0.4, 0.4, 0.4), false);
        w.add_force_field(radial([0.0, 0.0, 0.0], 2.0, 1.5, 3.0));
        shell(&mut w, 6, [0.0, 1.0, 0.0], 2.0, 3.0);
        let mut sc = scene(w);
        sc.push = Some((pushed, [-3.0, 0.0, 1.5]));
        rows(&mut m, sc, &[0, 1, 20, 30, 34, 35, 36, 60]);
    }
    // A pendulum on a distance joint and a pair on a spring under a particle shower.
    {
        let mut w = PhysicsWorld::new(base.clone());
        let anchor = w.add_body(BodyDesc {
            body_type: S,
            position: [0.0, 4.0, 0.0],
            ..BodyDesc::default()
        });
        let bob = body(&mut w, D, [1.5, 3.0, 0.5], 0.0, bx(0.3, 0.3, 0.3), false);
        let joint = |a: BodyHandle, b: BodyHandle, jt: JointType| JointDesc {
            body_a: a,
            body_b: b,
            joint_type: jt,
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
            damping: 0.0,
            break_force: None,
        };
        w.add_joint(joint(anchor, bob, JointType::Distance { length: 1.8 }));
        let s1 = body(&mut w, D, [-2.0, 2.0, 0.0], 0.0, ball(0.3), false);
        let s2 = body(&mut w, D, [-1.0, 2.0, 0.4], 0.0, capsule(0.3, 0.2), false);
        w.add_joint(joint(s1, s2, JointType::Spring { rest_length: 0.8, stiffness: 30.0, damping: 1.0 }));
        body(&mut w, S, [0.0, -0.5, 0.0], 0.0, bx(8.0, 0.5, 8.0), false);
        let mut shower = ParticleEmitter::new([0.0, 5.0, 0.0], [0.0, -1.0, 0.0], 60.0)
            .with_spread([3.0, 0.2, 1.0])
            .with_lifetime(3.0)
            .with_radius(0.05);
        shower.particle_restitution = 0.2;
        w.add_emitter(shower);
        rows(&mut m, scene(w), &[0, 1, 2, 30, 60]);
    }

    // Seeded scenes: random 3D colliders (some spun), fields, emitters and particles, in either broadphase, at 1 to 3
    // sub-steps.
    let mut rng = Rng::new(0x3d_a71d);
    for _ in 0..30 {
        let mut cfg = base.clone();
        cfg.sub_steps = u32::try_from(rng.int(1, 3)).unwrap_or(1);
        if rng.int(0, 1) == 1 {
            cfg.broadphase = BroadphaseKind::AabbTree;
        }
        if rng.int(0, 3) == 0 {
            cfg.gravity = [rng.uniform(-3.0, 3.0), rng.uniform(-12.0, 0.0), rng.uniform(-3.0, 3.0)];
        }
        let mut w = PhysicsWorld::new(cfg);
        body(&mut w, S, [0.0, -0.5, 0.0], 0.0, bx(6.0, 0.5, 6.0), false);
        for _ in 0..rng.int(1, 4) {
            let p = [rng.uniform(-4.0, 4.0), rng.uniform(0.5, 4.0), rng.uniform(-3.0, 3.0)];
            let shape = match rng.int(0, 3) {
                0 => ball(rng.uniform(0.2, 0.8)),
                1 => bx(rng.uniform(0.2, 1.0), rng.uniform(0.1, 0.6), rng.uniform(0.2, 1.0)),
                2 => capsule(rng.uniform(0.1, 0.8), rng.uniform(0.1, 0.4)),
                _ => segment(
                    [rng.uniform(-1.0, 1.0), rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5)],
                    [rng.uniform(-1.0, 1.0), rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5)],
                ),
            };
            match rng.int(0, 3) {
                0 => {
                    let v = [rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5)];
                    let at = [rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5)];
                    let off = [rng.uniform(-0.3, 0.3), 0.0, rng.uniform(-0.3, 0.3)];
                    tumbler(&mut w, p, shape, off, v, at);
                }
                1 => {
                    body(&mut w, D, p, rng.uniform(-1.5, 1.5), shape, rng.int(0, 6) == 0);
                }
                _ => {
                    body(&mut w, S, p, rng.uniform(-1.5, 1.5), shape, rng.int(0, 6) == 0);
                }
            }
        }
        for _ in 0..rng.int(0, 2) {
            if rng.int(0, 1) == 0 {
                w.add_force_field(radial(
                    [rng.uniform(-3.0, 3.0), rng.uniform(0.0, 3.0), rng.uniform(-2.0, 2.0)],
                    rng.uniform(-5.0, 5.0),
                    rng.uniform(0.0, 2.5),
                    if rng.int(0, 1) == 0 {
                        0.0
                    } else {
                        rng.uniform(0.5, 4.0)
                    },
                ));
            } else {
                let lo = [rng.uniform(-4.0, 0.0), rng.uniform(-1.0, 2.0), rng.uniform(-3.0, 0.0)];
                w.add_force_field(ForceField::Directional {
                    force: [rng.uniform(-4.0, 4.0), rng.uniform(-4.0, 4.0), rng.uniform(-4.0, 4.0)],
                    min: lo,
                    max: [
                        lo[0] + rng.uniform(0.5, 5.0),
                        lo[1] + rng.uniform(0.5, 4.0),
                        lo[2] + rng.uniform(0.5, 4.0),
                    ],
                });
            }
        }
        if rng.int(0, 1) == 0 {
            w.add_emitter(
                ParticleEmitter::new(
                    [rng.uniform(-3.0, 3.0), rng.uniform(1.0, 4.0), rng.uniform(-2.0, 2.0)],
                    [rng.uniform(-2.0, 2.0), rng.uniform(-1.0, 4.0), rng.uniform(-2.0, 2.0)],
                    rng.uniform(5.0, 90.0),
                )
                .with_spread([rng.uniform(0.0, 2.0), rng.uniform(0.0, 2.0), rng.uniform(0.0, 2.0)])
                .with_lifetime(rng.uniform(0.3, 2.0))
                .with_radius(rng.uniform(0.0, 0.1))
                .with_gravity_scale(rng.uniform(0.0, 1.5))
                .with_damping(rng.uniform(0.0, 1.0)),
            );
        }
        for _ in 0..rng.int(0, 20) {
            let mut p = Particle::new(
                [rng.uniform(-4.0, 4.0), rng.uniform(0.0, 4.0), rng.uniform(-3.0, 3.0)],
                [rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)],
                rng.uniform(0.0, 3.0),
            )
            .with_radius(rng.uniform(0.0, 0.15))
            .with_drag(if rng.int(0, 2) == 0 {
                rng.uniform(0.0, 1.0)
            } else {
                0.0
            })
            .with_restitution(rng.uniform(0.0, 1.0))
            .with_gravity_scale(rng.uniform(-0.5, 1.5));
            if rng.int(0, 4) == 0 {
                p = p.with_sub_emitter(SubEmitter {
                    count: u32::try_from(rng.int(1, 8)).unwrap_or(1),
                    speed: rng.uniform(0.5, 4.0),
                    lifetime: rng.uniform(0.1, 1.0),
                    radius: rng.uniform(0.0, 0.1),
                    gravity_scale: rng.uniform(0.0, 1.0),
                    restitution: rng.uniform(0.0, 1.0),
                });
            }
            w.spawn_particle(p);
        }
        let k = u64::try_from(rng.int(0, 100)).unwrap_or(0);
        rows(&mut m, scene(w), &[0, k, k + 1]);
    }
    m
}
