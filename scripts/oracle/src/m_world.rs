//! `world_step.tsv` — single-step rows (class S, port plan §5) of `PhysicsWorld::step` in 2D (M3d): the
//! frame counter, the backend's sub-steps and events, then the particles (emitter spawns with the R3
//! spread and the spawn cap, gravity, radial and directional force fields, quadratic drag, damping,
//! collisions with the colliders, sub-emitters, removal of the dead).
//!
//! A row is the world config (`flat::config`, its `step` the frame count so far), the world's own state
//! (`dumpcore::world`: the particle and emitter counters, the particles, emitters and force fields) and
//! the backend dump (`dump2d::backend`), taken after that frame's applied forces; its outputs are the
//! same three after `step()`, then the frame's collision events. Class C: the radial field divides by a
//! `powf`, the sub-emitters spread by `cos` and `sin`, and the backend rotates and normalises; the
//! reference is the corrected oracle (`validate.py` transcribes the world step and writes
//! `world_step.cr.tsv` where glibc misrounds a call).

use impetus::{
    BodyDesc, BodyHandle, BodyType, BroadphaseKind, ColliderDesc, ColliderShape, Force, ForceField,
    Particle, ParticleEmitter, PhysicsMaterial, PhysicsWorld, SubEmitter, WorldConfig,
};

use crate::dump2d;
use crate::dumpcore;
use crate::flat;
use crate::grids::Rng;
use crate::shapes::{ball, bx, capsule, ellipse_hull, segment};
use crate::tsv::{Class, Module, Tok};

const S: BodyType = BodyType::Static;
const D: BodyType = BodyType::Dynamic;
const K: BodyType = BodyType::Kinematic;

fn body(
    w: &mut PhysicsWorld,
    t: BodyType,
    pos: [f64; 2],
    rot: f64,
    shape: ColliderShape,
    sensor: bool,
) -> BodyHandle {
    let h = w.add_body(BodyDesc {
        body_type: t,
        position: [pos[0], pos[1], 0.0],
        rotation: rot,
        ..BodyDesc::default()
    });
    w.add_collider(
        h,
        ColliderDesc {
            shape,
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: sensor,
            mass: None,
            collision_layer: 1,
            collision_mask: u32::MAX,
        },
    );
    h
}

/// A scene: a built world, and a force applied to a body before each frame (if any).
struct Scene {
    world: PhysicsWorld,
    push: Option<(BodyHandle, [f64; 2])>,
}

fn scene(world: PhysicsWorld) -> Scene {
    Scene { world, push: None }
}

/// The world's tokens: the config, its own state, the backend.
fn tokens(w: &PhysicsWorld) -> Vec<Tok> {
    let d = w.oracle_dump();
    let mut t = flat::config(&d.config);
    t.extend(dumpcore::world(&d));
    t.extend(dump2d::backend(&d.backend));
    t
}

/// Runs a scene to the last frame asked, emitting a row for each frame k in `frames`.
fn rows(m: &mut Module, mut sc: Scene, frames: &[u64]) {
    let last = frames.iter().copied().max().unwrap_or(0);
    for k in 0..=last {
        if let Some((h, f)) = sc.push {
            sc.world.apply_force(h, Force::new(f[0], f[1], 0.0));
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
            m.row("world::step", Class::C, &ins, &outs);
        }
    }
}

const FRAMES: [u64; 7] = [0, 1, 2, 5, 10, 30, 60];

fn radial(c: [f64; 2], strength: f64, falloff: f64, radius: f64) -> ForceField {
    ForceField::Radial {
        center: [c[0], c[1], 0.0],
        strength,
        falloff,
        radius,
    }
}

/// A ring of `n` particles at radius `r` about (cx, cy), at rest, lifetime `life`.
fn ring(w: &mut PhysicsWorld, n: u32, c: [f64; 2], r: f64, life: f64) {
    for k in 0..n {
        // Rational points on the circle (no transcendental).
        let t = -2.0 + 4.0 * f64::from(k) / f64::from(n);
        let d = 1.0 + t * t;
        let p = [c[0] + r * (1.0 - t * t) / d, c[1] + r * (2.0 * t) / d, 0.0];
        w.spawn_particle(Particle::new(p, [0.0, 0.0, 0.0], life));
    }
}

#[allow(clippy::too_many_lines)] // the scenes
pub fn emit() -> Module {
    let mut m = Module::new("world_step");
    let base = WorldConfig::default();
    let zero_g = WorldConfig {
        gravity: [0.0, 0.0, 0.0],
        ..base.clone()
    };

    // An emitter over a floor: spawns at 60/s with spread, collisions, expiry; a second emitter that is
    // inactive, and one with no valid rate.
    {
        let mut w = PhysicsWorld::new(base.clone());
        body(&mut w, S, [0.0, -0.5], 0.0, bx(10.0, 0.5), false);
        w.add_emitter(
            ParticleEmitter::new([0.0, 2.0, 0.0], [0.5, 3.0, 0.0], 60.0)
                .with_spread([2.0, 1.0, 0.5])
                .with_lifetime(1.5)
                .with_radius(0.08)
                .with_damping(0.2),
        );
        let mut off = ParticleEmitter::new([1.0, 1.0, 0.0], [0.0, 1.0, 0.0], 100.0);
        off.active = false;
        w.add_emitter(off);
        w.add_emitter(ParticleEmitter::new(
            [0.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            f64::NAN,
        ));
        w.add_emitter(ParticleEmitter::new([0.0, 1.0, 0.0], [0.0, 1.0, 0.0], -3.0));
        rows(&mut m, scene(w), &[0, 1, 2, 10, 60, 100, 120]);
    }
    // A fast emitter (several spawns a frame) with gravity scale 0 and a sub-step config.
    {
        let mut cfg = base.clone();
        cfg.sub_steps = 2;
        let mut w = PhysicsWorld::new(cfg);
        w.add_emitter(
            ParticleEmitter::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 330.0)
                .with_spread([0.5, 0.5, 0.0])
                .with_lifetime(0.2)
                .with_gravity_scale(0.0),
        );
        rows(&mut m, scene(w), &[0, 1, 2, 5, 11, 12, 13]);
    }

    // Radial fields: attraction at falloffs 0, 1, 1.5 and 2, repulsion, a reach limit.
    for (falloff, strength, radius) in [
        (0.0, 4.0, 0.0),
        (1.0, 4.0, 0.0),
        (1.5, 4.0, 0.0),
        (2.0, 4.0, 0.0),
        (2.0, -6.0, 0.0),
        (1.0, 4.0, 1.2),
        (0.7, 3.0, 0.0),
    ] {
        let mut w = PhysicsWorld::new(zero_g.clone());
        w.add_force_field(radial([0.0, 0.0], strength, falloff, radius));
        ring(&mut w, 12, [0.0, 0.0], 1.5, 10.0);
        ring(&mut w, 8, [0.3, -0.2], 0.4, 10.0);
        // One exactly at the centre: the field skips it.
        w.spawn_particle(Particle::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 10.0));
        rows(&mut m, scene(w), &FRAMES);
    }
    // Directional fields: a wind box (inside and outside), stacked with gravity and a radial field.
    {
        let mut w = PhysicsWorld::new(base.clone());
        w.add_force_field(ForceField::Directional {
            force: [3.0, 1.0, 0.0],
            min: [-1.0, -1.0, -1.0],
            max: [1.0, 1.0, 1.0],
        });
        w.add_force_field(radial([2.0, 0.0], 2.0, 2.0, 0.0));
        ring(&mut w, 10, [0.0, 0.0], 0.9, 10.0);
        ring(&mut w, 6, [0.0, 0.0], 1.4, 10.0);
        w.spawn_particle(Particle::new([1.0, 1.0, 1.0], [0.0, 0.0, 0.0], 10.0));
        rows(&mut m, scene(w), &FRAMES);
    }

    // Drag and damping: fast particles with quadratic drag (including one whose drag would reverse it),
    // damped ones, a slow one under the drag threshold.
    {
        let mut w = PhysicsWorld::new(zero_g.clone());
        for (k, (v, drag, damping)) in [
            ([10.0, 0.0, 0.0], 0.5, 0.0),
            ([3.0, -4.0, 1.0], 2.0, 0.0),
            ([200.0, 50.0, 0.0], 5.0, 0.0),
            ([1e-11, 0.0, 0.0], 3.0, 0.0),
            ([2.0, 2.0, 0.0], 0.0, 1.5),
            ([5.0, 0.0, 0.0], 0.3, 0.7),
        ]
        .into_iter()
        .enumerate()
        {
            let y = f64::from(u32::try_from(k).unwrap_or(0));
            w.spawn_particle(
                Particle::new([0.0, y, 0.0], v, 5.0)
                    .with_drag(drag)
                    .with_damping(damping),
            );
        }
        rows(&mut m, scene(w), &FRAMES);
    }

    // Collisions: particles falling onto a box, a rotated box, a ball, a rotated capsule, a segment, a
    // hull (no contact for hulls), a sensor (ignored); a particle starting inside a box; one with radius 0
    // (skipped); restitutions 0, 0.3 and 1.
    {
        let mut w = PhysicsWorld::new(base.clone());
        body(&mut w, S, [-6.0, 0.0], 0.0, bx(1.0, 0.5), false);
        body(&mut w, S, [-3.0, 0.0], 0.4, bx(1.0, 0.3), false);
        body(&mut w, S, [0.0, 0.0], 0.0, ball(0.7), false);
        body(&mut w, S, [3.0, 0.0], 1.2, capsule(0.8, 0.3), false);
        body(
            &mut w,
            S,
            [6.0, 0.0],
            0.1,
            segment([-1.0, 0.0], [1.0, 0.2]),
            false,
        );
        body(
            &mut w,
            S,
            [9.0, 0.0],
            0.0,
            ellipse_hull(6, 1.0, 0.6, true),
            false,
        );
        body(&mut w, S, [12.0, 0.0], 0.0, bx(1.0, 0.5), true);
        for (k, rest) in [0.0, 0.3, 1.0].into_iter().enumerate() {
            let dx = 0.25 * f64::from(u32::try_from(k).unwrap_or(0)) - 0.25;
            for cx in [-6.0, -3.0, 0.0, 3.0, 6.0, 9.0, 12.0] {
                w.spawn_particle(
                    Particle::new([cx + dx, 1.5, 0.0], [0.0, -1.0, 0.0], 10.0)
                        .with_radius(0.1)
                        .with_restitution(rest),
                );
            }
        }
        w.spawn_particle(Particle::new([-6.2, 0.1, 0.0], [0.0, 0.0, 0.0], 10.0).with_radius(0.05));
        w.spawn_particle(Particle::new([0.0, 0.69, 0.0], [0.0, -2.0, 0.0], 10.0).with_radius(0.0));
        rows(&mut m, scene(w), &[0, 1, 2, 5, 10, 20, 30, 40, 60]);
    }
    // Particles on a moving kinematic paddle and around a dynamic box resting on the ground.
    {
        let mut w = PhysicsWorld::new(base.clone());
        body(&mut w, S, [0.0, -0.5], 0.0, bx(10.0, 0.5), false);
        let k = w.add_body(BodyDesc {
            body_type: K,
            position: [-3.0, 0.5, 0.0],
            linear_velocity: [1.5, 0.0, 0.0],
            angular_velocity: 0.4,
            ..BodyDesc::default()
        });
        w.add_collider(
            k,
            ColliderDesc {
                shape: bx(0.6, 0.15),
                offset: [0.2, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 1,
                collision_mask: u32::MAX,
            },
        );
        body(&mut w, D, [2.0, 0.5], 0.0, bx(0.5, 0.5), false);
        w.add_emitter(
            ParticleEmitter::new([0.0, 3.0, 0.0], [0.0, -1.0, 0.0], 40.0)
                .with_spread([4.0, 0.5, 0.0])
                .with_lifetime(2.5)
                .with_radius(0.06),
        );
        rows(&mut m, scene(w), &[0, 1, 2, 30, 60, 90]);
    }

    // Sub-emitters: particles that die and burst into 1, 6 and 13 children (whose own sub-emitters are
    // none), some at the frame the parent spawns them.
    {
        let mut w = PhysicsWorld::new(base.clone());
        body(&mut w, S, [0.0, -0.5], 0.0, bx(10.0, 0.5), false);
        for (k, (count, life)) in [(1_u32, 0.05), (6, 0.1), (13, 0.25), (6, 0.0), (4, 0.016)]
            .into_iter()
            .enumerate()
        {
            let x = 2.0 * f64::from(u32::try_from(k).unwrap_or(0)) - 4.0;
            w.spawn_particle(
                Particle::new([x, 1.0, 0.0], [0.0, 2.0, 0.0], life).with_sub_emitter(SubEmitter {
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

    // Particles beside bodies that touch: a ball dropped on a box (contact events) while a field pulls
    // particles to the box.
    {
        let mut w = PhysicsWorld::new(base.clone());
        body(&mut w, S, [0.0, -0.5], 0.0, bx(5.0, 0.5), false);
        body(&mut w, D, [0.0, 2.0], 0.0, ball(0.4), false);
        w.add_force_field(radial([0.0, 0.0], 2.0, 1.5, 3.0));
        ring(&mut w, 16, [0.0, 1.0], 2.0, 3.0);
        rows(&mut m, scene(w), &[0, 1, 20, 30, 34, 35, 36, 60]);
    }

    // Seeded scenes: random colliders, fields, emitters and particles, in either broadphase.
    let mut rng = Rng::new(0x3a71d);
    for _ in 0..30 {
        let mut cfg = base.clone();
        cfg.sub_steps = u32::try_from(rng.int(1, 2)).unwrap_or(1);
        if rng.int(0, 1) == 1 {
            cfg.broadphase = BroadphaseKind::AabbTree;
        }
        if rng.int(0, 3) == 0 {
            cfg.gravity = [rng.uniform(-3.0, 3.0), rng.uniform(-12.0, 0.0), 0.0];
        }
        let mut w = PhysicsWorld::new(cfg);
        body(&mut w, S, [0.0, -0.5], 0.0, bx(6.0, 0.5), false);
        for _ in 0..rng.int(1, 4) {
            let p = [rng.uniform(-4.0, 4.0), rng.uniform(0.5, 4.0)];
            let shape = match rng.int(0, 3) {
                0 => ball(rng.uniform(0.2, 0.8)),
                1 => bx(rng.uniform(0.2, 1.0), rng.uniform(0.1, 0.6)),
                2 => capsule(rng.uniform(0.1, 0.8), rng.uniform(0.1, 0.4)),
                _ => segment(
                    [rng.uniform(-1.0, 1.0), rng.uniform(-0.5, 0.5)],
                    [rng.uniform(-1.0, 1.0), rng.uniform(-0.5, 0.5)],
                ),
            };
            let t = if rng.int(0, 2) == 0 { D } else { S };
            body(
                &mut w,
                t,
                p,
                rng.uniform(-1.5, 1.5),
                shape,
                rng.int(0, 6) == 0,
            );
        }
        for _ in 0..rng.int(0, 2) {
            if rng.int(0, 1) == 0 {
                w.add_force_field(radial(
                    [rng.uniform(-3.0, 3.0), rng.uniform(0.0, 3.0)],
                    rng.uniform(-5.0, 5.0),
                    rng.uniform(0.0, 2.5),
                    if rng.int(0, 1) == 0 {
                        0.0
                    } else {
                        rng.uniform(0.5, 4.0)
                    },
                ));
            } else {
                let lo = [rng.uniform(-4.0, 0.0), rng.uniform(-1.0, 2.0), -1.0];
                w.add_force_field(ForceField::Directional {
                    force: [rng.uniform(-4.0, 4.0), rng.uniform(-4.0, 4.0), 0.0],
                    min: lo,
                    max: [
                        lo[0] + rng.uniform(0.5, 5.0),
                        lo[1] + rng.uniform(0.5, 4.0),
                        1.0,
                    ],
                });
            }
        }
        if rng.int(0, 1) == 0 {
            w.add_emitter(
                ParticleEmitter::new(
                    [rng.uniform(-3.0, 3.0), rng.uniform(1.0, 4.0), 0.0],
                    [rng.uniform(-2.0, 2.0), rng.uniform(-1.0, 4.0), 0.0],
                    rng.uniform(5.0, 90.0),
                )
                .with_spread([rng.uniform(0.0, 2.0), rng.uniform(0.0, 2.0), 0.0])
                .with_lifetime(rng.uniform(0.3, 2.0))
                .with_radius(rng.uniform(0.0, 0.1))
                .with_gravity_scale(rng.uniform(0.0, 1.5))
                .with_damping(rng.uniform(0.0, 1.0)),
            );
        }
        for _ in 0..rng.int(0, 20) {
            let mut p = Particle::new(
                [rng.uniform(-4.0, 4.0), rng.uniform(0.0, 4.0), 0.0],
                [rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0), 0.0],
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
