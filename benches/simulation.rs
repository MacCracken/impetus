use criterion::{Criterion, black_box, criterion_group, criterion_main};
use impetus::{
    body::{BodyDesc, BodyType},
    collider::{ColliderDesc, ColliderShape},
    config::WorldConfig,
    force::{Force, Impulse},
    joint::{JointDesc, JointType},
    material::PhysicsMaterial,
    particle::{Particle, ParticleEmitter},
    units::{PhysicsUnit, Quantity},
    PhysicsWorld,
};

// ---------------------------------------------------------------------------
// World stepping
// ---------------------------------------------------------------------------

fn bench_world(c: &mut Criterion) {
    let mut group = c.benchmark_group("world");

    group.bench_function("step_empty", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        b.iter(|| world.step())
    });

    group.bench_function("step_10_bodies", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        for i in 0..10 {
            let body = world.add_body(BodyDesc {
                body_type: BodyType::Dynamic,
                position: [(i % 5) as f64, (i / 5) as f64 * 2.0, 0.0],
                ..Default::default()
            });
            world.add_collider(
                body,
                ColliderDesc {
                    shape: ColliderShape::Ball { radius: 0.5 },
                    offset: [0.0, 0.0, 0.0],
                    material: PhysicsMaterial::default(),
                    is_sensor: false,
                    mass: None,
                },
            );
        }
        b.iter(|| world.step())
    });

    group.bench_function("step_100_bodies", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        for i in 0..100 {
            let body = world.add_body(BodyDesc {
                body_type: BodyType::Dynamic,
                position: [(i % 10) as f64, (i / 10) as f64 * 2.0, 0.0],
                ..Default::default()
            });
            world.add_collider(
                body,
                ColliderDesc {
                    shape: ColliderShape::Ball { radius: 0.5 },
                    offset: [0.0, 0.0, 0.0],
                    material: PhysicsMaterial::default(),
                    is_sensor: false,
                    mass: None,
                },
            );
        }
        b.iter(|| world.step())
    });

    group.bench_function("step_1000_bodies", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        for i in 0..1000 {
            let body = world.add_body(BodyDesc {
                body_type: BodyType::Dynamic,
                position: [(i % 32) as f64, (i / 32) as f64 * 2.0, 0.0],
                ..Default::default()
            });
            world.add_collider(
                body,
                ColliderDesc {
                    shape: ColliderShape::Ball { radius: 0.5 },
                    offset: [0.0, 0.0, 0.0],
                    material: PhysicsMaterial::default(),
                    is_sensor: false,
                    mass: None,
                },
            );
        }
        b.iter(|| world.step())
    });

    group.finish();
}

// ---------------------------------------------------------------------------
// Body management
// ---------------------------------------------------------------------------

fn bench_bodies(c: &mut Criterion) {
    let mut group = c.benchmark_group("bodies");

    group.bench_function("add_body", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        b.iter(|| {
            world.add_body(black_box(BodyDesc::default()));
        })
    });

    group.bench_function("add_remove_body", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        b.iter(|| {
            let body = world.add_body(black_box(BodyDesc::default()));
            world.remove_body(body).unwrap();
        })
    });

    group.bench_function("add_body_with_collider", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        b.iter(|| {
            let body = world.add_body(black_box(BodyDesc {
                body_type: BodyType::Dynamic,
                position: [1.0, 2.0, 0.0],
                ..Default::default()
            }));
            world.add_collider(
                body,
                black_box(ColliderDesc {
                    shape: ColliderShape::Ball { radius: 0.5 },
                    offset: [0.0, 0.0, 0.0],
                    material: PhysicsMaterial::default(),
                    is_sensor: false,
                    mass: None,
                }),
            );
        })
    });

    group.bench_function("add_joint", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let a = world.add_body(BodyDesc::default());
        let body_b = world.add_body(BodyDesc::default());
        b.iter(|| {
            world.add_joint(black_box(JointDesc {
                body_a: a,
                body_b,
                joint_type: JointType::Spring {
                    rest_length: 2.0,
                    stiffness: 100.0,
                    damping: 5.0,
                },
                local_anchor_a: [0.0, 0.0],
                local_anchor_b: [0.0, 0.0],
            }));
        })
    });

    group.finish();
}

// ---------------------------------------------------------------------------
// Forces and impulses
// ---------------------------------------------------------------------------

fn bench_forces(c: &mut Criterion) {
    let mut group = c.benchmark_group("forces");

    group.bench_function("apply_force", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let body = world.add_body(BodyDesc::default());
        b.iter(|| {
            world.apply_force(black_box(body), black_box(Force::new(10.0, -5.0, 0.0)));
        })
    });

    group.bench_function("apply_impulse", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let body = world.add_body(BodyDesc::default());
        b.iter(|| {
            world.apply_impulse(black_box(body), black_box(Impulse::new(0.0, 20.0, 0.0)));
        })
    });

    group.bench_function("force_magnitude", |b| {
        let f = Force::new(3.0, 4.0, 0.0);
        b.iter(|| black_box(&f).magnitude())
    });

    group.bench_function("gravity_force", |b| {
        b.iter(|| Force::gravity(black_box(10.0), black_box(9.81)))
    });

    group.finish();
}

// ---------------------------------------------------------------------------
// Materials and units
// ---------------------------------------------------------------------------

fn bench_materials(c: &mut Criterion) {
    let mut group = c.benchmark_group("materials");

    group.bench_function("preset_steel", |b| {
        b.iter(PhysicsMaterial::steel)
    });

    group.bench_function("preset_rubber", |b| {
        b.iter(PhysicsMaterial::rubber)
    });

    group.bench_function("quantity_display", |b| {
        let q = Quantity::newtons(9.81);
        b.iter(|| format!("{}", black_box(&q)))
    });

    group.bench_function("degrees_to_radians", |b| {
        let q = Quantity::degrees(180.0);
        b.iter(|| black_box(&q).to_radians())
    });

    group.bench_function("quantity_new", |b| {
        b.iter(|| Quantity::new(black_box(42.0), black_box(PhysicsUnit::Newtons)))
    });

    group.finish();
}

// ---------------------------------------------------------------------------
// Serialization (serde_json)
// ---------------------------------------------------------------------------

fn bench_serde(c: &mut Criterion) {
    let mut group = c.benchmark_group("serde");

    group.bench_function("serialize_body_desc", |b| {
        let desc = BodyDesc {
            body_type: BodyType::Dynamic,
            position: [5.0, 10.0, 0.0],
            rotation: 1.57,
            ..Default::default()
        };
        b.iter(|| serde_json::to_string(black_box(&desc)).unwrap())
    });

    group.bench_function("deserialize_body_desc", |b| {
        let json = serde_json::to_string(&BodyDesc::default()).unwrap();
        b.iter(|| serde_json::from_str::<BodyDesc>(black_box(&json)).unwrap())
    });

    group.bench_function("serialize_config", |b| {
        let config = WorldConfig::default();
        b.iter(|| serde_json::to_string(black_box(&config)).unwrap())
    });

    group.bench_function("roundtrip_collider_desc", |b| {
        let desc = ColliderDesc {
            shape: ColliderShape::Ball { radius: 1.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::rubber(),
            is_sensor: false,
            mass: Some(5.0),
        };
        b.iter(|| {
            let json = serde_json::to_string(black_box(&desc)).unwrap();
            serde_json::from_str::<ColliderDesc>(&json).unwrap()
        })
    });

    group.finish();
}

// ---------------------------------------------------------------------------
// Particles
// ---------------------------------------------------------------------------

fn bench_particles(c: &mut Criterion) {
    let mut group = c.benchmark_group("particles");

    group.bench_function("step_100_particles", |b| {
        b.iter_custom(|iters| {
            let mut total = std::time::Duration::ZERO;
            for _ in 0..iters {
                let mut world = PhysicsWorld::new(WorldConfig::default());
                for i in 0..100 {
                    world.spawn_particle(
                        Particle::new(
                            [(i % 10) as f64, (i / 10) as f64 * 2.0 + 5.0, 0.0],
                            [0.0, 0.0, 0.0],
                            100.0,
                        )
                        .with_radius(0.05),
                    );
                }
                let start = std::time::Instant::now();
                world.step();
                total += start.elapsed();
            }
            total
        })
    });

    group.bench_function("step_1000_particles", |b| {
        b.iter_custom(|iters| {
            let mut total = std::time::Duration::ZERO;
            for _ in 0..iters {
                let mut world = PhysicsWorld::new(WorldConfig::default());
                for i in 0..1000 {
                    world.spawn_particle(
                        Particle::new(
                            [(i % 32) as f64, (i / 32) as f64 * 2.0 + 5.0, 0.0],
                            [0.0, 0.0, 0.0],
                            100.0,
                        )
                        .with_radius(0.05),
                    );
                }
                let start = std::time::Instant::now();
                world.step();
                total += start.elapsed();
            }
            total
        })
    });

    group.bench_function("step_100_particles_with_floor", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let floor = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        world.add_collider(
            floor,
            ColliderDesc {
                shape: ColliderShape::Box {
                    half_extents: [50.0, 0.5, 0.0],
                },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
            },
        );
        for i in 0..100 {
            world.spawn_particle(
                Particle::new(
                    [(i % 10) as f64, (i / 10) as f64 + 1.0, 0.0],
                    [0.0, -2.0, 0.0],
                    10.0,
                )
                .with_radius(0.05)
                .with_restitution(0.5),
            );
        }
        b.iter(|| world.step())
    });

    group.bench_function("spawn_particle", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        b.iter(|| {
            world.spawn_particle(black_box(Particle::new([0.0, 0.0, 0.0], [1.0, 2.0, 0.0], 3.0)));
        })
    });

    group.bench_function("emitter_step", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        world.add_emitter(
            ParticleEmitter::new([0.0, 0.0, 0.0], [0.0, 10.0, 0.0], 100.0).with_lifetime(0.5),
        );
        b.iter(|| world.step())
    });

    group.finish();
}

// ---------------------------------------------------------------------------
// Serialization
// ---------------------------------------------------------------------------

#[cfg(feature = "serialize")]
fn bench_serialize(c: &mut Criterion) {
    let mut group = c.benchmark_group("snapshot");

    group.bench_function("serialize_empty", |b| {
        let world = PhysicsWorld::new(WorldConfig::default());
        b.iter(|| impetus::serialize::serialize_world(black_box(&world)).unwrap())
    });

    group.bench_function("serialize_100_bodies", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        for i in 0..100 {
            let body = world.add_body(BodyDesc {
                body_type: BodyType::Dynamic,
                position: [(i % 10) as f64, (i / 10) as f64, 0.0],
                ..Default::default()
            });
            world.add_collider(
                body,
                ColliderDesc {
                    shape: ColliderShape::Ball { radius: 0.5 },
                    offset: [0.0, 0.0, 0.0],
                    material: PhysicsMaterial::default(),
                    is_sensor: false,
                    mass: None,
                },
            );
        }
        b.iter(|| impetus::serialize::serialize_world(black_box(&world)).unwrap())
    });

    group.bench_function("roundtrip_100_bodies", |b| {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        for i in 0..100 {
            let body = world.add_body(BodyDesc {
                body_type: BodyType::Dynamic,
                position: [(i % 10) as f64, (i / 10) as f64, 0.0],
                ..Default::default()
            });
            world.add_collider(
                body,
                ColliderDesc {
                    shape: ColliderShape::Ball { radius: 0.5 },
                    offset: [0.0, 0.0, 0.0],
                    material: PhysicsMaterial::default(),
                    is_sensor: false,
                    mass: None,
                },
            );
        }
        let data = impetus::serialize::serialize_world(&world).unwrap();
        b.iter(|| {
            let mut w = PhysicsWorld::new(WorldConfig::default());
            impetus::serialize::deserialize_world(&mut w, black_box(&data)).unwrap();
        })
    });

    group.finish();
}

#[cfg(not(feature = "serialize"))]
fn bench_serialize(_c: &mut Criterion) {}

criterion_group!(
    benches,
    bench_world,
    bench_bodies,
    bench_forces,
    bench_materials,
    bench_serde,
    bench_particles,
    bench_serialize,
);
criterion_main!(benches);
