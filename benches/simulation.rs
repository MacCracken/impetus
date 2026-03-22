use criterion::{criterion_group, criterion_main, Criterion};
use impetus::{
    body::{BodyDesc, BodyType},
    collider::{ColliderDesc, ColliderShape},
    force::Force,
    material::PhysicsMaterial,
    PhysicsWorld, WorldConfig,
};

fn bench_step_empty(c: &mut Criterion) {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    c.bench_function("step_empty", |b| b.iter(|| world.step()));
}

fn bench_step_100_bodies(c: &mut Criterion) {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    for i in 0..100 {
        let body = world.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [(i % 10) as f64, (i / 10) as f64 * 2.0],
            ..Default::default()
        });
        world.add_collider(
            body,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 0.5 },
                offset: [0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
            },
        );
    }
    c.bench_function("step_100_bodies", |b| b.iter(|| world.step()));
}

fn bench_add_remove_body(c: &mut Criterion) {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    c.bench_function("add_remove_body", |b| {
        b.iter(|| {
            let body = world.add_body(BodyDesc::default());
            world.remove_body(body).unwrap();
        })
    });
}

criterion_group!(
    benches,
    bench_step_empty,
    bench_step_100_bodies,
    bench_add_remove_body
);
criterion_main!(benches);
