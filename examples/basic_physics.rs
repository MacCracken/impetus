//! Basic usage of the impetus physics engine.

use impetus::{
    body::{BodyDesc, BodyType},
    collider::{ColliderDesc, ColliderShape},
    config::WorldConfig,
    force::Force,
    material::PhysicsMaterial,
    units::Quantity,
    PhysicsWorld,
};

fn main() {
    // Create a world with default gravity
    let config = WorldConfig::default();
    println!("Timestep: {} s", config.timestep);
    println!("Gravity: {:?}", config.gravity);

    let mut world = PhysicsWorld::new(config);

    // Add a static floor
    let floor = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, -1.0, 0.0],
        ..Default::default()
    });
    world.add_collider(
        floor,
        ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [50.0, 1.0, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::wood(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Add a bouncy ball
    let ball = world.add_body(BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 10.0, 0.0],
        ..Default::default()
    });
    world.add_collider(
        ball,
        ColliderDesc {
            shape: ColliderShape::Ball { radius: 0.5 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::bouncy(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    println!("Bodies: {}", world.body_count());

    // Apply a lateral force
    world.apply_force(ball, Force::new(5.0, 0.0, 0.0));

    // Step for 1 second (60 steps at 1/60 timestep)
    for _ in 0..60 {
        world.step();
    }

    println!("After 1s: step={}", world.current_step());
    println!(
        "Collision events this frame: {}",
        world.collision_events().len()
    );

    // Unit-aware quantities
    let force = Quantity::newtons(9.81);
    let distance = Quantity::meters(10.0);
    let angle = Quantity::degrees(45.0);
    println!("Force: {force}");
    println!("Distance: {distance}");
    println!("Angle: {angle} = {:.4} rad", angle.to_radians());
}
