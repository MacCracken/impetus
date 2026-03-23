//! Simple 2D platformer physics setup.
//!
//! Demonstrates:
//! - Static floor and walls
//! - Dynamic player body with capsule collider
//! - Force-based movement
//! - Reading body state each frame

use impetus::{
    body::{BodyDesc, BodyType},
    collider::{ColliderDesc, ColliderShape},
    config::WorldConfig,
    force::Force,
    material::PhysicsMaterial,
    PhysicsWorld,
};

fn main() {
    // --- World ---
    let config = WorldConfig {
        gravity: [0.0, -20.0, 0.0], // stronger gravity for snappy platformer feel
        ..Default::default()
    };
    let mut world = PhysicsWorld::new(config);

    // --- Static geometry ---

    // Floor: 40 units wide, 1 unit tall, centered at y = -0.5
    let floor = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, -0.5, 0.0],
        ..Default::default()
    });
    world.add_collider(
        floor,
        ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [20.0, 0.5, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Left wall
    let left_wall = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        position: [-20.5, 10.0, 0.0],
        ..Default::default()
    });
    world.add_collider(
        left_wall,
        ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [0.5, 10.0, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Right wall
    let right_wall = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        position: [20.5, 10.0, 0.0],
        ..Default::default()
    });
    world.add_collider(
        right_wall,
        ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [0.5, 10.0, 0.0],
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // --- Player ---

    let player = world.add_body(BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 2.0, 0.0],
        fixed_rotation: true, // prevent tumbling
        linear_damping: 0.5,  // slight air resistance
        ..Default::default()
    });
    world.add_collider(
        player,
        ColliderDesc {
            shape: ColliderShape::Capsule {
                half_height: 0.4,
                radius: 0.3,
            },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial {
                friction: 0.8,
                restitution: 0.0,
                density: 1.5,
                ..PhysicsMaterial::default()
            },
            is_sensor: false,
            mass: Some(70.0), // 70 kg player
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    println!(
        "Platformer: {} bodies, timestep = {} s",
        world.body_count(),
        world.timestep()
    );

    // --- Simulate 3 seconds ---

    let move_force = 500.0; // Newtons of horizontal push

    for frame in 0..180 {
        // Move right for the first second, then stop pushing
        if frame < 60 {
            world.apply_force(player, Force::new(move_force, 0.0, 0.0));
        }

        world.step();

        // Print position every 30 frames (~0.5 s)
        if frame % 30 == 0
            && let Ok(state) = world.get_body_state(player)
        {
            println!(
                "t = {:.2}s  pos = ({:.2}, {:.2})  vel = ({:.2}, {:.2})",
                frame as f64 / 60.0,
                state.position[0],
                state.position[1],
                state.linear_velocity[0],
                state.linear_velocity[1],
            );
        }

        // Report collision events
        for event in world.collision_events() {
            println!("  frame {frame}: {event:?}");
        }
    }

    println!("Done — {} steps simulated.", world.current_step());
}
