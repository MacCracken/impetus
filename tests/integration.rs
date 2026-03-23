//! Integration tests exercising cross-module usage.

use impetus::{
    body::{BodyDesc, BodyType},
    collider::{ColliderDesc, ColliderShape},
    config::WorldConfig,
    force::{Force, Impulse},
    joint::{JointDesc, JointType},
    material::PhysicsMaterial,
    units::Quantity,
    PhysicsWorld,
};

#[test]
fn full_world_lifecycle() {
    let mut world = PhysicsWorld::new(WorldConfig::default());

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

    // Add a dynamic ball
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
            material: PhysicsMaterial::rubber(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    assert_eq!(world.body_count(), 2);

    // Step the simulation
    for _ in 0..60 {
        world.step();
    }
    assert_eq!(world.current_step(), 60);

    // Remove ball
    world.remove_body(ball).unwrap();
    assert_eq!(world.body_count(), 1);
}

#[test]
fn joint_connects_bodies() {
    let mut world = PhysicsWorld::new(WorldConfig::default());

    let body_a = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        position: [0.0, 5.0, 0.0],
        ..Default::default()
    });

    let body_b = world.add_body(BodyDesc {
        body_type: BodyType::Dynamic,
        position: [0.0, 3.0, 0.0],
        ..Default::default()
    });

    let _joint = world.add_joint(JointDesc {
        body_a,
        body_b,
        joint_type: JointType::Spring {
            rest_length: 2.0,
            stiffness: 100.0,
            damping: 5.0,
        },
        local_anchor_a: [0.0, 0.0],
        local_anchor_b: [0.0, 0.0],
        motor: None,
        damping: 0.0,
        break_force: None,
    });

    // Stepping should not panic with joints
    for _ in 0..10 {
        world.step();
    }
}

#[test]
fn forces_and_impulses() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let body = world.add_body(BodyDesc::default());

    // Apply gravity-like force
    world.apply_force(body, Force::gravity(10.0, 9.81));

    // Apply an impulse
    world.apply_impulse(body, Impulse::new(5.0, 0.0, 0.0));

    // Step should not panic
    world.step();
    assert_eq!(world.current_step(), 1);
}

#[test]
fn material_presets_are_distinct() {
    let ice = PhysicsMaterial::ice();
    let rubber = PhysicsMaterial::rubber();
    let steel = PhysicsMaterial::steel();

    // Ice has the lowest friction
    assert!(ice.friction < rubber.friction);
    assert!(ice.friction < steel.friction);

    // Rubber has the highest restitution of common materials
    assert!(rubber.restitution > steel.restitution);

    // Steel has the highest density
    assert!(steel.density > rubber.density);
    assert!(steel.density > ice.density);
}

#[test]
fn unit_quantities_display() {
    let force = Quantity::newtons(9.81);
    assert!(force.to_string().contains("N"));

    let angle = Quantity::degrees(90.0);
    let radians = angle.to_radians();
    assert!((radians - std::f64::consts::FRAC_PI_2).abs() < 1e-10);
}

#[test]
fn config_serde_roundtrip() {
    let config = WorldConfig {
        timestep: 1.0 / 120.0,
        gravity: [0.0, -10.0, 0.0],
        velocity_iterations: 8,
        position_iterations: 3,
        deterministic: true,
        step: 0,
        ..Default::default()
    };

    let json = serde_json::to_string(&config).unwrap();
    let back: WorldConfig = serde_json::from_str(&json).unwrap();
    assert_eq!(config.timestep, back.timestep);
    assert_eq!(config.gravity, back.gravity);
    assert_eq!(config.velocity_iterations, back.velocity_iterations);
}

#[test]
fn mixed_body_types() {
    let mut world = PhysicsWorld::new(WorldConfig::default());

    let _static_body = world.add_body(BodyDesc {
        body_type: BodyType::Static,
        ..Default::default()
    });

    let _dynamic = world.add_body(BodyDesc {
        body_type: BodyType::Dynamic,
        ..Default::default()
    });

    let _kinematic = world.add_body(BodyDesc {
        body_type: BodyType::Kinematic,
        ..Default::default()
    });

    assert_eq!(world.body_count(), 3);

    for _ in 0..10 {
        world.step();
    }

    assert_eq!(world.current_step(), 10);
}

#[test]
fn sensor_collider_creation() {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    let body = world.add_body(BodyDesc::default());

    let _sensor = world.add_collider(
        body,
        ColliderDesc {
            shape: ColliderShape::Ball { radius: 5.0 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: true,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    world.step();
}

#[test]
fn deterministic_stepping() {
    let mut world_a = PhysicsWorld::new(WorldConfig::default());
    let mut world_b = PhysicsWorld::new(WorldConfig::default());

    // Same operations on both worlds
    for _ in 0..100 {
        world_a.step();
        world_b.step();
    }

    assert_eq!(world_a.current_step(), world_b.current_step());
    assert_eq!(world_a.current_step(), 100);
}
