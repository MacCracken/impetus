#![no_main]

use libfuzzer_sys::fuzz_target;

use impetus::{
    BodyDesc, BodyType, ColliderDesc, ColliderShape, PhysicsMaterial, PhysicsWorld, WorldConfig,
};

/// Interpret arbitrary bytes as two circles and step the simulation.
/// Verifies no panics and no NaN in the resulting body states.
fuzz_target!(|data: &[u8]| {
    if data.len() < 40 {
        return;
    }

    let f = |offset: usize| -> f64 {
        let bytes: [u8; 8] = data[offset..offset + 8].try_into().unwrap();
        let v = f64::from_le_bytes(bytes);
        // Clamp to reasonable range to avoid degenerate floats
        if v.is_finite() { v.clamp(-1000.0, 1000.0) } else { 0.0 }
    };

    let x1 = f(0);
    let y1 = f(8);
    let r1 = f(16).abs().max(0.01).min(100.0);
    let x2 = f(24);
    let y2 = f(32);

    let r2 = if data.len() >= 48 {
        f(40).abs().max(0.01).min(100.0)
    } else {
        1.0
    };

    let mut world = PhysicsWorld::new(WorldConfig {
        gravity: [0.0, 0.0, 0.0],
        ..WorldConfig::default()
    });

    let a = world.add_body(BodyDesc {
        body_type: BodyType::Dynamic,
        position: [x1, y1, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(
        a,
        ColliderDesc {
            shape: ColliderShape::Ball { radius: r1 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    let b = world.add_body(BodyDesc {
        body_type: BodyType::Dynamic,
        position: [x2, y2, 0.0],
        ..BodyDesc::default()
    });
    world.add_collider(
        b,
        ColliderDesc {
            shape: ColliderShape::Ball { radius: r2 },
            offset: [0.0, 0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: false,
            mass: None,
            collision_layer: 0xFFFF_FFFF,
            collision_mask: 0xFFFF_FFFF,
        },
    );

    // Step the simulation a few times
    for _ in 0..3 {
        world.step();
    }

    // Verify no NaN in body states
    if let Ok(state_a) = world.get_body_state(a) {
        for v in &state_a.position {
            assert!(v.is_finite(), "position contains NaN/Inf");
        }
        for v in &state_a.linear_velocity {
            assert!(v.is_finite(), "velocity contains NaN/Inf");
        }
    }
    if let Ok(state_b) = world.get_body_state(b) {
        for v in &state_b.position {
            assert!(v.is_finite(), "position contains NaN/Inf");
        }
        for v in &state_b.linear_velocity {
            assert!(v.is_finite(), "velocity contains NaN/Inf");
        }
    }
});
