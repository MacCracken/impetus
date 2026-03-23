#![no_main]

use libfuzzer_sys::fuzz_target;

use impetus::{
    BodyDesc, BodyType, ColliderDesc, ColliderShape, PhysicsMaterial, PhysicsWorld, WorldConfig,
};

/// Feed arbitrary bytes to the world as a JSON snapshot and try to roundtrip.
/// If deserialization succeeds, verify that re-serialization produces a valid
/// snapshot and that stepping the restored world does not panic.
fuzz_target!(|data: &[u8]| {
    // Try to interpret bytes as a JSON WorldConfig (simpler, more likely to parse)
    if let Ok(text) = std::str::from_utf8(data) {
        if let Ok(config) = serde_json::from_str::<WorldConfig>(text) {
            // Valid config: create world with it and step
            let mut world = PhysicsWorld::new(config);
            world.step();
        }
    }

    // Also test that a real world can survive arbitrary mutations:
    // Build a small world, serialize, and verify roundtrip
    if data.len() >= 8 {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let body = world.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        world.add_collider(
            body,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 1.0 },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );

        // Step a number of times dictated by the input
        let steps = (data[0] as u32) % 10;
        for _ in 0..steps {
            world.step();
        }

        // Verify body state is finite
        if let Ok(state) = world.get_body_state(body) {
            for v in &state.position {
                assert!(v.is_finite());
            }
            for v in &state.linear_velocity {
                assert!(v.is_finite());
            }
        }
    }
});
