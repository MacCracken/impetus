#![no_main]

use libfuzzer_sys::fuzz_target;

use impetus::serialize::{deserialize_world, serialize_world};
use impetus::{PhysicsWorld, WorldConfig};

// Feed arbitrary bytes to the snapshot decoder and the config parser.
//
// Neither may panic. A snapshot that `try_restore` accepts must produce a
// world that steps and re-serializes without panicking; a JSON config that
// passes `WorldConfig::validate` must step.
fuzz_target!(|data: &[u8]| {
    let mut world = PhysicsWorld::new(WorldConfig::default());
    if deserialize_world(&mut world, data).is_ok() {
        for _ in 0..3 {
            world.step();
        }
        let _ = serialize_world(&world);
    }

    if let Ok(text) = std::str::from_utf8(data)
        && let Ok(config) = serde_json::from_str::<WorldConfig>(text)
        && let Ok(mut world) = PhysicsWorld::try_new(config)
    {
        world.step();
    }
});
