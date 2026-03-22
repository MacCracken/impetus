//! World state serialization for replay and network sync.

use crate::ImpetusError;

/// Serialize world state to bytes (for network sync / replay).
pub fn serialize_world(_world: &crate::PhysicsWorld) -> Result<Vec<u8>, ImpetusError> {
    // TODO: Serialize all body positions/velocities + collider state via bincode
    Ok(vec![])
}

/// Deserialize world state from bytes.
pub fn deserialize_world(
    _world: &mut crate::PhysicsWorld,
    _data: &[u8],
) -> Result<(), ImpetusError> {
    // TODO: Restore body/collider state from bincode
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{PhysicsWorld, WorldConfig};

    #[test]
    fn roundtrip_empty_world() {
        let world = PhysicsWorld::new(WorldConfig::default());
        let data = serialize_world(&world).unwrap();
        let mut world2 = PhysicsWorld::new(WorldConfig::default());
        deserialize_world(&mut world2, &data).unwrap();
    }
}
