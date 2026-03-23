//! World state serialization for replay and network sync.
//!
//! Captures a complete snapshot of the physics world (bodies, colliders, joints,
//! config) as bytes. Use for save/load, deterministic replay, and network state
//! transfer.

use serde::{Deserialize, Serialize};

use crate::ImpetusError;
use crate::PhysicsWorld;
use crate::body::{BodyDesc, BodyHandle};
use crate::collider::{ColliderDesc, ColliderHandle};
use crate::config::WorldConfig;
use crate::joint::{JointDesc, JointHandle};
use crate::particle::{Particle, ParticleEmitter};

/// A serializable snapshot of the entire physics world.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorldSnapshot {
    pub config: WorldConfig,
    pub next_particle_id: u64,
    pub next_emitter_id: u64,
    pub bodies: Vec<BodySnapshot>,
    pub colliders: Vec<ColliderSnapshot>,
    pub joints: Vec<JointSnapshot>,
    pub particles: Vec<Particle>,
    pub emitters: Vec<ParticleEmitter>,
}

/// Serializable body state (position, velocity, etc.).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BodySnapshot {
    pub handle: BodyHandle,
    pub desc: BodyDesc,
    pub position: [f64; 3],
    pub rotation: f64,
    pub linear_velocity: [f64; 3],
    pub angular_velocity: f64,
}

/// Serializable collider (shape, material, attachment).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColliderSnapshot {
    pub handle: ColliderHandle,
    pub body: BodyHandle,
    pub desc: ColliderDesc,
}

/// Serializable joint.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct JointSnapshot {
    pub handle: JointHandle,
    pub desc: JointDesc,
}

/// Serialize world state to bytes.
pub fn serialize_world(world: &PhysicsWorld) -> Result<Vec<u8>, ImpetusError> {
    let snapshot = world.snapshot();
    bitcode::serialize(&snapshot).map_err(|e| ImpetusError::Serialize(e.to_string()))
}

/// Deserialize world state from bytes and restore it.
pub fn deserialize_world(world: &mut PhysicsWorld, data: &[u8]) -> Result<(), ImpetusError> {
    let snapshot: WorldSnapshot =
        bitcode::deserialize(data).map_err(|e| ImpetusError::Deserialize(e.to_string()))?;
    world.restore(&snapshot);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::BodyType;
    use crate::collider::ColliderShape;
    use crate::material::PhysicsMaterial;

    #[test]
    fn roundtrip_empty_world() {
        let world = PhysicsWorld::new(WorldConfig::default());
        let data = serialize_world(&world).unwrap();
        let mut world2 = PhysicsWorld::new(WorldConfig::default());
        deserialize_world(&mut world2, &data).unwrap();
        assert_eq!(world2.body_count(), 0);
        assert_eq!(world2.config().timestep, world.config().timestep);
    }

    #[test]
    fn roundtrip_with_bodies() {
        let mut world = PhysicsWorld::new(WorldConfig::default());

        let ball = world.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [5.0, 10.0, 0.0],
            ..BodyDesc::default()
        });
        world.add_collider(
            ball,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 1.0 },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::rubber(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );

        let floor = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        world.add_collider(
            floor,
            ColliderDesc {
                shape: ColliderShape::Box {
                    half_extents: [50.0, 0.5, 0.0],
                },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::wood(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );

        for _ in 0..10 {
            world.step();
        }

        let data = serialize_world(&world).unwrap();
        assert!(!data.is_empty());

        let mut world2 = PhysicsWorld::new(WorldConfig::default());
        deserialize_world(&mut world2, &data).unwrap();

        assert_eq!(world2.body_count(), 2);
        assert_eq!(world2.current_step(), world.current_step());
    }

    #[cfg(feature = "2d")]
    #[test]
    fn roundtrip_preserves_positions() {
        let mut world = PhysicsWorld::new(WorldConfig::default());

        let body = world.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 10.0, 0.0],
            ..BodyDesc::default()
        });
        world.add_collider(
            body,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 0.5 },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );

        for _ in 0..30 {
            world.step();
        }

        let state_before = world.get_body_state(body).unwrap();
        let data = serialize_world(&world).unwrap();
        let mut world2 = PhysicsWorld::new(WorldConfig::default());
        deserialize_world(&mut world2, &data).unwrap();

        let state_after = world2.get_body_state(body).unwrap();
        assert!(
            (state_before.position[1] - state_after.position[1]).abs() < 1e-10,
            "position should be preserved"
        );
        assert!(
            (state_before.linear_velocity[1] - state_after.linear_velocity[1]).abs() < 1e-10,
            "velocity should be preserved"
        );
    }

    #[cfg(feature = "2d")]
    #[test]
    fn roundtrip_continues_simulation() {
        let mut world = PhysicsWorld::new(WorldConfig {
            gravity: [0.0, 0.0, 0.0],
            ..WorldConfig::default()
        });

        let body = world.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            linear_velocity: [1.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        world.add_collider(
            body,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 0.5 },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );

        for _ in 0..10 {
            world.step();
        }
        let pos_at_save = world.get_body_state(body).unwrap().position[0];

        let data = serialize_world(&world).unwrap();
        let mut world2 = PhysicsWorld::new(WorldConfig::default());
        deserialize_world(&mut world2, &data).unwrap();

        for _ in 0..10 {
            world2.step();
        }

        let pos_after = world2.get_body_state(body).unwrap().position[0];
        assert!(
            pos_after > pos_at_save,
            "body should continue moving after restore"
        );
    }

    #[test]
    fn snapshot_serde_json() {
        let snapshot = WorldSnapshot {
            config: WorldConfig::default(),
            next_particle_id: 0,
            next_emitter_id: 0,
            bodies: vec![],
            colliders: vec![],
            joints: vec![],
            particles: vec![],
            emitters: vec![],
        };
        let json = serde_json::to_string(&snapshot).unwrap();
        let back: WorldSnapshot = serde_json::from_str(&json).unwrap();
        assert_eq!(back.next_particle_id, 0);
    }

    /// Step-exact determinism: serialize at step N, restore, step 100 more,
    /// compare against a world that was never serialized.
    #[cfg(feature = "2d")]
    #[test]
    fn step_exact_determinism() {
        let config = WorldConfig::default();

        // World A: run 50 steps, serialize, restore, run 100 more
        let mut world_a = PhysicsWorld::new(config.clone());
        let ball_a = world_a.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [3.0, 15.0, 0.0],
            linear_velocity: [1.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        world_a.add_collider(
            ball_a,
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
        let floor_a = world_a.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        world_a.add_collider(
            floor_a,
            ColliderDesc {
                shape: ColliderShape::Box {
                    half_extents: [50.0, 0.5, 0.0],
                },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::wood(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        for _ in 0..50 {
            world_a.step();
        }
        let data = serialize_world(&world_a).unwrap();
        let mut world_a_restored = PhysicsWorld::new(WorldConfig::default());
        deserialize_world(&mut world_a_restored, &data).unwrap();
        for _ in 0..100 {
            world_a_restored.step();
        }

        // World B: run 150 steps straight (no serialize/restore)
        let mut world_b = PhysicsWorld::new(config);
        let ball_b = world_b.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [3.0, 15.0, 0.0],
            linear_velocity: [1.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        world_b.add_collider(
            ball_b,
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
        let floor_b = world_b.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..BodyDesc::default()
        });
        world_b.add_collider(
            floor_b,
            ColliderDesc {
                shape: ColliderShape::Box {
                    half_extents: [50.0, 0.5, 0.0],
                },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::wood(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        for _ in 0..150 {
            world_b.step();
        }

        // Compare: positions and velocities must match exactly
        let state_a = world_a_restored.get_body_state(ball_a).unwrap();
        let state_b = world_b.get_body_state(ball_b).unwrap();

        assert!(
            (state_a.position[0] - state_b.position[0]).abs() < 1e-10,
            "x position diverged: {} vs {}",
            state_a.position[0],
            state_b.position[0]
        );
        assert!(
            (state_a.position[1] - state_b.position[1]).abs() < 1e-10,
            "y position diverged: {} vs {}",
            state_a.position[1],
            state_b.position[1]
        );
        assert!(
            (state_a.linear_velocity[0] - state_b.linear_velocity[0]).abs() < 1e-10,
            "x velocity diverged"
        );
        assert!(
            (state_a.linear_velocity[1] - state_b.linear_velocity[1]).abs() < 1e-10,
            "y velocity diverged"
        );
    }
}
