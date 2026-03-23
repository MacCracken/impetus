//! # Impetus — Physics Engine for AGNOS
//!
//! Impetus (Latin: *impetus* — driving force) provides 2D/3D rigid body
//! physics simulation for the AGNOS ecosystem. Built on
//! [hisab](https://crates.io/crates/hisab) for math — no external physics
//! engine dependencies.
//!
//! ## Consumers
//!
//! - **Kiran** — game engine (ECS + physics)
//! - **Aethersafha** — desktop compositor (window animations, spring physics)
//! - **Simulation workloads** — headless agent training environments

pub mod body;
pub mod collider;
pub mod config;
pub mod event;
pub mod force;
pub mod joint;
pub mod material;
pub mod particle;
pub mod query;
#[cfg(feature = "serialize")]
pub mod serialize;
pub mod spring;
pub mod units;
pub mod world;

#[cfg(all(feature = "2d", not(feature = "3d")))]
mod backend_2d;

#[cfg(feature = "3d")]
mod backend_3d;

mod arena;
mod spatial_hash;

mod error;
pub use error::ImpetusError;

// Body
pub use body::{BodyDesc, BodyHandle, BodyState, BodyType};

// Collider
pub use collider::{ColliderDesc, ColliderHandle, ColliderShape};

// Config
pub use config::WorldConfig;

// Events
pub use event::{CollisionEvent, ContactData, ContactPoint};

// Forces
pub use force::{Force, Impulse, Torque};

// Joints
pub use joint::{JointDesc, JointHandle, JointMotor, JointType};

// Material
pub use material::{CombineRule, PhysicsMaterial};

// Particles
pub use particle::{EmitterHandle, ForceField, Particle, ParticleEmitter, ParticleHandle, SubEmitter};

// Queries
pub use query::{PointQuery, RayHit};

// Springs (standalone, no world needed)
pub use spring::{Spring, Spring2d, Spring3d};

// Units
pub use units::{PhysicsUnit, Quantity};

// World
pub use world::PhysicsWorld;

pub type Result<T> = std::result::Result<T, ImpetusError>;

// Compile-time Send + Sync assertions for public types.
const _: () = {
    #[allow(dead_code)]
    fn assert_send_sync<T: Send + Sync>() {}
    #[allow(dead_code)]
    fn assertions() {
        assert_send_sync::<PhysicsWorld>();
        assert_send_sync::<WorldConfig>();
        assert_send_sync::<BodyDesc>();
        assert_send_sync::<BodyHandle>();
        assert_send_sync::<BodyState>();
        assert_send_sync::<ColliderDesc>();
        assert_send_sync::<ColliderHandle>();
        assert_send_sync::<CollisionEvent>();
        assert_send_sync::<ContactData>();
        assert_send_sync::<ContactPoint>();
        assert_send_sync::<Force>();
        assert_send_sync::<Impulse>();
        assert_send_sync::<Torque>();
        assert_send_sync::<JointDesc>();
        assert_send_sync::<JointHandle>();
        assert_send_sync::<PhysicsMaterial>();
        assert_send_sync::<RayHit>();
        assert_send_sync::<PointQuery>();
        assert_send_sync::<Quantity>();
        assert_send_sync::<Particle>();
        assert_send_sync::<ParticleHandle>();
        assert_send_sync::<ParticleEmitter>();
        assert_send_sync::<EmitterHandle>();
        assert_send_sync::<ForceField>();
        assert_send_sync::<SubEmitter>();
        assert_send_sync::<Spring>();
        assert_send_sync::<Spring2d>();
        assert_send_sync::<Spring3d>();
        assert_send_sync::<ImpetusError>();
    }
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn create_default_world() {
        let config = WorldConfig::default();
        let world = PhysicsWorld::new(config);
        assert_eq!(world.body_count(), 0);
    }
}
