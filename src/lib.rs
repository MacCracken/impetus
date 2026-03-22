//! # Impetus — Physics Engine for AGNOS
//!
//! Impetus (Latin: driving force — the medieval theory of why objects keep
//! moving) provides 2D/3D rigid body physics simulation for the AGNOS
//! ecosystem. It wraps [rapier](https://rapier.rs/) with AGNOS-specific
//! integration: deterministic stepping, serializable state, TOML scene
//! loading, and unit-aware quantities.
//!
//! ## Consumers
//!
//! - **Joshua** — game engine (ECS + physics)
//! - **Aethersafha** — desktop compositor (window animations, spring physics)
//! - **Simulation workloads** — headless agent training environments

pub mod body;
pub mod collider;
pub mod joint;
pub mod world;
pub mod query;
pub mod material;
pub mod force;
pub mod event;
pub mod config;
#[cfg(feature = "serialize")]
pub mod serialize;
pub mod units;

mod error;
pub use error::ImpetusError;

pub use body::{BodyDesc, BodyHandle, BodyType};
pub use collider::{ColliderDesc, ColliderHandle, ColliderShape};
pub use config::WorldConfig;
pub use event::CollisionEvent;
pub use force::{Force, Impulse};
pub use joint::{JointDesc, JointHandle, JointType};
pub use material::PhysicsMaterial;
pub use query::RayHit;
pub use world::PhysicsWorld;

pub type Result<T> = std::result::Result<T, ImpetusError>;

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
