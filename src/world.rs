//! Physics world — the simulation container.

use crate::body::{BodyDesc, BodyHandle};
use crate::collider::{ColliderDesc, ColliderHandle};
use crate::config::WorldConfig;
use crate::event::CollisionEvent;
use crate::force::{Force, Impulse, Torque};
use crate::joint::{JointDesc, JointHandle};
use crate::query::RayHit;

/// The physics world — owns all bodies, colliders, joints, and the simulation pipeline.
pub struct PhysicsWorld {
    config: WorldConfig,
    next_body_id: u64,
    next_collider_id: u64,
    next_joint_id: u64,
    body_count: usize,
    collision_events: Vec<CollisionEvent>,
    // TODO: rapier2d::dynamics::RigidBodySet
    // TODO: rapier2d::geometry::ColliderSet
    // TODO: rapier2d::dynamics::JointSet
    // TODO: rapier2d::pipeline::PhysicsPipeline
}

impl PhysicsWorld {
    /// Create a new physics world.
    pub fn new(config: WorldConfig) -> Self {
        Self {
            config,
            next_body_id: 0,
            next_collider_id: 0,
            next_joint_id: 0,
            body_count: 0,
            collision_events: vec![],
        }
    }

    /// Step the simulation by one fixed timestep.
    pub fn step(&mut self) {
        self.config.step += 1;
        self.collision_events.clear();
        // TODO: Run rapier physics pipeline step
    }

    /// Current simulation step number.
    pub fn current_step(&self) -> u64 {
        self.config.step
    }

    /// Timestep duration.
    pub fn timestep(&self) -> f64 {
        self.config.timestep
    }

    /// Add a rigid body.
    pub fn add_body(&mut self, _desc: BodyDesc) -> BodyHandle {
        let handle = BodyHandle(self.next_body_id);
        self.next_body_id += 1;
        self.body_count += 1;
        // TODO: Create rapier rigid body from desc
        handle
    }

    /// Add a collider attached to a body.
    pub fn add_collider(&mut self, _body: BodyHandle, _desc: ColliderDesc) -> ColliderHandle {
        let handle = ColliderHandle(self.next_collider_id);
        self.next_collider_id += 1;
        // TODO: Create rapier collider from desc, attach to body
        handle
    }

    /// Add a joint between two bodies.
    pub fn add_joint(&mut self, _desc: JointDesc) -> JointHandle {
        let handle = JointHandle(self.next_joint_id);
        self.next_joint_id += 1;
        // TODO: Create rapier joint from desc
        handle
    }

    /// Apply a force to a body (applied over the next step).
    pub fn apply_force(&mut self, _body: BodyHandle, _force: Force) {
        // TODO: Apply to rapier body
    }

    /// Apply an impulse to a body (instant velocity change).
    pub fn apply_impulse(&mut self, _body: BodyHandle, _impulse: Impulse) {
        // TODO: Apply to rapier body
    }

    /// Apply torque to a body.
    pub fn apply_torque(&mut self, _body: BodyHandle, _torque: Torque) {
        // TODO: Apply to rapier body
    }

    /// Remove a body and its attached colliders.
    pub fn remove_body(&mut self, _handle: BodyHandle) -> crate::Result<()> {
        self.body_count = self.body_count.saturating_sub(1);
        // TODO: Remove from rapier
        Ok(())
    }

    /// Cast a ray and return the first hit.
    pub fn raycast(
        &self,
        _origin: [f64; 2],
        _direction: [f64; 2],
        _max_dist: f64,
    ) -> Option<RayHit> {
        // TODO: rapier ray cast
        None
    }

    /// Get collision events from the last step.
    pub fn collision_events(&self) -> &[CollisionEvent] {
        &self.collision_events
    }

    /// Number of bodies in the world.
    pub fn body_count(&self) -> usize {
        self.body_count
    }

    /// Get the world configuration.
    pub fn config(&self) -> &WorldConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::body::{BodyDesc, BodyType};
    use crate::collider::{ColliderDesc, ColliderShape};
    use crate::joint::JointType;
    use crate::material::PhysicsMaterial;

    #[test]
    fn world_lifecycle() {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        assert_eq!(world.body_count(), 0);

        let body = world.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 10.0],
            ..Default::default()
        });

        let _collider = world.add_collider(
            body,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 0.5 },
                offset: [0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
            },
        );

        assert_eq!(world.body_count(), 1);

        world.step();
        assert_eq!(world.current_step(), 1);

        world.remove_body(body).unwrap();
        assert_eq!(world.body_count(), 0);
    }

    #[test]
    fn deterministic_stepping() {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        for _ in 0..100 {
            world.step();
        }
        assert_eq!(world.current_step(), 100);
    }

    #[test]
    fn force_application() {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let body = world.add_body(BodyDesc::default());
        world.apply_force(body, Force::new(10.0, 0.0));
        world.apply_impulse(body, Impulse::new(0.0, 5.0));
        world.apply_torque(body, Torque::new(1.0));
        world.step();
    }

    #[test]
    fn multiple_bodies() {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        for _ in 0..50 {
            world.add_body(BodyDesc::default());
        }
        assert_eq!(world.body_count(), 50);
    }

    #[test]
    fn add_joint() {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let a = world.add_body(BodyDesc::default());
        let b = world.add_body(BodyDesc::default());
        let _joint = world.add_joint(JointDesc {
            body_a: a,
            body_b: b,
            joint_type: JointType::Fixed,
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
        });
        world.step();
    }

    #[test]
    fn raycast_returns_none() {
        let world = PhysicsWorld::new(WorldConfig::default());
        assert!(world.raycast([0.0, 0.0], [1.0, 0.0], 100.0).is_none());
    }

    #[test]
    fn collision_events_empty_after_step() {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        world.step();
        assert!(world.collision_events().is_empty());
    }

    #[test]
    fn config_accessors() {
        let config = WorldConfig {
            timestep: 1.0 / 120.0,
            ..Default::default()
        };
        let world = PhysicsWorld::new(config);
        assert_eq!(world.timestep(), 1.0 / 120.0);
        assert_eq!(world.config().gravity, [0.0, -9.81]);
    }

    #[test]
    fn remove_more_than_added() {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let body = world.add_body(BodyDesc::default());
        world.remove_body(body).unwrap();
        // Saturating sub prevents underflow
        world.remove_body(BodyHandle(999)).unwrap();
        assert_eq!(world.body_count(), 0);
    }

    #[test]
    fn unique_handles() {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let a = world.add_body(BodyDesc::default());
        let b = world.add_body(BodyDesc::default());
        assert_ne!(a, b);

        let c1 = world.add_collider(
            a,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 1.0 },
                offset: [0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
            },
        );
        let c2 = world.add_collider(
            b,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 1.0 },
                offset: [0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
            },
        );
        assert_ne!(c1, c2);
    }
}
