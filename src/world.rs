//! Physics world — the simulation container.

use crate::body::{BodyDesc, BodyHandle, BodyState};
use crate::collider::{ColliderDesc, ColliderHandle};
use crate::config::WorldConfig;
use crate::event::CollisionEvent;
use crate::force::{Force, Impulse, Torque};
use crate::joint::{JointDesc, JointHandle};
use crate::query::RayHit;
#[cfg(not(any(feature = "2d", feature = "3d")))]
use crate::ImpetusError;

/// The physics world — owns all bodies, colliders, joints, and the simulation pipeline.
pub struct PhysicsWorld {
    config: WorldConfig,
    next_body_id: u64,
    next_collider_id: u64,
    next_joint_id: u64,
    collision_events: Vec<CollisionEvent>,

    #[cfg(feature = "2d")]
    backend: crate::backend_2d::PhysicsState2d,

    #[cfg(not(any(feature = "2d", feature = "3d")))]
    body_count: usize,
}

impl PhysicsWorld {
    /// Create a new physics world.
    pub fn new(config: WorldConfig) -> Self {
        Self {
            config,
            next_body_id: 0,
            next_collider_id: 0,
            next_joint_id: 0,
            collision_events: vec![],

            #[cfg(feature = "2d")]
            backend: crate::backend_2d::PhysicsState2d::new(),

            #[cfg(not(any(feature = "2d", feature = "3d")))]
            body_count: 0,
        }
    }

    /// Step the simulation by one fixed timestep.
    pub fn step(&mut self) {
        self.config.step += 1;

        #[cfg(feature = "2d")]
        {
            self.collision_events = self.backend.step(
                self.config.gravity,
                self.config.timestep,
                self.config.velocity_iterations,
                self.config.position_iterations,
            );
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            self.collision_events.clear();
        }
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
    pub fn add_body(&mut self, desc: BodyDesc) -> BodyHandle {
        let handle = BodyHandle(self.next_body_id);
        self.next_body_id += 1;

        #[cfg(feature = "2d")]
        self.backend.add_body(handle, &desc);

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = desc;
            self.body_count += 1;
        }

        handle
    }

    /// Add a collider attached to a body.
    pub fn add_collider(&mut self, body: BodyHandle, desc: ColliderDesc) -> ColliderHandle {
        let handle = ColliderHandle(self.next_collider_id);
        self.next_collider_id += 1;

        #[cfg(feature = "2d")]
        self.backend.add_collider(handle, body, &desc);

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (body, desc);
        }

        handle
    }

    /// Add a joint between two bodies.
    pub fn add_joint(&mut self, desc: JointDesc) -> JointHandle {
        let handle = JointHandle(self.next_joint_id);
        self.next_joint_id += 1;

        #[cfg(feature = "2d")]
        self.backend.add_joint(handle, &desc);

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = desc;
        }

        handle
    }

    /// Apply a force to a body (applied over the next step).
    pub fn apply_force(&mut self, body: BodyHandle, force: Force) {
        #[cfg(feature = "2d")]
        self.backend.apply_force(body, &force);

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (body, force);
        }
    }

    /// Apply an impulse to a body (instant velocity change).
    pub fn apply_impulse(&mut self, body: BodyHandle, impulse: Impulse) {
        #[cfg(feature = "2d")]
        self.backend.apply_impulse(body, &impulse);

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (body, impulse);
        }
    }

    /// Apply torque to a body.
    pub fn apply_torque(&mut self, body: BodyHandle, torque: Torque) {
        #[cfg(feature = "2d")]
        self.backend.apply_torque(body, &torque);

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (body, torque);
        }
    }

    /// Remove a body and its attached colliders.
    pub fn remove_body(&mut self, handle: BodyHandle) -> crate::Result<()> {
        #[cfg(feature = "2d")]
        {
            self.backend.remove_body(handle);
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = handle;
            self.body_count = self.body_count.saturating_sub(1);
        }

        Ok(())
    }

    /// Cast a ray and return the first hit.
    pub fn raycast(
        &self,
        origin: [f64; 2],
        direction: [f64; 2],
        max_dist: f64,
    ) -> Option<RayHit> {
        #[cfg(feature = "2d")]
        {
            self.backend.raycast(origin, direction, max_dist)
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (origin, direction, max_dist);
            None
        }
    }

    /// Get collision events from the last step.
    pub fn collision_events(&self) -> &[CollisionEvent] {
        &self.collision_events
    }

    /// Number of bodies in the world.
    pub fn body_count(&self) -> usize {
        #[cfg(feature = "2d")]
        {
            self.backend.body_count()
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            self.body_count
        }
    }

    /// Get the world configuration.
    pub fn config(&self) -> &WorldConfig {
        &self.config
    }

    /// Read the current state of a body from the simulation.
    pub fn get_body_state(&self, handle: BodyHandle) -> crate::Result<BodyState> {
        #[cfg(feature = "2d")]
        {
            self.backend.get_body_state(handle)
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            Err(ImpetusError::BodyNotFound(format!("{:?}", handle)))
        }
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
        // Idempotent removal
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

    // -----------------------------------------------------------------------
    // Physics validation tests (only meaningful with backend)
    // -----------------------------------------------------------------------

    #[cfg(feature = "2d")]
    #[test]
    fn gravity_moves_body() {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        let body = world.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 10.0],
            ..Default::default()
        });
        world.add_collider(
            body,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 0.5 },
                offset: [0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
            },
        );

        let initial = world.get_body_state(body).unwrap();
        assert_eq!(initial.position[1], 10.0);

        for _ in 0..60 {
            world.step();
        }

        let state = world.get_body_state(body).unwrap();
        assert!(state.position[1] < 10.0, "body should have fallen");
    }

    #[cfg(feature = "2d")]
    #[test]
    fn impulse_changes_velocity() {
        let mut world = PhysicsWorld::new(WorldConfig {
            gravity: [0.0, 0.0],
            ..Default::default()
        });
        let body = world.add_body(BodyDesc::default());
        world.add_collider(
            body,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 0.5 },
                offset: [0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
            },
        );

        world.apply_impulse(body, Impulse::new(10.0, 0.0));
        world.step();

        let state = world.get_body_state(body).unwrap();
        assert!(state.linear_velocity[0] > 0.0, "should have x velocity");
        assert!(state.position[0] > 0.0, "should have moved right");
    }

    #[cfg(feature = "2d")]
    #[test]
    fn collision_generates_events() {
        let mut world = PhysicsWorld::new(WorldConfig::default());

        // Floor
        let floor = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0],
            ..Default::default()
        });
        world.add_collider(
            floor,
            ColliderDesc {
                shape: ColliderShape::Box {
                    half_extents: [50.0, 0.5],
                },
                offset: [0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
            },
        );

        // Falling ball
        let ball = world.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 2.0],
            ..Default::default()
        });
        world.add_collider(
            ball,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 0.5 },
                offset: [0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
            },
        );

        let mut found_event = false;
        for _ in 0..120 {
            world.step();
            if !world.collision_events().is_empty() {
                found_event = true;
                break;
            }
        }
        assert!(found_event, "should have generated collision events");
    }

    #[cfg(feature = "2d")]
    #[test]
    fn raycast_hits_body() {
        let mut world = PhysicsWorld::new(WorldConfig {
            gravity: [0.0, 0.0],
            ..Default::default()
        });

        let body = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [5.0, 0.0],
            ..Default::default()
        });
        world.add_collider(
            body,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 1.0 },
                offset: [0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
            },
        );

        let hit = world.raycast([0.0, 0.0], [1.0, 0.0], 100.0);
        assert!(hit.is_some(), "ray should hit the ball");
        let hit = hit.unwrap();
        assert!((hit.distance - 4.0).abs() < 0.1, "should hit at distance ~4 (5 - radius 1)");
    }
}
