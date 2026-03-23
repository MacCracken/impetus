//! Physics particles — debris, fragments, projectiles.
//!
//! Lightweight particles that interact with the rigid body world: affected by
//! gravity and force fields, can collide with colliders, have finite lifetime.
//! For visual-only particles (smoke, fire, trails), use the rendering layer.

use serde::{Deserialize, Serialize};

/// A force field that affects particles within its radius.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum ForceField {
    /// Radial attraction/repulsion: F = strength * dir / |r|^falloff
    /// Positive strength = attraction, negative = repulsion.
    Radial {
        center: [f64; 3],
        strength: f64,
        falloff: f64, // 0 = constant, 1 = linear, 2 = inverse-square
        radius: f64,  // max effect radius (0 = infinite)
    },
    /// Constant directional force within a region (wind zone).
    Directional {
        force: [f64; 3],
        min: [f64; 3], // AABB min of the affected region
        max: [f64; 3], // AABB max of the affected region
    },
}

/// Describes particles to spawn when a parent particle dies.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SubEmitter {
    /// Number of particles to spawn.
    pub count: u32,
    /// Velocity spread of spawned particles.
    pub speed: f64,
    /// Lifetime of spawned particles.
    pub lifetime: f64,
    /// Radius of spawned particles.
    pub radius: f64,
    /// Gravity scale of spawned particles.
    pub gravity_scale: f64,
    /// Restitution of spawned particles.
    pub restitution: f64,
}

/// Unique handle to a particle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ParticleHandle(pub u64);

/// A physics particle — position, velocity, lifetime, optional collision.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Particle {
    pub handle: ParticleHandle,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    /// Remaining lifetime in seconds. Particle is removed when this reaches 0.
    pub lifetime: f64,
    /// Particle radius for collision detection. 0 = no collision.
    pub radius: f64,
    /// Drag coefficient — reduces velocity proportional to speed squared.
    /// Useful for projectiles that slow in air. 0 = no drag.
    pub drag: f64,
    /// Coefficient of restitution (bounciness) when hitting colliders.
    pub restitution: f64,
    /// Gravity scale (1.0 = normal gravity, 0.0 = no gravity).
    pub gravity_scale: f64,
    /// Linear damping (air resistance).
    pub damping: f64,
    /// Optional sub-emitter: spawns particles when this particle dies.
    #[serde(default)]
    pub on_death_emit: Option<SubEmitter>,
}

impl Particle {
    /// Create a new particle.
    pub fn new(position: [f64; 3], velocity: [f64; 3], lifetime: f64) -> Self {
        Self {
            handle: ParticleHandle(0), // Set by the world on spawn
            position,
            velocity,
            lifetime,
            radius: 0.05,
            drag: 0.0,
            restitution: 0.3,
            gravity_scale: 1.0,
            damping: 0.0,
            on_death_emit: None,
        }
    }

    /// Set the collision radius.
    pub fn with_radius(mut self, radius: f64) -> Self {
        self.radius = radius;
        self
    }

    /// Set the drag coefficient.
    pub fn with_drag(mut self, drag: f64) -> Self {
        self.drag = drag;
        self
    }

    /// Set the restitution (bounciness).
    pub fn with_restitution(mut self, restitution: f64) -> Self {
        self.restitution = restitution;
        self
    }

    /// Set the gravity scale.
    pub fn with_gravity_scale(mut self, scale: f64) -> Self {
        self.gravity_scale = scale;
        self
    }

    /// Set linear damping.
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.damping = damping;
        self
    }

    /// Set a sub-emitter that fires when this particle dies.
    pub fn with_sub_emitter(mut self, sub: SubEmitter) -> Self {
        self.on_death_emit = Some(sub);
        self
    }

    /// Whether this particle is still alive.
    #[must_use]
    pub fn is_alive(&self) -> bool {
        self.lifetime > 0.0
    }
}

/// Unique handle to an emitter.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct EmitterHandle(pub u64);

/// A particle emitter — spawns particles over time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ParticleEmitter {
    /// Handle for this emitter.
    pub handle: EmitterHandle,
    /// Emitter position in world space.
    pub position: [f64; 3],
    /// Base velocity for spawned particles.
    pub velocity: [f64; 3],
    /// Random spread added to velocity (per axis).
    pub velocity_spread: [f64; 3],
    /// Particles spawned per second.
    pub rate: f64,
    /// Lifetime of spawned particles in seconds.
    pub particle_lifetime: f64,
    /// Radius of spawned particles.
    pub particle_radius: f64,
    /// Restitution of spawned particles.
    pub particle_restitution: f64,
    /// Gravity scale of spawned particles.
    pub particle_gravity_scale: f64,
    /// Damping of spawned particles.
    pub particle_damping: f64,
    /// Whether the emitter is active.
    pub active: bool,
    // Internal: time accumulator for spawn timing
    pub(crate) accumulator: f64,
}

impl ParticleEmitter {
    /// Create a new emitter.
    pub fn new(position: [f64; 3], velocity: [f64; 3], rate: f64) -> Self {
        Self {
            handle: EmitterHandle(0), // Set by the world on add
            position,
            velocity,
            velocity_spread: [0.0, 0.0, 0.0],
            rate,
            particle_lifetime: 2.0,
            particle_radius: 0.05,
            particle_restitution: 0.3,
            particle_gravity_scale: 1.0,
            particle_damping: 0.0,
            active: true,
            accumulator: 0.0,
        }
    }

    /// Set velocity spread (randomization per axis).
    pub fn with_spread(mut self, spread: [f64; 3]) -> Self {
        self.velocity_spread = spread;
        self
    }

    /// Set particle lifetime.
    pub fn with_lifetime(mut self, lifetime: f64) -> Self {
        self.particle_lifetime = lifetime;
        self
    }

    /// Set particle radius.
    pub fn with_radius(mut self, radius: f64) -> Self {
        self.particle_radius = radius;
        self
    }

    /// Set particle gravity scale.
    pub fn with_gravity_scale(mut self, scale: f64) -> Self {
        self.particle_gravity_scale = scale;
        self
    }

    /// Set particle damping.
    pub fn with_damping(mut self, damping: f64) -> Self {
        self.particle_damping = damping;
        self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn particle_new() {
        let p = Particle::new([1.0, 2.0, 0.0], [3.0, 4.0, 0.0], 5.0);
        assert_eq!(p.position, [1.0, 2.0, 0.0]);
        assert_eq!(p.velocity, [3.0, 4.0, 0.0]);
        assert_eq!(p.lifetime, 5.0);
        assert!(p.is_alive());
    }

    #[test]
    fn particle_dead() {
        let p = Particle::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0);
        assert!(!p.is_alive());
    }

    #[test]
    fn particle_builder() {
        let p = Particle::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 3.0)
            .with_radius(0.1)
            .with_drag(0.5)
            .with_restitution(0.8)
            .with_gravity_scale(0.5)
            .with_damping(0.1);
        assert_eq!(p.radius, 0.1);
        assert_eq!(p.drag, 0.5);
        assert_eq!(p.restitution, 0.8);
        assert_eq!(p.gravity_scale, 0.5);
        assert_eq!(p.damping, 0.1);
    }

    #[test]
    fn particle_serde() {
        let p = Particle::new([1.0, 2.0, 0.0], [3.0, 4.0, 0.0], 5.0);
        let json = serde_json::to_string(&p).unwrap();
        let back: Particle = serde_json::from_str(&json).unwrap();
        assert_eq!(p, back);
    }

    #[test]
    fn emitter_new() {
        let e = ParticleEmitter::new([0.0, 0.0, 0.0], [0.0, 10.0, 0.0], 100.0);
        assert_eq!(e.rate, 100.0);
        assert!(e.active);
        assert_eq!(e.particle_lifetime, 2.0);
    }

    #[test]
    fn emitter_builder() {
        let e = ParticleEmitter::new([0.0, 0.0, 0.0], [0.0, 5.0, 0.0], 50.0)
            .with_spread([1.0, 2.0, 0.0])
            .with_lifetime(3.0)
            .with_radius(0.2)
            .with_gravity_scale(0.0)
            .with_damping(0.5);
        assert_eq!(e.velocity_spread, [1.0, 2.0, 0.0]);
        assert_eq!(e.particle_lifetime, 3.0);
        assert_eq!(e.particle_radius, 0.2);
        assert_eq!(e.particle_gravity_scale, 0.0);
        assert_eq!(e.particle_damping, 0.5);
    }

    #[test]
    fn emitter_serde() {
        let e = ParticleEmitter::new([1.0, 2.0, 0.0], [3.0, 4.0, 0.0], 10.0);
        let json = serde_json::to_string(&e).unwrap();
        let back: ParticleEmitter = serde_json::from_str(&json).unwrap();
        assert_eq!(e, back);
    }

    #[test]
    fn particle_handle_eq() {
        assert_eq!(ParticleHandle(1), ParticleHandle(1));
        assert_ne!(ParticleHandle(1), ParticleHandle(2));
    }
}
