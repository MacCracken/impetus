//! Physics world — the simulation container.

#[cfg(any(feature = "2d", feature = "3d"))]
use crate::arena::ArenaHandle;
use crate::body::{BodyDesc, BodyHandle, BodyState};
use crate::collider::{ColliderDesc, ColliderHandle};
use crate::config::WorldConfig;
use crate::event::CollisionEvent;
use crate::force::{Force, Impulse, Torque};
use crate::joint::{JointDesc, JointHandle};
use crate::particle::{EmitterHandle, Particle, ParticleEmitter, ParticleHandle};
use crate::query::RayHit;
#[cfg(not(any(feature = "2d", feature = "3d")))]
use crate::ImpetusError;

/// The physics world — owns all bodies, colliders, joints, and the simulation pipeline.
pub struct PhysicsWorld {
    config: WorldConfig,
    next_particle_id: u64,
    next_emitter_id: u64,
    collision_events: Vec<CollisionEvent>,
    particles: Vec<Particle>,
    emitters: Vec<ParticleEmitter>,

    #[cfg(all(feature = "2d", not(feature = "3d")))]
    backend_2d: crate::backend_2d::PhysicsState2d,

    #[cfg(feature = "3d")]
    backend_3d: crate::backend_3d::PhysicsState3d,

    #[cfg(not(any(feature = "2d", feature = "3d")))]
    body_count: usize,
    #[cfg(not(any(feature = "2d", feature = "3d")))]
    next_stub_id: u64,
}

impl PhysicsWorld {
    /// Create a new physics world.
    pub fn new(config: WorldConfig) -> Self {
        Self {
            config,
            next_particle_id: 0,
            next_emitter_id: 0,
            collision_events: vec![],
            particles: Vec::new(),
            emitters: Vec::new(),

            #[cfg(all(feature = "2d", not(feature = "3d")))]
            backend_2d: crate::backend_2d::PhysicsState2d::new(),

            #[cfg(feature = "3d")]
            backend_3d: crate::backend_3d::PhysicsState3d::new(),

            #[cfg(not(any(feature = "2d", feature = "3d")))]
            body_count: 0,
            #[cfg(not(any(feature = "2d", feature = "3d")))]
            next_stub_id: 0,
        }
    }

    /// Step the simulation by one fixed timestep.
    pub fn step(&mut self) {
        self.config.step += 1;

        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.collision_events = self.backend_2d.step(
                self.config.gravity,
                self.config.timestep,
                self.config.velocity_iterations,
                self.config.position_iterations,
                self.config.position_slop,
                self.config.position_correction,
                self.config.max_velocity,
            );
        }

        #[cfg(feature = "3d")]
        {
            self.collision_events = self.backend_3d.step(
                self.config.gravity,
                self.config.timestep,
                self.config.velocity_iterations,
                self.config.position_iterations,
                self.config.position_slop,
                self.config.position_correction,
                self.config.max_velocity,
            );
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            self.collision_events.clear();
        }

        // Step particles
        self.step_particles();
    }

    fn step_particles(&mut self) {
        let dt = self.config.timestep;
        let gravity = self.config.gravity;

        // Spawn from emitters — low-discrepancy sequence for spread
        let mut new_particles = Vec::new();
        for emitter in &mut self.emitters {
            if !emitter.active {
                continue;
            }
            emitter.accumulator += dt;
            let interval = 1.0 / emitter.rate;
            while emitter.accumulator >= interval {
                emitter.accumulator -= interval;
                // Golden ratio based spread for both axes
                let sx = (self.next_particle_id as f64 * 0.618033).fract() * 2.0 - 1.0;
                let sy = (self.next_particle_id as f64 * 0.381966).fract() * 2.0 - 1.0;
                let sz = (self.next_particle_id as f64 * 0.291796).fract() * 2.0 - 1.0;
                let vx = emitter.velocity[0] + emitter.velocity_spread[0] * sx;
                let vy = emitter.velocity[1] + emitter.velocity_spread[1] * sy;
                let vz = emitter.velocity[2] + emitter.velocity_spread[2] * sz;

                let mut p = Particle::new(emitter.position, [vx, vy, vz], emitter.particle_lifetime)
                    .with_radius(emitter.particle_radius)
                    .with_restitution(emitter.particle_restitution)
                    .with_gravity_scale(emitter.particle_gravity_scale)
                    .with_damping(emitter.particle_damping);
                p.handle = ParticleHandle(self.next_particle_id);
                self.next_particle_id = self.next_particle_id.wrapping_add(1);
                new_particles.push(p);
            }
        }
        self.particles.extend(new_particles);

        // Integrate particles
        for p in &mut self.particles {
            if !p.is_alive() {
                continue;
            }

            // Gravity
            p.velocity[0] += gravity[0] * p.gravity_scale * dt;
            p.velocity[1] += gravity[1] * p.gravity_scale * dt;
            p.velocity[2] += gravity[2] * p.gravity_scale * dt;

            // Quadratic drag (proportional to speed²)
            if p.drag > 0.0 {
                let speed_sq = p.velocity[0] * p.velocity[0]
                    + p.velocity[1] * p.velocity[1]
                    + p.velocity[2] * p.velocity[2];
                if speed_sq > 1e-20 {
                    let speed = speed_sq.sqrt();
                    let drag_force = p.drag * speed_sq;
                    let factor = (drag_force * dt / speed).min(speed); // clamp to not reverse
                    p.velocity[0] -= (p.velocity[0] / speed) * factor;
                    p.velocity[1] -= (p.velocity[1] / speed) * factor;
                    p.velocity[2] -= (p.velocity[2] / speed) * factor;
                }
            }

            // Linear damping
            let damping_factor = 1.0 / (1.0 + dt * p.damping);
            p.velocity[0] *= damping_factor;
            p.velocity[1] *= damping_factor;
            p.velocity[2] *= damping_factor;

            // Integrate position
            p.position[0] += p.velocity[0] * dt;
            p.position[1] += p.velocity[1] * dt;
            p.position[2] += p.velocity[2] * dt;

            // Decay lifetime
            p.lifetime -= dt;
        }

        // Collide particles with rigid body colliders
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        self.collide_particles();

        #[cfg(feature = "3d")]
        self.collide_particles();

        // Remove dead particles
        self.particles.retain(|p| p.is_alive());
    }

    #[cfg(all(feature = "2d", not(feature = "3d")))]
    fn collide_particles(&mut self) {
        // Pre-compute collider world positions and AABBs to avoid redundant
        // sin_cos calculations and enable AABB pre-filtering.
        struct ColliderInfo {
            shape: crate::collider::ColliderShape,
            pos: [f64; 2],
            rotation: f64,
            aabb_min: [f64; 2],
            aabb_max: [f64; 2],
        }
        let infos: Vec<ColliderInfo> = self
            .backend_2d
            .colliders
            .values()
            .filter_map(|collider| {
                if collider.is_sensor {
                    return None;
                }
                let rb = self.backend_2d.bodies.get(ArenaHandle(collider.body.0))?;
                let (sin, cos) = rb.rotation.sin_cos();
                let cx = rb.position[0] + cos * collider.offset[0] - sin * collider.offset[1];
                let cy = rb.position[1] + sin * collider.offset[0] + cos * collider.offset[1];
                let aabb = collider.world_aabb(rb.position, rb.rotation);
                Some(ColliderInfo {
                    shape: collider.shape.clone(),
                    pos: [cx, cy],
                    rotation: rb.rotation,
                    aabb_min: aabb.min,
                    aabb_max: aabb.max,
                })
            })
            .collect();

        for p in &mut self.particles {
            if !p.is_alive() || p.radius <= 0.0 {
                continue;
            }

            let px = p.position[0];
            let py = p.position[1];
            let pr = p.radius;

            for info in &infos {
                // AABB pre-check: skip colliders whose AABB doesn't overlap the
                // particle's bounding box.
                if px + pr < info.aabb_min[0]
                    || px - pr > info.aabb_max[0]
                    || py + pr < info.aabb_min[1]
                    || py - pr > info.aabb_max[1]
                {
                    continue;
                }

                let contact = particle_vs_collider_2d(
                    [px, py],
                    pr,
                    &info.shape,
                    info.pos,
                    info.rotation,
                );

                if let Some((normal, depth)) = contact {
                    // Separate particle from collider
                    p.position[0] += normal[0] * depth;
                    p.position[1] += normal[1] * depth;

                    // Reflect velocity
                    let vel_dot_n = p.velocity[0] * normal[0] + p.velocity[1] * normal[1];
                    if vel_dot_n < 0.0 {
                        p.velocity[0] -= (1.0 + p.restitution) * vel_dot_n * normal[0];
                        p.velocity[1] -= (1.0 + p.restitution) * vel_dot_n * normal[1];
                    }
                }
            }
        }
    }

    #[cfg(feature = "3d")]
    fn collide_particles(&mut self) {
        for p in &mut self.particles {
            if !p.is_alive() || p.radius <= 0.0 {
                continue;
            }

            for collider in self.backend_3d.colliders.values() {
                if collider.is_sensor {
                    continue;
                }
                let rb = match self.backend_3d.bodies.get(ArenaHandle(collider.body.0)) {
                    Some(b) => b,
                    None => continue,
                };

                let contact = particle_vs_collider_3d(
                    p.position,
                    p.radius,
                    &collider.shape,
                    rb.position.into(),
                    collider.offset.into(),
                );

                if let Some((normal, depth)) = contact {
                    // Separate particle from collider
                    p.position[0] += normal[0] * depth;
                    p.position[1] += normal[1] * depth;
                    p.position[2] += normal[2] * depth;

                    // Reflect velocity
                    let vel_dot_n = p.velocity[0] * normal[0]
                        + p.velocity[1] * normal[1]
                        + p.velocity[2] * normal[2];
                    if vel_dot_n < 0.0 {
                        p.velocity[0] -= (1.0 + p.restitution) * vel_dot_n * normal[0];
                        p.velocity[1] -= (1.0 + p.restitution) * vel_dot_n * normal[1];
                        p.velocity[2] -= (1.0 + p.restitution) * vel_dot_n * normal[2];
                    }
                }
            }
        }
    }

    /// Current simulation step number.
    #[must_use]
    pub fn current_step(&self) -> u64 {
        self.config.step
    }

    /// Timestep duration.
    #[must_use]
    pub fn timestep(&self) -> f64 {
        self.config.timestep
    }

    /// Add a rigid body.
    pub fn add_body(&mut self, desc: BodyDesc) -> BodyHandle {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.add_body(&desc)
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d.add_body(&desc)
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = desc;
            self.body_count += 1;
            let id = self.next_stub_id;
            self.next_stub_id = self.next_stub_id.wrapping_add(1);
            BodyHandle(id)
        }
    }

    /// Add a collider attached to a body.
    pub fn add_collider(&mut self, body: BodyHandle, desc: ColliderDesc) -> ColliderHandle {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.add_collider(body, &desc)
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d.add_collider(body, &desc)
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (body, desc);
            let id = self.next_stub_id;
            self.next_stub_id = self.next_stub_id.wrapping_add(1);
            ColliderHandle(id)
        }
    }

    /// Add a joint between two bodies.
    pub fn add_joint(&mut self, desc: JointDesc) -> JointHandle {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.add_joint(&desc)
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d.add_joint(&desc)
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = desc;
            let id = self.next_stub_id;
            self.next_stub_id = self.next_stub_id.wrapping_add(1);
            JointHandle(id)
        }
    }

    /// Apply a force to a body (applied over the next step).
    pub fn apply_force(&mut self, body: BodyHandle, force: Force) {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        self.backend_2d.apply_force(body, &force);

        #[cfg(feature = "3d")]
        self.backend_3d.apply_force(body, &force);

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (body, force);
        }
    }

    /// Apply an impulse to a body (instant velocity change).
    pub fn apply_impulse(&mut self, body: BodyHandle, impulse: Impulse) {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        self.backend_2d.apply_impulse(body, &impulse);

        #[cfg(feature = "3d")]
        self.backend_3d.apply_impulse(body, &impulse);

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (body, impulse);
        }
    }

    /// Apply torque to a body.
    pub fn apply_torque(&mut self, body: BodyHandle, torque: Torque) {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        self.backend_2d.apply_torque(body, &torque);

        #[cfg(feature = "3d")]
        self.backend_3d.apply_torque(body, &torque);

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (body, torque);
        }
    }

    /// Remove a body and its attached colliders.
    pub fn remove_body(&mut self, handle: BodyHandle) -> crate::Result<()> {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.remove_body(handle);
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d.remove_body(handle);
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
        origin: [f64; 3],
        direction: [f64; 3],
        max_dist: f64,
    ) -> Option<RayHit> {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.raycast(origin, direction, max_dist)
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d.raycast(origin, direction, max_dist)
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (origin, direction, max_dist);
            None
        }
    }

    /// Find all colliders overlapping a sphere at the given position.
    pub fn overlap_sphere(&self, center: [f64; 3], radius: f64) -> Vec<ColliderHandle> {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.overlap_sphere(center, radius)
        }

        #[cfg(feature = "3d")]
        {
            // 3D overlap not implemented yet
            let _ = (center, radius);
            vec![]
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (center, radius);
            vec![]
        }
    }

    /// Find all colliders overlapping an AABB.
    pub fn overlap_aabb(&self, min: [f64; 3], max: [f64; 3]) -> Vec<ColliderHandle> {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.overlap_aabb(min, max)
        }

        #[cfg(feature = "3d")]
        {
            // 3D overlap not implemented yet
            let _ = (min, max);
            vec![]
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (min, max);
            vec![]
        }
    }

    /// Get collision events from the last step.
    #[must_use]
    pub fn collision_events(&self) -> &[CollisionEvent] {
        &self.collision_events
    }

    /// Number of bodies in the world.
    #[must_use]
    pub fn body_count(&self) -> usize {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.body_count()
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d.body_count()
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            self.body_count
        }
    }

    /// Get the world configuration.
    #[must_use]
    pub fn config(&self) -> &WorldConfig {
        &self.config
    }

    /// Read the current state of a body from the simulation.
    pub fn get_body_state(&self, handle: BodyHandle) -> crate::Result<BodyState> {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.get_body_state(handle)
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d.get_body_state(handle)
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            Err(ImpetusError::BodyNotFound(format!("{:?}", handle)))
        }
    }

    /// Set the state of a body (teleport, change velocity, etc.).
    pub fn set_body_state(&mut self, handle: BodyHandle, state: &BodyState) -> crate::Result<()> {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.set_body_state(handle, state)
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d.set_body_state(handle, state)
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (handle, state);
            Err(ImpetusError::BodyNotFound(format!("{:?}", handle)))
        }
    }

    /// Change the body type (Static, Dynamic, Kinematic).
    pub fn set_body_type(&mut self, handle: BodyHandle, body_type: crate::body::BodyType) -> crate::Result<()> {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.set_body_type(handle, body_type)
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d.set_body_type(handle, body_type)
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (handle, body_type);
            Err(ImpetusError::BodyNotFound(format!("{:?}", handle)))
        }
    }

    /// Spawn a particle. Returns its handle.
    pub fn spawn_particle(&mut self, mut particle: Particle) -> ParticleHandle {
        let handle = ParticleHandle(self.next_particle_id);
        self.next_particle_id = self.next_particle_id.wrapping_add(1);
        particle.handle = handle;
        self.particles.push(particle);
        handle
    }

    /// Add a particle emitter. Returns its handle.
    pub fn add_emitter(&mut self, mut emitter: ParticleEmitter) -> EmitterHandle {
        let handle = EmitterHandle(self.next_emitter_id);
        self.next_emitter_id = self.next_emitter_id.wrapping_add(1);
        emitter.handle = handle;
        self.emitters.push(emitter);
        handle
    }

    /// Remove a particle emitter by handle.
    pub fn remove_emitter(&mut self, handle: EmitterHandle) {
        self.emitters.retain(|e| e.handle != handle);
    }

    /// Get all live particles (read-only).
    #[must_use]
    pub fn particles(&self) -> &[Particle] {
        &self.particles
    }

    /// Number of live particles.
    #[must_use]
    pub fn particle_count(&self) -> usize {
        self.particles.len()
    }

    /// Remove all particles.
    pub fn clear_particles(&mut self) {
        self.particles.clear();
    }

    /// Remove all emitters.
    pub fn clear_emitters(&mut self) {
        self.emitters.clear();
    }

    /// Capture a snapshot of the current world state for serialization.
    #[cfg(feature = "serialize")]
    pub fn snapshot(&self) -> crate::serialize::WorldSnapshot {
        use crate::serialize::*;

        let mut bodies = Vec::new();
        let mut colliders = Vec::new();
        let mut joints = Vec::new();

        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            for rb in self.backend_2d.bodies.values() {
                let pos3 = [rb.position[0], rb.position[1], 0.0];
                let vel3 = [rb.linear_velocity[0], rb.linear_velocity[1], 0.0];
                bodies.push(BodySnapshot {
                    handle: rb.handle,
                    desc: BodyDesc {
                        body_type: rb.body_type,
                        position: pos3,
                        rotation: rb.rotation,
                        linear_velocity: vel3,
                        angular_velocity: rb.angular_velocity,
                        linear_damping: rb.linear_damping,
                        angular_damping: rb.angular_damping,
                        fixed_rotation: rb.fixed_rotation,
                        gravity_scale: Some(rb.gravity_scale),
                    },
                    position: pos3,
                    rotation: rb.rotation,
                    linear_velocity: vel3,
                    angular_velocity: rb.angular_velocity,
                });
            }

            for c in self.backend_2d.colliders.values() {
                colliders.push(ColliderSnapshot {
                    handle: c.handle,
                    body: c.body,
                    desc: ColliderDesc {
                        shape: c.shape.clone(),
                        offset: [c.offset[0], c.offset[1], 0.0],
                        material: c.material.clone(),
                        is_sensor: c.is_sensor,
                        mass: c.mass,
                        collision_layer: c.collision_layer,
                        collision_mask: c.collision_mask,
                    },
                });
            }

            for (ah, j) in self.backend_2d.joints.iter() {
                joints.push(JointSnapshot {
                    handle: JointHandle(ah.0),
                    desc: crate::joint::JointDesc {
                        body_a: j.body_a,
                        body_b: j.body_b,
                        joint_type: j.joint_type.clone(),
                        local_anchor_a: j.local_anchor_a,
                        local_anchor_b: j.local_anchor_b,
                        motor: j.motor.clone(),
                        damping: j.damping,
                    },
                });
            }
        }

        #[cfg(feature = "3d")]
        {
            for rb in self.backend_3d.bodies.values() {
                // Extract Z-rotation angle from quaternion. This is correct for
                // pure Z-axis rotations (created via DQuat::from_rotation_z).
                // For arbitrary 3D rotations, a full Euler decomposition would
                // be needed, but BodyDesc.rotation is a single f64 representing
                // only Z-rotation.
                let rotation = rb.rotation.z.atan2(rb.rotation.w) * 2.0;
                bodies.push(BodySnapshot {
                    handle: rb.handle,
                    desc: BodyDesc {
                        body_type: rb.body_type,
                        position: rb.position.into(),
                        rotation,
                        linear_velocity: rb.linear_velocity.into(),
                        angular_velocity: rb.angular_velocity.z,
                        linear_damping: rb.linear_damping,
                        angular_damping: rb.angular_damping,
                        fixed_rotation: rb.fixed_rotation,
                        gravity_scale: Some(rb.gravity_scale),
                    },
                    position: rb.position.into(),
                    rotation,
                    linear_velocity: rb.linear_velocity.into(),
                    angular_velocity: rb.angular_velocity.z,
                });
            }

            for c in self.backend_3d.colliders.values() {
                colliders.push(ColliderSnapshot {
                    handle: c.handle,
                    body: c.body,
                    desc: ColliderDesc {
                        shape: c.shape.clone(),
                        offset: c.offset.into(),
                        material: c.material.clone(),
                        is_sensor: c.is_sensor,
                        mass: c.mass,
                        collision_layer: c.collision_layer,
                        collision_mask: c.collision_mask,
                    },
                });
            }

            for (ah, j) in self.backend_3d.joints.iter() {
                joints.push(JointSnapshot {
                    handle: JointHandle(ah.0),
                    desc: crate::joint::JointDesc {
                        body_a: j.body_a,
                        body_b: j.body_b,
                        joint_type: j.joint_type.clone(),
                        local_anchor_a: [j.local_anchor_a[0], j.local_anchor_a[1]],
                        local_anchor_b: [j.local_anchor_b[0], j.local_anchor_b[1]],
                        motor: j.motor.clone(),
                        damping: j.damping,
                    },
                });
            }
        }

        WorldSnapshot {
            config: self.config.clone(),
            next_particle_id: self.next_particle_id,
            next_emitter_id: self.next_emitter_id,
            bodies,
            colliders,
            joints,
            particles: self.particles.clone(),
            emitters: self.emitters.clone(),
        }
    }

    /// Restore world state from a snapshot. Replaces all current state.
    #[cfg(feature = "serialize")]
    pub fn restore(&mut self, snapshot: &crate::serialize::WorldSnapshot) {
        self.config = snapshot.config.clone();
        self.next_particle_id = snapshot.next_particle_id;
        self.next_emitter_id = snapshot.next_emitter_id;
        self.collision_events.clear();
        self.particles = snapshot.particles.clone();
        self.emitters = snapshot.emitters.clone();

        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d = crate::backend_2d::PhysicsState2d::new();

            for bs in &snapshot.bodies {
                self.backend_2d.add_body_at(bs.handle, &bs.desc);
                if let Some(rb) = self.backend_2d.bodies.get_mut(ArenaHandle(bs.handle.0)) {
                    rb.position = [bs.position[0], bs.position[1]];
                    rb.rotation = bs.rotation;
                    rb.linear_velocity = [bs.linear_velocity[0], bs.linear_velocity[1]];
                    rb.angular_velocity = bs.angular_velocity;
                }
            }

            for cs in &snapshot.colliders {
                self.backend_2d.add_collider_at(cs.handle, cs.body, &cs.desc);
            }

            for js in &snapshot.joints {
                self.backend_2d.add_joint_at(js.handle, &js.desc);
            }
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d = crate::backend_3d::PhysicsState3d::new();

            for bs in &snapshot.bodies {
                self.backend_3d.add_body_at(bs.handle, &bs.desc);
                if let Some(rb) = self.backend_3d.bodies.get_mut(ArenaHandle(bs.handle.0)) {
                    rb.position = bs.position.into();
                    rb.linear_velocity = bs.linear_velocity.into();
                    rb.angular_velocity.z = bs.angular_velocity;
                }
            }

            for cs in &snapshot.colliders {
                self.backend_3d.add_collider_at(cs.handle, cs.body, &cs.desc);
            }

            for js in &snapshot.joints {
                self.backend_3d.add_joint_at(js.handle, &js.desc);
            }
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            self.body_count = snapshot.bodies.len();
        }
    }
}

// ---------------------------------------------------------------------------
// Particle-vs-collider overlap (circle vs shape)
// ---------------------------------------------------------------------------

#[cfg(all(feature = "2d", not(feature = "3d")))]
fn particle_vs_collider_2d(
    pos: [f64; 2],
    radius: f64,
    shape: &crate::collider::ColliderShape,
    shape_pos: [f64; 2],
    shape_rot: f64,
) -> Option<([f64; 2], f64)> {
    use crate::collider::ColliderShape;

    match shape {
        ColliderShape::Ball {
            radius: shape_radius,
        } => {
            let dx = pos[0] - shape_pos[0];
            let dy = pos[1] - shape_pos[1];
            let dist = (dx * dx + dy * dy).sqrt();
            let overlap = (radius + shape_radius) - dist;
            if overlap > 0.0 && dist > 1e-10 {
                Some(([dx / dist, dy / dist], overlap))
            } else {
                None
            }
        }
        ColliderShape::Box { half_extents } => {
            let dx = pos[0] - shape_pos[0];
            let dy = pos[1] - shape_pos[1];
            let cx = dx.clamp(-half_extents[0], half_extents[0]);
            let cy = dy.clamp(-half_extents[1], half_extents[1]);
            let diff_x = dx - cx;
            let diff_y = dy - cy;
            let dist = (diff_x * diff_x + diff_y * diff_y).sqrt();
            if dist < radius && dist > 1e-10 {
                Some(([diff_x / dist, diff_y / dist], radius - dist))
            } else {
                None
            }
        }
        ColliderShape::Capsule {
            half_height,
            radius: cap_radius,
        } => {
            // Capsule = segment with radius. Find closest point on segment to particle.
            let (sin, cos) = shape_rot.sin_cos();
            let ep_a = [
                shape_pos[0] + sin * half_height,
                shape_pos[1] - cos * half_height,
            ];
            let ep_b = [
                shape_pos[0] - sin * half_height,
                shape_pos[1] + cos * half_height,
            ];
            let ab = [ep_b[0] - ep_a[0], ep_b[1] - ep_a[1]];
            let len_sq = ab[0] * ab[0] + ab[1] * ab[1];
            let closest = if len_sq < 1e-20 {
                ep_a
            } else {
                let t =
                    ((pos[0] - ep_a[0]) * ab[0] + (pos[1] - ep_a[1]) * ab[1]) / len_sq;
                let t = t.clamp(0.0, 1.0);
                [ep_a[0] + ab[0] * t, ep_a[1] + ab[1] * t]
            };
            let dx = pos[0] - closest[0];
            let dy = pos[1] - closest[1];
            let dist = (dx * dx + dy * dy).sqrt();
            let overlap = (radius + cap_radius) - dist;
            if overlap > 0.0 && dist > 1e-10 {
                Some(([dx / dist, dy / dist], overlap))
            } else {
                None
            }
        }
        _ => None,
    }
}

#[cfg(feature = "3d")]
fn particle_vs_collider_3d(
    pos: [f64; 3],
    radius: f64,
    shape: &crate::collider::ColliderShape,
    body_pos: [f64; 3],
    collider_offset: [f64; 3],
) -> Option<([f64; 3], f64)> {
    use crate::collider::ColliderShape;

    let shape_pos = [
        body_pos[0] + collider_offset[0],
        body_pos[1] + collider_offset[1],
        body_pos[2] + collider_offset[2],
    ];

    match shape {
        ColliderShape::Ball {
            radius: shape_radius,
        } => {
            let dx = pos[0] - shape_pos[0];
            let dy = pos[1] - shape_pos[1];
            let dz = pos[2] - shape_pos[2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            let overlap = (radius + shape_radius) - dist;
            if overlap > 0.0 && dist > 1e-10 {
                Some(([dx / dist, dy / dist, dz / dist], overlap))
            } else {
                None
            }
        }
        ColliderShape::Box { half_extents } => {
            let dx = pos[0] - shape_pos[0];
            let dy = pos[1] - shape_pos[1];
            let dz = pos[2] - shape_pos[2];
            let he_z = if half_extents.len() > 2 {
                half_extents[2]
            } else {
                0.0
            };
            let cx = dx.clamp(-half_extents[0], half_extents[0]);
            let cy = dy.clamp(-half_extents[1], half_extents[1]);
            let cz = dz.clamp(-he_z, he_z);
            let diff_x = dx - cx;
            let diff_y = dy - cy;
            let diff_z = dz - cz;
            let dist = (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z).sqrt();
            if dist < radius && dist > 1e-10 {
                Some((
                    [diff_x / dist, diff_y / dist, diff_z / dist],
                    radius - dist,
                ))
            } else {
                None
            }
        }
        ColliderShape::Capsule {
            half_height,
            radius: cap_radius,
        } => {
            // Capsule along Y axis
            let dy_from_center = pos[1] - shape_pos[1];
            let closest_y = dy_from_center.clamp(-half_height, *half_height);
            let closest = [shape_pos[0], shape_pos[1] + closest_y, shape_pos[2]];
            let dx = pos[0] - closest[0];
            let dy = pos[1] - closest[1];
            let dz = pos[2] - closest[2];
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            let overlap = (radius + cap_radius) - dist;
            if overlap > 0.0 && dist > 1e-10 {
                Some(([dx / dist, dy / dist, dz / dist], overlap))
            } else {
                None
            }
        }
        _ => None,
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
            position: [0.0, 10.0, 0.0],
            ..Default::default()
        });

        let _collider = world.add_collider(
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
        world.apply_force(body, Force::new(10.0, 0.0, 0.0));
        world.apply_impulse(body, Impulse::new(0.0, 5.0, 0.0));
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
            motor: None,
            damping: 0.0,
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
        assert_eq!(world.config().gravity, [0.0, -9.81, 0.0]);
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
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        let c2 = world.add_collider(
            b,
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
            position: [0.0, 10.0, 0.0],
            ..Default::default()
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
            gravity: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        let body = world.add_body(BodyDesc::default());
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

        world.apply_impulse(body, Impulse::new(10.0, 0.0, 0.0));
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
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        world.add_collider(
            floor,
            ColliderDesc {
                shape: ColliderShape::Box {
                    half_extents: [50.0, 0.5, 0.0],
                },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );

        // Falling ball
        let ball = world.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 2.0, 0.0],
            ..Default::default()
        });
        world.add_collider(
            ball,
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
            gravity: [0.0, 0.0, 0.0],
            ..Default::default()
        });

        let body = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [5.0, 0.0, 0.0],
            ..Default::default()
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

        let hit = world.raycast([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 100.0);
        assert!(hit.is_some(), "ray should hit the ball");
        let hit = hit.unwrap();
        assert!((hit.distance - 4.0).abs() < 0.1, "should hit at distance ~4 (5 - radius 1)");
    }

    // -----------------------------------------------------------------------
    // Particle tests
    // -----------------------------------------------------------------------

    #[test]
    fn spawn_particle() {
        let mut world = PhysicsWorld::new(WorldConfig {
            gravity: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        let p = Particle::new([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 1.0);
        let handle = world.spawn_particle(p);
        assert_eq!(world.particle_count(), 1);
        assert_eq!(world.particles()[0].handle, handle);
    }

    #[test]
    fn particle_falls_with_gravity() {
        let mut world = PhysicsWorld::new(WorldConfig::default());
        world.spawn_particle(Particle::new([0.0, 10.0, 0.0], [0.0, 0.0, 0.0], 5.0));

        for _ in 0..60 {
            world.step();
        }

        assert_eq!(world.particle_count(), 1);
        assert!(world.particles()[0].position[1] < 10.0, "particle should fall");
    }

    #[test]
    fn particle_expires() {
        let mut world = PhysicsWorld::new(WorldConfig {
            gravity: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        // Lifetime of 0.5 seconds = 30 frames at 60fps
        world.spawn_particle(Particle::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.5));
        assert_eq!(world.particle_count(), 1);

        for _ in 0..60 {
            world.step();
        }

        assert_eq!(world.particle_count(), 0, "particle should have expired");
    }

    #[test]
    fn emitter_spawns_particles() {
        let mut world = PhysicsWorld::new(WorldConfig {
            gravity: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        // 60 particles per second
        world.add_emitter(ParticleEmitter::new([0.0, 0.0, 0.0], [0.0, 5.0, 0.0], 60.0));

        // Step 1 second
        for _ in 0..60 {
            world.step();
        }

        // Should have spawned ~60 particles, minus any that expired
        assert!(world.particle_count() > 0, "emitter should have spawned particles");
    }

    #[cfg(feature = "2d")]
    #[test]
    fn particle_bounces_off_floor() {
        let mut world = PhysicsWorld::new(WorldConfig::default());

        // Static floor
        let floor = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        world.add_collider(
            floor,
            ColliderDesc {
                shape: ColliderShape::Box {
                    half_extents: [50.0, 0.5, 0.0],
                },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );

        // Particle falling toward floor
        world.spawn_particle(
            Particle::new([0.0, 2.0, 0.0], [0.0, -5.0, 0.0], 5.0)
                .with_radius(0.1)
                .with_restitution(0.8),
        );

        for _ in 0..120 {
            world.step();
        }

        // Particle should still be alive and above the floor
        assert_eq!(world.particle_count(), 1);
        assert!(
            world.particles()[0].position[1] > -1.0,
            "particle should have bounced, not fallen through"
        );
    }

    #[test]
    fn clear_particles() {
        let mut world = PhysicsWorld::new(WorldConfig {
            gravity: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        world.spawn_particle(Particle::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 10.0));
        world.spawn_particle(Particle::new([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], 10.0));
        assert_eq!(world.particle_count(), 2);

        world.clear_particles();
        assert_eq!(world.particle_count(), 0);
    }
}
