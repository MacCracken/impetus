//! Physics world — the simulation container.

#[cfg(not(any(feature = "2d", feature = "3d")))]
use crate::ImpetusError;
#[cfg(any(feature = "2d", feature = "3d"))]
use crate::arena::ArenaHandle;
use crate::body::{BodyDesc, BodyHandle, BodyState};
use crate::collider::{ColliderDesc, ColliderHandle};
use crate::config::WorldConfig;
use crate::event::CollisionEvent;
use crate::force::{Force, Impulse, Torque};
use crate::joint::{JointDesc, JointHandle};
use crate::particle::{EmitterHandle, ForceField, Particle, ParticleEmitter, ParticleHandle};
use crate::query::RayHit;

/// The physics world — owns all bodies, colliders, joints, and the simulation pipeline.
pub struct PhysicsWorld {
    config: WorldConfig,
    next_particle_id: u64,
    next_emitter_id: u64,
    collision_events: Vec<CollisionEvent>,
    particles: Vec<Particle>,
    emitters: Vec<ParticleEmitter>,
    force_fields: Vec<ForceField>,

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
    /// Create a validated physics world.
    ///
    /// # Errors
    /// [`ImpetusError::InvalidConfig`](crate::ImpetusError::InvalidConfig) if
    /// [`WorldConfig::validate`] rejects the config.
    pub fn try_new(config: WorldConfig) -> crate::Result<Self> {
        config.validate()?;
        Ok(Self::new(config))
    }

    /// Create a new physics world. The config is not validated; see
    /// [`try_new`](Self::try_new).
    #[must_use]
    pub fn new(config: WorldConfig) -> Self {
        Self {
            config,
            next_particle_id: 0,
            next_emitter_id: 0,
            collision_events: vec![],
            particles: Vec::new(),
            emitters: Vec::new(),
            force_fields: Vec::new(),

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
            let num_sub_steps = self.config.sub_steps.max(1);
            let sub_dt = self.config.timestep / num_sub_steps as f64;
            for _ in 0..num_sub_steps {
                self.backend_2d.step(
                    self.config.gravity,
                    sub_dt,
                    self.config.velocity_iterations,
                    self.config.position_iterations,
                    self.config.position_slop,
                    self.config.position_correction,
                    self.config.max_velocity,
                    self.config.constraint_frequency,
                    self.config.constraint_damping_ratio,
                    self.config.broadphase,
                    self.config.solver,
                );
            }
            self.collision_events = self.backend_2d.finish_step();
        }

        // The 3D backend always uses the sequential-impulse solver; XPBD
        // (`SolverKind::Xpbd`) is 2D-only.
        #[cfg(feature = "3d")]
        {
            let num_sub_steps = self.config.sub_steps.max(1);
            let sub_dt = self.config.timestep / num_sub_steps as f64;
            for _ in 0..num_sub_steps {
                self.backend_3d.step(
                    self.config.gravity,
                    sub_dt,
                    self.config.velocity_iterations,
                    self.config.position_iterations,
                    self.config.position_slop,
                    self.config.position_correction,
                    self.config.max_velocity,
                    self.config.constraint_frequency,
                    self.config.constraint_damping_ratio,
                    self.config.broadphase,
                );
            }
            self.collision_events = self.backend_3d.finish_step();
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
            // Non-positive, NaN or infinite rates emit nothing.
            if !(emitter.rate > 0.0 && emitter.rate.is_finite()) {
                continue;
            }
            emitter.accumulator += dt;
            let interval = 1.0 / emitter.rate;
            let mut spawned = 0usize;
            while emitter.accumulator >= interval {
                if spawned >= MAX_SPAWNS_PER_EMITTER_STEP {
                    // Drop the backlog rather than carrying it forever.
                    emitter.accumulator = 0.0;
                    break;
                }
                spawned += 1;
                emitter.accumulator -= interval;
                // R3 low-discrepancy sequence (generalised golden ratio):
                // independent, evenly spread offsets on each axis.
                let n = self.next_particle_id as f64;
                let sx = (0.5 + n * R3_ALPHA[0]).fract() * 2.0 - 1.0;
                let sy = (0.5 + n * R3_ALPHA[1]).fract() * 2.0 - 1.0;
                let sz = (0.5 + n * R3_ALPHA[2]).fract() * 2.0 - 1.0;
                let vx = emitter.velocity[0] + emitter.velocity_spread[0] * sx;
                let vy = emitter.velocity[1] + emitter.velocity_spread[1] * sy;
                let vz = emitter.velocity[2] + emitter.velocity_spread[2] * sz;

                let mut p =
                    Particle::new(emitter.position, [vx, vy, vz], emitter.particle_lifetime)
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

            // Force fields
            for field in &self.force_fields {
                match field {
                    ForceField::Radial {
                        center,
                        strength,
                        falloff,
                        radius,
                    } => {
                        let dx = center[0] - p.position[0];
                        let dy = center[1] - p.position[1];
                        let dz = center[2] - p.position[2];
                        let dist_sq = dx * dx + dy * dy + dz * dz;
                        let dist = dist_sq.sqrt();
                        if dist < 1e-10 {
                            continue;
                        }
                        if *radius > 0.0 && dist > *radius {
                            continue;
                        }
                        // Softened so the force stays bounded near the centre.
                        let force_mag = strength
                            / (dist_sq + FIELD_SOFTENING * FIELD_SOFTENING).powf(falloff * 0.5);
                        let dir = [dx / dist, dy / dist, dz / dist];
                        p.velocity[0] += dir[0] * force_mag * dt;
                        p.velocity[1] += dir[1] * force_mag * dt;
                        p.velocity[2] += dir[2] * force_mag * dt;
                    }
                    ForceField::Directional { force, min, max } => {
                        if p.position[0] >= min[0]
                            && p.position[0] <= max[0]
                            && p.position[1] >= min[1]
                            && p.position[1] <= max[1]
                            && p.position[2] >= min[2]
                            && p.position[2] <= max[2]
                        {
                            p.velocity[0] += force[0] * dt;
                            p.velocity[1] += force[1] * dt;
                            p.velocity[2] += force[2] * dt;
                        }
                    }
                }
            }

            // Quadratic drag (proportional to speed²)
            if p.drag > 0.0 {
                let speed_sq = p.velocity[0] * p.velocity[0]
                    + p.velocity[1] * p.velocity[1]
                    + p.velocity[2] * p.velocity[2];
                if speed_sq > 1e-20 {
                    let speed = speed_sq.sqrt();
                    // Δv = c·v²·dt, clamped so drag never reverses the motion.
                    let factor = (p.drag * speed_sq * dt).min(speed);
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

        // Collect sub-emitter spawns from dying particles
        let mut sub_spawns = Vec::new();
        for p in &self.particles {
            if !p.is_alive()
                && let Some(sub) = &p.on_death_emit
            {
                for i in 0..sub.count {
                    let angle = (i as f64 / sub.count as f64) * std::f64::consts::TAU;
                    let vx = angle.cos() * sub.speed;
                    let vy = angle.sin() * sub.speed;
                    sub_spawns.push(
                        Particle::new(p.position, [vx, vy, 0.0], sub.lifetime)
                            .with_radius(sub.radius)
                            .with_gravity_scale(sub.gravity_scale)
                            .with_restitution(sub.restitution),
                    );
                }
            }
        }
        for mut child in sub_spawns {
            child.handle = ParticleHandle(self.next_particle_id);
            self.next_particle_id = self.next_particle_id.wrapping_add(1);
            self.particles.push(child);
        }

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

            let pr = p.radius;

            for info in &infos {
                // Use the live position: earlier contacts this step may have moved it.
                let px = p.position[0];
                let py = p.position[1];
                // AABB pre-check: skip colliders whose AABB doesn't overlap the
                // particle's bounding box.
                if px + pr < info.aabb_min[0]
                    || px - pr > info.aabb_max[0]
                    || py + pr < info.aabb_min[1]
                    || py - pr > info.aabb_max[1]
                {
                    continue;
                }

                let contact =
                    particle_vs_collider_2d([px, py], pr, &info.shape, info.pos, info.rotation);

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
                    rb.position,
                    rb.rotation,
                    collider.offset,
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

    /// Remove a single collider by handle.
    ///
    /// The parent body's mass and inertia are recomputed from remaining
    /// colliders.
    pub fn remove_collider(&mut self, handle: ColliderHandle) -> crate::Result<()> {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.remove_collider(handle)
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d.remove_collider(handle)
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = handle;
            Err(ImpetusError::ColliderNotFound(format!("{:?}", handle)))
        }
    }

    /// Remove a joint by handle.
    pub fn remove_joint(&mut self, handle: JointHandle) -> crate::Result<()> {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d.remove_joint(handle)
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d.remove_joint(handle)
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = handle;
            Err(ImpetusError::JointNotFound(format!("{:?}", handle)))
        }
    }

    /// Cast a ray and return the first hit.
    #[must_use]
    pub fn raycast(&self, origin: [f64; 3], direction: [f64; 3], max_dist: f64) -> Option<RayHit> {
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

    /// Cast a ray with a collision layer filter and return the first hit.
    ///
    /// Only colliders whose `collision_layer` has at least one bit in common
    /// with `layer_mask` are tested. Use `0xFFFF_FFFF` to hit everything
    /// (equivalent to [`raycast`](Self::raycast)).
    #[must_use]
    pub fn raycast_filtered(
        &self,
        origin: [f64; 3],
        direction: [f64; 3],
        max_dist: f64,
        layer_mask: u32,
    ) -> Option<RayHit> {
        #[cfg(all(feature = "2d", not(feature = "3d")))]
        {
            self.backend_2d
                .raycast_filtered(origin, direction, max_dist, layer_mask)
        }

        #[cfg(feature = "3d")]
        {
            self.backend_3d
                .raycast_filtered(origin, direction, max_dist, layer_mask)
        }

        #[cfg(not(any(feature = "2d", feature = "3d")))]
        {
            let _ = (origin, direction, max_dist, layer_mask);
            None
        }
    }

    /// Find all colliders overlapping a sphere at the given position.
    #[must_use]
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
    #[must_use]
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

    /// Full world state for the Cyrius port's golden-value generator. Not public
    /// API; see [`crate::oracle`].
    #[cfg(all(feature = "oracle", any(feature = "2d", feature = "3d")))]
    #[doc(hidden)]
    #[must_use]
    pub fn oracle_dump(&self) -> crate::oracle::WorldDump {
        crate::oracle::WorldDump {
            config: self.config.clone(),
            next_particle_id: self.next_particle_id,
            next_emitter_id: self.next_emitter_id,
            collision_events: self.collision_events.clone(),
            particles: self.particles.clone(),
            emitters: self.emitters.clone(),
            force_fields: self.force_fields.clone(),
            #[cfg(all(feature = "2d", not(feature = "3d")))]
            backend: self.backend_2d.oracle_dump(),
            #[cfg(feature = "3d")]
            backend: self.backend_3d.oracle_dump(),
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
    pub fn set_body_type(
        &mut self,
        handle: BodyHandle,
        body_type: crate::body::BodyType,
    ) -> crate::Result<()> {
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

    /// Add a force field that affects particles.
    pub fn add_force_field(&mut self, field: ForceField) {
        self.force_fields.push(field);
    }

    /// Remove all force fields.
    pub fn clear_force_fields(&mut self) {
        self.force_fields.clear();
    }

    /// Get all force fields (read-only).
    #[must_use]
    pub fn force_fields(&self) -> &[ForceField] {
        &self.force_fields
    }

    /// Capture a snapshot of the current world state for serialization.
    #[cfg(feature = "serialize")]
    #[must_use]
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
                        break_force: j.break_force,
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
                        break_force: j.break_force,
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

    /// Largest arena index (body, collider or joint slot) accepted from a
    /// snapshot. Restoring allocates every slot up to the largest index, so
    /// this bounds the memory an untrusted snapshot can request.
    #[cfg(feature = "serialize")]
    pub const MAX_SNAPSHOT_INDEX: u32 = 1 << 20;

    /// Restore world state from a snapshot.
    ///
    /// Deprecated: an invalid snapshot is silently ignored (the world is left
    /// unchanged). Use [`try_restore`](Self::try_restore).
    #[cfg(feature = "serialize")]
    #[deprecated(
        since = "1.4.0",
        note = "use `try_restore`, which validates the snapshot and reports errors"
    )]
    pub fn restore(&mut self, snapshot: &crate::serialize::WorldSnapshot) {
        let _ = self.try_restore(snapshot);
    }

    /// Restore world state from a snapshot, replacing the bodies, colliders,
    /// joints, particles, emitters and config.
    ///
    /// Not captured by snapshots (planned for 2.0): force fields (left as
    /// they are), contact manifolds and warm-start impulses, sleep state, and
    /// the arena free list — so the next handles allocated after a restore
    /// can differ from the original world's, and the first step after a
    /// restore re-establishes contacts. In the 3D backend only rotation about
    /// Z is captured.
    ///
    /// # Errors
    /// The world is left unchanged and an error is returned if the snapshot's
    /// config fails [`WorldConfig::validate`], a handle is duplicated or has
    /// an index above [`MAX_SNAPSHOT_INDEX`](Self::MAX_SNAPSHOT_INDEX), or a
    /// collider or joint refers to a body that isn't in the snapshot.
    #[cfg(feature = "serialize")]
    pub fn try_restore(&mut self, snapshot: &crate::serialize::WorldSnapshot) -> crate::Result<()> {
        validate_snapshot(snapshot)?;
        self.restore_unchecked(snapshot);
        Ok(())
    }

    #[cfg(feature = "serialize")]
    fn restore_unchecked(&mut self, snapshot: &crate::serialize::WorldSnapshot) {
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
                self.backend_2d
                    .add_collider_at(cs.handle, cs.body, &cs.desc);
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
                self.backend_3d
                    .add_collider_at(cs.handle, cs.body, &cs.desc);
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

/// Reject snapshots that would corrupt a world or exhaust memory on restore.
#[cfg(feature = "serialize")]
fn validate_snapshot(snapshot: &crate::serialize::WorldSnapshot) -> crate::Result<()> {
    use std::collections::BTreeSet;

    use crate::ImpetusError;
    use crate::joint::JointType;

    snapshot.config.validate()?;
    let invalid = |what: String| {
        Err(ImpetusError::Deserialize(format!(
            "invalid snapshot: {what}"
        )))
    };
    let index_ok = |raw: u64| (raw as u32) <= PhysicsWorld::MAX_SNAPSHOT_INDEX;

    let mut bodies = BTreeSet::new();
    for b in &snapshot.bodies {
        if !index_ok(b.handle.0) {
            return invalid(format!("body handle {:?} index too large", b.handle));
        }
        if !bodies.insert(b.handle) {
            return invalid(format!("duplicate body handle {:?}", b.handle));
        }
    }
    let mut colliders = BTreeSet::new();
    for c in &snapshot.colliders {
        if !index_ok(c.handle.0) {
            return invalid(format!("collider handle {:?} index too large", c.handle));
        }
        if !colliders.insert(c.handle) {
            return invalid(format!("duplicate collider handle {:?}", c.handle));
        }
        if !bodies.contains(&c.body) {
            return invalid(format!(
                "collider {:?} on missing body {:?}",
                c.handle, c.body
            ));
        }
    }
    let mut joints = BTreeSet::new();
    for j in &snapshot.joints {
        if !index_ok(j.handle.0) {
            return invalid(format!("joint handle {:?} index too large", j.handle));
        }
        if !joints.insert(j.handle) {
            return invalid(format!("duplicate joint handle {:?}", j.handle));
        }
        let needs_b = !matches!(j.desc.joint_type, JointType::Mouse { .. });
        if !bodies.contains(&j.desc.body_a) || (needs_b && !bodies.contains(&j.desc.body_b)) {
            return invalid(format!("joint {:?} on missing body", j.handle));
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Particle-vs-collider overlap (circle vs shape)
// ---------------------------------------------------------------------------

/// Softening length for radial force fields.
const FIELD_SOFTENING: f64 = 0.01;

/// Most particles one emitter may spawn in a single step.
const MAX_SPAWNS_PER_EMITTER_STEP: usize = 10_000;

/// R3 sequence increments: 1/φ₃, 1/φ₃², 1/φ₃³ with φ₃ ≈ 1.2207 the plastic-like
/// root of x⁴ = x + 1 (Roberts, "The Unreasonable Effectiveness of
/// Quasirandom Sequences").
const R3_ALPHA: [f64; 3] = [
    0.819_172_513_396_164_4,
    0.671_043_606_703_789_2,
    0.549_700_477_901_970_4,
];

/// Particle (sphere of `radius`, centre `local` in the box's frame) vs an
/// axis-aligned box centred at the origin. Returns (normal from box to
/// particle, depth). A centre inside the box is pushed out through the
/// nearest face.
#[cfg(any(feature = "2d", feature = "3d"))]
fn particle_vs_local_box<const N: usize>(
    local: [f64; N],
    radius: f64,
    half_extents: [f64; N],
) -> Option<([f64; N], f64)> {
    let mut diff = [0.0; N];
    let mut dist_sq = 0.0;
    let mut inside = true;
    for i in 0..N {
        let he = half_extents[i].abs();
        let c = local[i].clamp(-he, he);
        diff[i] = local[i] - c;
        dist_sq += diff[i] * diff[i];
        inside &= local[i].abs() <= he;
    }
    if inside {
        // Least-penetration axis.
        let mut axis = 0;
        let mut pen = f64::INFINITY;
        for i in 0..N {
            let p = half_extents[i].abs() - local[i].abs();
            if p < pen {
                pen = p;
                axis = i;
            }
        }
        let mut n = [0.0; N];
        n[axis] = if local[axis] >= 0.0 { 1.0 } else { -1.0 };
        return Some((n, pen + radius));
    }
    let dist = dist_sq.sqrt();
    if dist < radius && dist > 1e-12 {
        Some((diff.map(|d| d / dist), radius - dist))
    } else {
        None
    }
}

/// Particle vs a round shape: a sphere/circle around the closest point on a
/// segment `a`–`b` (a point when `a == b`) with radius `shape_radius`.
#[cfg(any(feature = "2d", feature = "3d"))]
fn particle_vs_rounded_segment<const N: usize>(
    pos: [f64; N],
    radius: f64,
    a: [f64; N],
    b: [f64; N],
    shape_radius: f64,
) -> Option<([f64; N], f64)> {
    let mut ab = [0.0; N];
    let mut ap = [0.0; N];
    for i in 0..N {
        ab[i] = b[i] - a[i];
        ap[i] = pos[i] - a[i];
    }
    let len_sq: f64 = ab.iter().map(|x| x * x).sum();
    let t = if len_sq < 1e-20 {
        0.0
    } else {
        (ap.iter().zip(&ab).map(|(p, q)| p * q).sum::<f64>() / len_sq).clamp(0.0, 1.0)
    };
    let mut d = [0.0; N];
    for i in 0..N {
        d[i] = pos[i] - (a[i] + ab[i] * t);
    }
    let dist = d.iter().map(|x| x * x).sum::<f64>().sqrt();
    let overlap = radius + shape_radius - dist;
    if overlap > 0.0 && dist > 1e-12 {
        Some((d.map(|x| x / dist), overlap))
    } else {
        None
    }
}

#[cfg(all(feature = "2d", not(feature = "3d")))]
fn particle_vs_collider_2d(
    pos: [f64; 2],
    radius: f64,
    shape: &crate::collider::ColliderShape,
    shape_pos: [f64; 2],
    shape_rot: f64,
) -> Option<([f64; 2], f64)> {
    use crate::collider::ColliderShape;

    let (sin, cos) = shape_rot.sin_cos();
    match shape {
        ColliderShape::Ball {
            radius: shape_radius,
        } => particle_vs_rounded_segment(pos, radius, shape_pos, shape_pos, *shape_radius),
        ColliderShape::Box { half_extents } => {
            let d = [pos[0] - shape_pos[0], pos[1] - shape_pos[1]];
            let local = [cos * d[0] + sin * d[1], -sin * d[0] + cos * d[1]];
            particle_vs_local_box(local, radius, [half_extents[0], half_extents[1]])
                .map(|(n, depth)| ([cos * n[0] - sin * n[1], sin * n[0] + cos * n[1]], depth))
        }
        ColliderShape::Capsule {
            half_height,
            radius: cap_radius,
        } => {
            // Capsule axis is local Y.
            let a = [
                shape_pos[0] + sin * half_height,
                shape_pos[1] - cos * half_height,
            ];
            let b = [
                shape_pos[0] - sin * half_height,
                shape_pos[1] + cos * half_height,
            ];
            particle_vs_rounded_segment(pos, radius, a, b, *cap_radius)
        }
        ColliderShape::Segment { a, b } => {
            let wa = [
                shape_pos[0] + cos * a[0] - sin * a[1],
                shape_pos[1] + sin * a[0] + cos * a[1],
            ];
            let wb = [
                shape_pos[0] + cos * b[0] - sin * b[1],
                shape_pos[1] + sin * b[0] + cos * b[1],
            ];
            particle_vs_rounded_segment(pos, radius, wa, wb, 0.0)
        }
        _ => None,
    }
}

#[cfg(feature = "3d")]
fn particle_vs_collider_3d(
    pos: [f64; 3],
    radius: f64,
    shape: &crate::collider::ColliderShape,
    body_pos: hisab::DVec3,
    body_rot: hisab::DQuat,
    collider_offset: hisab::DVec3,
) -> Option<([f64; 3], f64)> {
    use crate::collider::ColliderShape;
    use hisab::DVec3;

    let center = body_pos + body_rot * collider_offset;
    let p = DVec3::from_array(pos);
    let to_world = |n: [f64; 3]| (body_rot * DVec3::from_array(n)).to_array();
    match shape {
        ColliderShape::Ball {
            radius: shape_radius,
        } => {
            let c = center.to_array();
            particle_vs_rounded_segment(pos, radius, c, c, *shape_radius)
        }
        ColliderShape::Box { half_extents } => {
            let local = (body_rot.inverse() * (p - center)).to_array();
            particle_vs_local_box(local, radius, *half_extents).map(|(n, d)| (to_world(n), d))
        }
        ColliderShape::Capsule {
            half_height,
            radius: cap_radius,
        } => {
            // Capsule axis is local Y.
            let axis = body_rot * DVec3::new(0.0, *half_height, 0.0);
            particle_vs_rounded_segment(
                pos,
                radius,
                (center - axis).to_array(),
                (center + axis).to_array(),
                *cap_radius,
            )
        }
        ColliderShape::Segment { a, b } => {
            let wa = (center + body_rot * DVec3::from_array(*a)).to_array();
            let wb = (center + body_rot * DVec3::from_array(*b)).to_array();
            particle_vs_rounded_segment(pos, radius, wa, wb, 0.0)
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
            break_force: None,
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
    // Force field and sub-emitter tests
    // -----------------------------------------------------------------------

    #[test]
    fn radial_force_field_attracts() {
        let mut world = PhysicsWorld::new(WorldConfig {
            gravity: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        // Attractor at origin
        world.add_force_field(crate::particle::ForceField::Radial {
            center: [0.0, 0.0, 0.0],
            strength: 100.0,
            falloff: 0.0,
            radius: 0.0,
        });
        // Particle to the right
        let h = world.spawn_particle(
            Particle::new([10.0, 0.0, 0.0], [0.0, 0.0, 0.0], 5.0).with_gravity_scale(0.0),
        );
        let x_before = world
            .particles()
            .iter()
            .find(|p| p.handle == h)
            .unwrap()
            .position[0];
        for _ in 0..10 {
            world.step();
        }
        let x_after = world
            .particles()
            .iter()
            .find(|p| p.handle == h)
            .unwrap()
            .position[0];
        assert!(x_after < x_before, "particle should move toward attractor");
    }

    #[test]
    fn radial_force_field_repels() {
        let mut world = PhysicsWorld::new(WorldConfig {
            gravity: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        // Repulsor at origin (negative strength)
        world.add_force_field(crate::particle::ForceField::Radial {
            center: [0.0, 0.0, 0.0],
            strength: -100.0,
            falloff: 0.0,
            radius: 0.0,
        });
        let h = world.spawn_particle(
            Particle::new([5.0, 0.0, 0.0], [0.0, 0.0, 0.0], 5.0).with_gravity_scale(0.0),
        );
        let x_before = world
            .particles()
            .iter()
            .find(|p| p.handle == h)
            .unwrap()
            .position[0];
        for _ in 0..10 {
            world.step();
        }
        let x_after = world
            .particles()
            .iter()
            .find(|p| p.handle == h)
            .unwrap()
            .position[0];
        assert!(
            x_after > x_before,
            "particle should be pushed away from repulsor"
        );
    }

    #[test]
    fn directional_force_field() {
        let mut world = PhysicsWorld::new(WorldConfig {
            gravity: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        // Wind zone pushing +X
        world.add_force_field(crate::particle::ForceField::Directional {
            force: [50.0, 0.0, 0.0],
            min: [-100.0, -100.0, -100.0],
            max: [100.0, 100.0, 100.0],
        });
        let h = world.spawn_particle(
            Particle::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 5.0).with_gravity_scale(0.0),
        );
        for _ in 0..10 {
            world.step();
        }
        let p = world.particles().iter().find(|p| p.handle == h).unwrap();
        assert!(
            p.position[0] > 0.0,
            "particle should be pushed in +X by wind"
        );
    }

    #[test]
    fn directional_force_field_outside() {
        let mut world = PhysicsWorld::new(WorldConfig {
            gravity: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        // Wind zone far from the particle
        world.add_force_field(crate::particle::ForceField::Directional {
            force: [50.0, 0.0, 0.0],
            min: [100.0, 100.0, 100.0],
            max: [200.0, 200.0, 200.0],
        });
        let h = world.spawn_particle(
            Particle::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 5.0).with_gravity_scale(0.0),
        );
        for _ in 0..10 {
            world.step();
        }
        let p = world.particles().iter().find(|p| p.handle == h).unwrap();
        assert!(
            p.position[0].abs() < 1e-10,
            "particle outside wind zone should not move"
        );
    }

    #[test]
    fn sub_emitter_on_death() {
        use crate::particle::SubEmitter;

        let mut world = PhysicsWorld::new(WorldConfig {
            gravity: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        // Particle with short lifetime and a sub-emitter
        let dt = world.timestep();
        world.spawn_particle(
            Particle::new([5.0, 5.0, 0.0], [0.0, 0.0, 0.0], dt * 1.5)
                .with_gravity_scale(0.0)
                .with_sub_emitter(SubEmitter {
                    count: 4,
                    speed: 3.0,
                    lifetime: 2.0,
                    radius: 0.02,
                    gravity_scale: 0.0,
                    restitution: 0.5,
                }),
        );
        assert_eq!(world.particle_count(), 1);

        // Step twice — the parent should die after ~1.5*dt and spawn 4 children
        world.step();
        world.step();

        // Parent is dead and removed, 4 children remain
        assert_eq!(
            world.particle_count(),
            4,
            "sub-emitter should have spawned 4 children"
        );
        // All children should be near the parent's last position
        for p in world.particles() {
            let dx = p.position[0] - 5.0;
            let dy = p.position[1] - 5.0;
            assert!(
                (dx * dx + dy * dy).sqrt() < 2.0,
                "children should be near parent"
            );
        }
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
        assert!(
            (hit.distance - 4.0).abs() < 0.1,
            "should hit at distance ~4 (5 - radius 1)"
        );
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
        assert!(
            world.particles()[0].position[1] < 10.0,
            "particle should fall"
        );
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
        assert!(
            world.particle_count() > 0,
            "emitter should have spawned particles"
        );
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

    // -----------------------------------------------------------------------
    // XPBD solver tests
    // -----------------------------------------------------------------------

    #[cfg(all(feature = "2d", not(feature = "3d")))]
    #[test]
    fn xpbd_gravity_moves_body() {
        let mut world = PhysicsWorld::new(WorldConfig {
            solver: crate::config::SolverKind::Xpbd,
            ..Default::default()
        });
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

        for _ in 0..60 {
            world.step();
        }

        let state = world.get_body_state(body).unwrap();
        assert!(
            state.position[1] < 10.0,
            "XPBD: body should have fallen, got y={}",
            state.position[1]
        );
    }

    #[cfg(all(feature = "2d", not(feature = "3d")))]
    #[test]
    fn xpbd_collision_prevents_fallthrough() {
        let mut world = PhysicsWorld::new(WorldConfig {
            solver: crate::config::SolverKind::Xpbd,
            velocity_iterations: 20,
            constraint_frequency: 0.0, // infinite stiffness for contacts
            sub_steps: 4,
            ..Default::default()
        });
        // Static floor
        let floor = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, -1.0, 0.0],
            ..Default::default()
        });
        world.add_collider(
            floor,
            ColliderDesc {
                shape: ColliderShape::Box {
                    half_extents: [50.0, 1.0, 0.0],
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

        for _ in 0..300 {
            world.step();
        }

        let state = world.get_body_state(ball).unwrap();
        // Ball should rest on the floor (y ≈ 0.5, since floor top is at y=0 and ball radius=0.5)
        // But at minimum it should not have fallen through (y > -1)
        assert!(
            state.position[1] > -1.0,
            "XPBD: ball should not fall through floor, got y={}",
            state.position[1]
        );
    }

    #[cfg(all(feature = "2d", not(feature = "3d")))]
    #[test]
    fn xpbd_collision_generates_events() {
        let mut world = PhysicsWorld::new(WorldConfig {
            solver: crate::config::SolverKind::Xpbd,
            ..Default::default()
        });
        let floor = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, -1.0, 0.0],
            ..Default::default()
        });
        world.add_collider(
            floor,
            ColliderDesc {
                shape: ColliderShape::Box {
                    half_extents: [50.0, 1.0, 0.0],
                },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0xFFFF_FFFF,
                collision_mask: 0xFFFF_FFFF,
            },
        );
        let ball = world.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [0.0, 1.0, 0.0],
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

        let mut had_collision = false;
        for _ in 0..120 {
            world.step();
            if !world.collision_events().is_empty() {
                had_collision = true;
            }
        }
        assert!(
            had_collision,
            "XPBD: should have generated collision events"
        );
    }

    #[cfg(all(feature = "2d", not(feature = "3d")))]
    #[test]
    fn xpbd_joint_distance() {
        use crate::joint::{JointDesc, JointType};

        let mut world = PhysicsWorld::new(WorldConfig {
            solver: crate::config::SolverKind::Xpbd,
            gravity: [0.0, 0.0, 0.0],
            velocity_iterations: 8,
            ..Default::default()
        });
        let a = world.add_body(BodyDesc {
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            ..Default::default()
        });
        world.add_collider(
            a,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 0.1 },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0,
                collision_mask: 0,
            },
        );
        let b = world.add_body(BodyDesc {
            body_type: BodyType::Dynamic,
            position: [5.0, 0.0, 0.0], // start 5 units away
            ..Default::default()
        });
        world.add_collider(
            b,
            ColliderDesc {
                shape: ColliderShape::Ball { radius: 0.1 },
                offset: [0.0, 0.0, 0.0],
                material: PhysicsMaterial::default(),
                is_sensor: false,
                mass: None,
                collision_layer: 0,
                collision_mask: 0,
            },
        );
        world.add_joint(JointDesc {
            body_a: a,
            body_b: b,
            joint_type: JointType::Distance { length: 3.0 },
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
            damping: 0.0,
            break_force: None,
        });

        for _ in 0..120 {
            world.step();
        }

        let state = world.get_body_state(b).unwrap();
        let dist =
            (state.position[0] * state.position[0] + state.position[1] * state.position[1]).sqrt();
        assert!(
            (dist - 3.0).abs() < 0.5,
            "XPBD distance joint should pull body toward length=3, got dist={}",
            dist
        );
    }
}
