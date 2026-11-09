//! PhysicsState2d struct definition and add/remove/get/set methods.

use std::collections::BTreeMap;

use crate::ImpetusError;
use crate::arena::Arena;
use crate::body::{BodyDesc, BodyHandle, BodyState, BodyType};
use crate::collider::{ColliderDesc, ColliderHandle};
use crate::force::{Force, Impulse, Torque};
use crate::joint::{JointDesc, JointHandle};

use super::types::*;
use super::{body_ah, body_from, coll_ah, coll_from, joint_ah, joint_from};

// ---------------------------------------------------------------------------
// Physics state
// ---------------------------------------------------------------------------

pub(crate) struct PhysicsState2d {
    pub bodies: Arena<RigidBody2d>,
    pub colliders: Arena<Collider2d>,
    pub joints: Arena<Joint2d>,
    pub body_colliders: BTreeMap<BodyHandle, Vec<ColliderHandle>>,
    /// Persistent contact manifolds keyed by ordered collider pair.
    pub(super) manifolds: BTreeMap<ManifoldKey, ContactManifold>,
    /// Previous frame's manifold keys for collision event generation.
    pub(super) contact_tracker: crate::contact_tracker::ContactTracker,
    /// Union-find structure for simulation islands.
    pub(super) island_manager: IslandManager,
}

impl PhysicsState2d {
    pub fn new() -> Self {
        Self {
            bodies: Arena::new(),
            colliders: Arena::new(),
            joints: Arena::new(),
            body_colliders: BTreeMap::new(),
            manifolds: BTreeMap::new(),
            contact_tracker: crate::contact_tracker::ContactTracker::default(),
            island_manager: IslandManager::new(0),
        }
    }

    pub fn add_body(&mut self, desc: &BodyDesc) -> BodyHandle {
        // Insert with a placeholder handle; we'll patch it once the arena assigns the slot.
        let ah = self
            .bodies
            .insert(RigidBody2d::from_desc(BodyHandle(0), desc));
        let handle = body_from(ah);
        if let Some(rb) = self.bodies.get_mut(ah) {
            rb.handle = handle;
        }
        self.body_colliders.insert(handle, Vec::new());
        handle
    }

    pub fn add_collider(&mut self, body: BodyHandle, desc: &ColliderDesc) -> ColliderHandle {
        // Insert with placeholder handle, patch after arena assigns slot.
        let collider = Collider2d::from_desc(ColliderHandle(0), body, desc);

        // Add this collider's mass and inertia (about the body origin) to the
        // body — the same sum `recompute_mass` would produce.
        let (m, i_c, cen) = collider.mass_properties();
        let d = [collider.offset[0] + cen[0], collider.offset[1] + cen[1]];
        if let Some(rb) = self.bodies.get_mut(body_ah(body)) {
            rb.mass += m;
            rb.inertia += i_c + m * (d[0] * d[0] + d[1] * d[1]);
            rb.refresh_inverse_mass();
        }

        let ah = self.colliders.insert(collider);
        let handle = coll_from(ah);
        if let Some(c) = self.colliders.get_mut(ah) {
            c.handle = handle;
        }
        self.body_colliders.entry(body).or_default().push(handle);
        handle
    }

    /// Recompute a body's mass and rotational inertia from its colliders.
    ///
    /// The body origin is treated as the centre of mass; each collider's
    /// inertia is shifted to it with the parallel-axis theorem. Only dynamic
    /// bodies get a non-zero inverse mass.
    pub(super) fn recompute_mass(&mut self, body: BodyHandle) {
        let mut mass = 0.0_f64;
        let mut inertia = 0.0_f64;
        if let Some(handles) = self.body_colliders.get(&body) {
            for ch in handles {
                if let Some(c) = self.colliders.get(coll_ah(*ch)) {
                    let (m, i_c, cen) = c.mass_properties();
                    let d = [c.offset[0] + cen[0], c.offset[1] + cen[1]];
                    mass += m;
                    inertia += i_c + m * (d[0] * d[0] + d[1] * d[1]);
                }
            }
        }
        let Some(rb) = self.bodies.get_mut(body_ah(body)) else {
            return;
        };
        rb.mass = mass;
        rb.inertia = inertia;
        rb.refresh_inverse_mass();
    }

    pub fn add_joint(&mut self, desc: &JointDesc) -> JointHandle {
        let joint = self.joint_from_desc(desc);
        joint_from(self.joints.insert(joint))
    }

    fn joint_from_desc(&self, desc: &JointDesc) -> Joint2d {
        let rot = |h: BodyHandle| self.bodies.get(body_ah(h)).map_or(0.0, |b| b.rotation);
        Joint2d {
            body_a: desc.body_a,
            body_b: desc.body_b,
            joint_type: desc.joint_type.clone(),
            local_anchor_a: desc.local_anchor_a,
            local_anchor_b: desc.local_anchor_b,
            motor: desc.motor.clone(),
            damping: desc.damping,
            break_force: desc.break_force,
            reference_angle: rot(desc.body_b) - rot(desc.body_a),
        }
    }

    pub fn apply_force(&mut self, body: BodyHandle, force: &Force) {
        if let Some(rb) = self.bodies.get_mut(body_ah(body)) {
            // Wake the body
            rb.is_sleeping = false;
            rb.sleep_timer = 0.0;
            rb.force_accumulator[0] += force.vector[0];
            rb.force_accumulator[1] += force.vector[1];
            if let Some(point) = force.point {
                rb.torque_accumulator += point[0] * force.vector[1] - point[1] * force.vector[0];
            }
        }
    }

    pub fn apply_impulse(&mut self, body: BodyHandle, impulse: &Impulse) {
        if let Some(rb) = self.bodies.get_mut(body_ah(body))
            && rb.is_dynamic()
            && rb.inv_mass > 0.0
        {
            // Wake the body
            rb.is_sleeping = false;
            rb.sleep_timer = 0.0;
            rb.linear_velocity[0] += impulse.vector[0] * rb.inv_mass;
            rb.linear_velocity[1] += impulse.vector[1] * rb.inv_mass;
            if let Some(point) = impulse.point {
                let angular_impulse = point[0] * impulse.vector[1] - point[1] * impulse.vector[0];
                rb.angular_velocity += angular_impulse * rb.inv_inertia;
            }
        }
    }

    pub fn apply_torque(&mut self, body: BodyHandle, torque: &Torque) {
        if let Some(rb) = self.bodies.get_mut(body_ah(body)) {
            // Wake the body
            rb.is_sleeping = false;
            rb.sleep_timer = 0.0;
            rb.torque_accumulator += torque.value;
        }
    }

    /// Wake a body and everything touching or jointed to it — used before
    /// removals so bodies resting on the removed object don't hang in the air.
    fn wake_neighbours(&mut self, body: BodyHandle) {
        let mut wake: Vec<BodyHandle> = vec![body];
        for m in self.manifolds.values() {
            if m.body_a == body {
                wake.push(m.body_b);
            } else if m.body_b == body {
                wake.push(m.body_a);
            }
        }
        for (_, j) in self.joints.iter() {
            if j.body_a == body {
                wake.push(j.body_b);
            } else if j.body_b == body {
                wake.push(j.body_a);
            }
        }
        for h in wake {
            if let Some(rb) = self.bodies.get_mut(body_ah(h)) {
                rb.is_sleeping = false;
                rb.sleep_timer = 0.0;
            }
        }
    }

    pub fn remove_body(&mut self, handle: BodyHandle) {
        self.wake_neighbours(handle);
        self.bodies.remove(body_ah(handle));
        if let Some(collider_handles) = self.body_colliders.remove(&handle) {
            for ch in &collider_handles {
                self.colliders.remove(coll_ah(*ch));
            }
            // Clean stale manifolds referencing removed colliders
            self.manifolds
                .retain(|(a, b), _| !collider_handles.contains(a) && !collider_handles.contains(b));
        }
        self.joints
            .retain(|_, j| j.body_a != handle && j.body_b != handle);
    }

    /// Remove a single collider and recompute the parent body's mass properties.
    pub fn remove_collider(&mut self, handle: ColliderHandle) -> Result<(), ImpetusError> {
        let collider = self
            .colliders
            .remove(coll_ah(handle))
            .ok_or_else(|| ImpetusError::ColliderNotFound(format!("{:?}", handle)))?;
        let body = collider.body;
        self.wake_neighbours(body);

        // Remove from parent body's collider list
        if let Some(list) = self.body_colliders.get_mut(&body) {
            list.retain(|ch| *ch != handle);
        }

        // Clean stale manifolds
        self.manifolds
            .retain(|(a, b), _| *a != handle && *b != handle);

        // Recompute mass/inertia from remaining colliders
        self.recompute_mass(body);

        Ok(())
    }

    /// Remove a joint by handle.
    pub fn remove_joint(&mut self, handle: JointHandle) -> Result<(), ImpetusError> {
        let joint = self
            .joints
            .remove(joint_ah(handle))
            .ok_or_else(|| ImpetusError::JointNotFound(format!("{:?}", handle)))?;
        for h in [joint.body_a, joint.body_b] {
            if let Some(rb) = self.bodies.get_mut(body_ah(h)) {
                rb.is_sleeping = false;
                rb.sleep_timer = 0.0;
            }
        }
        Ok(())
    }

    /// Insert a body at a specific handle (for snapshot restore).
    #[cfg(feature = "serialize")]
    pub fn add_body_at(&mut self, handle: BodyHandle, desc: &BodyDesc) {
        let mut rb = RigidBody2d::from_desc(handle, desc);
        rb.handle = handle;
        self.bodies.insert_at(body_ah(handle), rb);
        self.body_colliders.insert(handle, Vec::new());
    }

    /// Insert a collider at a specific handle (for snapshot restore).
    #[cfg(feature = "serialize")]
    pub fn add_collider_at(
        &mut self,
        handle: ColliderHandle,
        body: BodyHandle,
        desc: &ColliderDesc,
    ) {
        let collider = Collider2d::from_desc(handle, body, desc);
        if self.colliders.insert_at(coll_ah(handle), collider) {
            self.body_colliders.entry(body).or_default().push(handle);
            self.recompute_mass(body);
        }
    }

    /// Insert a joint at a specific handle (for snapshot restore).
    #[cfg(feature = "serialize")]
    pub fn add_joint_at(&mut self, handle: JointHandle, desc: &JointDesc) {
        let joint = self.joint_from_desc(desc);
        self.joints.insert_at(joint_ah(handle), joint);
    }

    pub fn body_count(&self) -> usize {
        self.bodies.len()
    }

    pub fn get_body_state(&self, handle: BodyHandle) -> Result<BodyState, ImpetusError> {
        let rb = self
            .bodies
            .get(body_ah(handle))
            .ok_or_else(|| ImpetusError::BodyNotFound(format!("{:?}", handle)))?;
        Ok(BodyState {
            handle: rb.handle,
            body_type: rb.body_type,
            position: [rb.position[0], rb.position[1], 0.0],
            rotation: rb.rotation,
            linear_velocity: [rb.linear_velocity[0], rb.linear_velocity[1], 0.0],
            angular_velocity: rb.angular_velocity,
            is_sleeping: rb.is_sleeping,
        })
    }

    pub fn set_body_state(
        &mut self,
        handle: BodyHandle,
        state: &BodyState,
    ) -> Result<(), ImpetusError> {
        let rb = self
            .bodies
            .get_mut(body_ah(handle))
            .ok_or_else(|| ImpetusError::BodyNotFound(format!("{:?}", handle)))?;
        rb.position = [state.position[0], state.position[1]];
        rb.rotation = state.rotation;
        rb.linear_velocity = [state.linear_velocity[0], state.linear_velocity[1]];
        rb.angular_velocity = state.angular_velocity;
        rb.is_sleeping = false; // wake on teleport
        rb.sleep_timer = 0.0;
        Ok(())
    }

    pub fn set_body_type(
        &mut self,
        handle: BodyHandle,
        body_type: BodyType,
    ) -> Result<(), ImpetusError> {
        let rb = self
            .bodies
            .get_mut(body_ah(handle))
            .ok_or_else(|| ImpetusError::BodyNotFound(format!("{:?}", handle)))?;
        rb.body_type = body_type;
        rb.is_sleeping = false;
        rb.sleep_timer = 0.0;
        if matches!(body_type, BodyType::Static | BodyType::Kinematic) {
            rb.linear_velocity = [0.0, 0.0];
            rb.angular_velocity = 0.0;
        }
        // Mass properties depend on the body type (only dynamic bodies have
        // finite mass); a body created static gets its mass here.
        self.recompute_mass(handle);
        Ok(())
    }
}
