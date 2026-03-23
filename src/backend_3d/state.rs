//! PhysicsState3d struct definition and add/remove/get/set methods.

use std::collections::{BTreeMap, BTreeSet};

use hisab::{DQuat, DVec3};

use crate::arena::Arena;
use crate::body::{BodyDesc, BodyHandle, BodyState, BodyType};
use crate::collider::{ColliderDesc, ColliderHandle};
use crate::force::{Force, Impulse, Torque};
use crate::joint::{JointDesc, JointHandle};
use crate::ImpetusError;

use super::types::*;
use super::{body_ah, body_from, coll_ah, coll_from, joint_ah, joint_from};

// ---------------------------------------------------------------------------
// Physics state
// ---------------------------------------------------------------------------

pub(crate) struct PhysicsState3d {
    pub bodies: Arena<RigidBody3d>,
    pub colliders: Arena<Collider3d>,
    pub joints: Arena<Joint3d>,
    pub body_colliders: BTreeMap<BodyHandle, Vec<ColliderHandle>>,
    pub(super) prev_collision_pairs: BTreeSet<(ColliderHandle, ColliderHandle)>,
}

impl PhysicsState3d {
    pub fn new() -> Self {
        Self {
            bodies: Arena::new(),
            colliders: Arena::new(),
            joints: Arena::new(),
            body_colliders: BTreeMap::new(),
            prev_collision_pairs: BTreeSet::new(),
        }
    }

    pub fn add_body(&mut self, desc: &BodyDesc) -> BodyHandle {
        let ah = self.bodies.insert(RigidBody3d::from_desc(BodyHandle(0), desc));
        let handle = body_from(ah);
        self.bodies.get_mut(ah).expect("just-inserted body").handle = handle;
        self.body_colliders.insert(handle, Vec::new());
        handle
    }

    pub fn add_collider(&mut self, body: BodyHandle, desc: &ColliderDesc) -> ColliderHandle {
        let collider = Collider3d::from_desc(ColliderHandle(0), body, desc);

        if let Some(rb) = self.bodies.get_mut(body_ah(body))
            && rb.is_dynamic()
        {
            let c_mass = collider.compute_mass();
            let c_inertia = collider.compute_inertia(c_mass);
            rb.mass += c_mass;
            rb.inertia += c_inertia;
            rb.inv_mass = 1.0 / rb.mass;
            rb.inv_inertia = if rb.fixed_rotation {
                DVec3::ZERO
            } else {
                DVec3::new(
                    1.0 / rb.inertia.x,
                    1.0 / rb.inertia.y,
                    1.0 / rb.inertia.z,
                )
            };
        }

        let ah = self.colliders.insert(collider);
        let handle = coll_from(ah);
        self.colliders.get_mut(ah).expect("just-inserted collider").handle = handle;
        self.body_colliders.entry(body).or_default().push(handle);
        handle
    }

    pub fn add_joint(&mut self, desc: &JointDesc) -> JointHandle {
        let ah = self.joints.insert(Joint3d {
            body_a: desc.body_a,
            body_b: desc.body_b,
            joint_type: desc.joint_type.clone(),
            local_anchor_a: DVec3::new(desc.local_anchor_a[0], desc.local_anchor_a[1], 0.0),
            local_anchor_b: DVec3::new(desc.local_anchor_b[0], desc.local_anchor_b[1], 0.0),
            motor: desc.motor.clone(),
            damping: desc.damping,
            break_force: desc.break_force,
        });
        joint_from(ah)
    }

    pub fn apply_force(&mut self, body: BodyHandle, force: &Force) {
        if let Some(rb) = self.bodies.get_mut(body_ah(body)) {
            // Wake the body
            rb.is_sleeping = false;
            rb.sleep_timer = 0.0;
            let fv = DVec3::from_array(force.vector);
            rb.force_accumulator += fv;
            if let Some(point) = force.point {
                let p = DVec3::from_array(point);
                rb.torque_accumulator += p.cross(fv);
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
            let iv = DVec3::from_array(impulse.vector);
            rb.linear_velocity += iv * rb.inv_mass;
            if let Some(point) = impulse.point {
                let p = DVec3::from_array(point);
                let ang = p.cross(iv);
                rb.angular_velocity += ang * rb.inv_inertia;
            }
        }
    }

    pub fn apply_torque(&mut self, body: BodyHandle, torque: &Torque) {
        if let Some(rb) = self.bodies.get_mut(body_ah(body)) {
            // Wake the body
            rb.is_sleeping = false;
            rb.sleep_timer = 0.0;
            rb.torque_accumulator.z += torque.value;
        }
    }

    pub fn remove_body(&mut self, handle: BodyHandle) {
        self.bodies.remove(body_ah(handle));
        if let Some(collider_handles) = self.body_colliders.remove(&handle) {
            for ch in &collider_handles {
                self.colliders.remove(coll_ah(*ch));
            }
            self.prev_collision_pairs
                .retain(|(a, b)| !collider_handles.contains(a) && !collider_handles.contains(b));
        }
        self.joints
            .retain(|_, j| j.body_a != handle && j.body_b != handle);
    }

    /// Remove a single collider and recompute the parent body's mass properties.
    pub fn remove_collider(&mut self, handle: ColliderHandle) -> Result<(), ImpetusError> {
        let collider = self.colliders.remove(coll_ah(handle))
            .ok_or_else(|| ImpetusError::ColliderNotFound(format!("{:?}", handle)))?;
        let body = collider.body;

        // Remove from parent body's collider list
        if let Some(list) = self.body_colliders.get_mut(&body) {
            list.retain(|ch| *ch != handle);
        }

        // Clean stale collision pairs
        self.prev_collision_pairs
            .retain(|(a, b)| *a != handle && *b != handle);

        // Recompute mass/inertia from remaining colliders
        if let Some(rb) = self.bodies.get_mut(body_ah(body))
            && rb.is_dynamic()
        {
            let mut mass = 0.0_f64;
            let mut inertia = DVec3::ZERO;
            if let Some(collider_handles) = self.body_colliders.get(&body) {
                for ch in collider_handles {
                    if let Some(c) = self.colliders.get(coll_ah(*ch)) {
                        let cm = c.compute_mass();
                        mass += cm;
                        inertia += c.compute_inertia(cm);
                    }
                }
            }
            rb.mass = mass;
            rb.inertia = inertia;
            if mass > 0.0 {
                rb.inv_mass = 1.0 / mass;
                rb.inv_inertia = if rb.fixed_rotation {
                    DVec3::ZERO
                } else {
                    DVec3::new(1.0 / inertia.x, 1.0 / inertia.y, 1.0 / inertia.z)
                };
            } else {
                rb.inv_mass = 0.0;
                rb.inv_inertia = DVec3::ZERO;
            }
        }

        Ok(())
    }

    /// Remove a joint by handle.
    pub fn remove_joint(&mut self, handle: JointHandle) -> Result<(), ImpetusError> {
        self.joints.remove(joint_ah(handle))
            .ok_or_else(|| ImpetusError::JointNotFound(format!("{:?}", handle)))?;
        Ok(())
    }

    /// Insert a body at a specific handle (for snapshot restore).
    #[cfg(feature = "serialize")]
    pub fn add_body_at(&mut self, handle: BodyHandle, desc: &BodyDesc) {
        let mut rb = RigidBody3d::from_desc(handle, desc);
        rb.handle = handle;
        self.bodies.insert_at(body_ah(handle), rb);
        self.body_colliders.insert(handle, Vec::new());
    }

    /// Insert a collider at a specific handle (for snapshot restore).
    #[cfg(feature = "serialize")]
    pub fn add_collider_at(&mut self, handle: ColliderHandle, body: BodyHandle, desc: &ColliderDesc) {
        let collider = Collider3d::from_desc(handle, body, desc);
        if let Some(rb) = self.bodies.get_mut(body_ah(body))
            && rb.is_dynamic()
        {
            let c_mass = collider.compute_mass();
            let c_inertia = collider.compute_inertia(c_mass);
            rb.mass += c_mass;
            rb.inertia += c_inertia;
            rb.inv_mass = 1.0 / rb.mass;
            rb.inv_inertia = if rb.fixed_rotation {
                DVec3::ZERO
            } else {
                DVec3::new(1.0 / rb.inertia.x, 1.0 / rb.inertia.y, 1.0 / rb.inertia.z)
            };
        }
        self.colliders.insert_at(coll_ah(handle), collider);
        self.body_colliders.entry(body).or_default().push(handle);
    }

    /// Insert a joint at a specific handle (for snapshot restore).
    #[cfg(feature = "serialize")]
    pub fn add_joint_at(&mut self, handle: JointHandle, desc: &JointDesc) {
        self.joints.insert_at(joint_ah(handle), Joint3d {
            body_a: desc.body_a,
            body_b: desc.body_b,
            joint_type: desc.joint_type.clone(),
            local_anchor_a: DVec3::new(desc.local_anchor_a[0], desc.local_anchor_a[1], 0.0),
            local_anchor_b: DVec3::new(desc.local_anchor_b[0], desc.local_anchor_b[1], 0.0),
            motor: desc.motor.clone(),
            damping: desc.damping,
            break_force: desc.break_force,
        });
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
            position: rb.position.to_array(),
            rotation: rb.rotation.z.atan2(rb.rotation.w) * 2.0, // extract z-rotation
            linear_velocity: rb.linear_velocity.to_array(),
            angular_velocity: rb.angular_velocity.z,
            is_sleeping: rb.is_sleeping,
        })
    }

    pub fn set_body_state(&mut self, handle: BodyHandle, state: &BodyState) -> Result<(), ImpetusError> {
        let rb = self.bodies.get_mut(body_ah(handle))
            .ok_or_else(|| ImpetusError::BodyNotFound(format!("{:?}", handle)))?;
        rb.position = DVec3::from_array(state.position);
        rb.rotation = DQuat::from_rotation_z(state.rotation);
        rb.linear_velocity = DVec3::from_array(state.linear_velocity);
        rb.angular_velocity = DVec3::new(0.0, 0.0, state.angular_velocity);
        rb.is_sleeping = false;
        rb.sleep_timer = 0.0;
        Ok(())
    }

    pub fn set_body_type(&mut self, handle: BodyHandle, body_type: BodyType) -> Result<(), ImpetusError> {
        let rb = self.bodies.get_mut(body_ah(handle))
            .ok_or_else(|| ImpetusError::BodyNotFound(format!("{:?}", handle)))?;
        rb.body_type = body_type;
        match body_type {
            BodyType::Static | BodyType::Kinematic => {
                rb.inv_mass = 0.0;
                rb.inv_inertia = DVec3::ZERO;
                rb.linear_velocity = DVec3::ZERO;
                rb.angular_velocity = DVec3::ZERO;
            }
            BodyType::Dynamic => {
                if rb.mass > 0.0 {
                    rb.inv_mass = 1.0 / rb.mass;
                    rb.inv_inertia = if rb.fixed_rotation {
                        DVec3::ZERO
                    } else {
                        DVec3::new(
                            1.0 / rb.inertia.x,
                            1.0 / rb.inertia.y,
                            1.0 / rb.inertia.z,
                        )
                    };
                }
            }
        }
        Ok(())
    }
}
