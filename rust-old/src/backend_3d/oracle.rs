//! 3D backend side of the oracle hooks (see `crate::oracle`).

use hisab::{DQuat, DVec3};

use super::narrowphase;
use super::state::PhysicsState3d;
use crate::collider::ColliderShape;
use crate::oracle::{Backend3dDump, Body3dDump, ColliderDump, Joint3dDump};

impl PhysicsState3d {
    /// Every stored field of the 3D backend, in arena slot order.
    pub(crate) fn oracle_dump(&self) -> Backend3dDump {
        let bodies = self
            .bodies
            .values()
            .map(|b| Body3dDump {
                handle: b.handle.0,
                body_type: b.body_type,
                position: b.position.to_array(),
                rotation: b.rotation.to_array(),
                linear_velocity: b.linear_velocity.to_array(),
                angular_velocity: b.angular_velocity.to_array(),
                linear_damping: b.linear_damping,
                angular_damping: b.angular_damping,
                fixed_rotation: b.fixed_rotation,
                gravity_scale: b.gravity_scale,
                force_accumulator: b.force_accumulator.to_array(),
                torque_accumulator: b.torque_accumulator.to_array(),
                mass: b.mass,
                inv_mass: b.inv_mass,
                inertia: b.inertia.to_cols_array_2d(),
                inv_inertia: b.inv_inertia.to_cols_array_2d(),
                inv_inertia_world: b.inv_inertia_world.to_cols_array_2d(),
                pseudo_velocity: b.pseudo_velocity.to_array(),
                pseudo_angular_velocity: b.pseudo_angular_velocity.to_array(),
                is_sleeping: b.is_sleeping,
                sleep_timer: b.sleep_timer,
            })
            .collect();
        let colliders = self
            .colliders
            .values()
            .map(|c| ColliderDump {
                handle: c.handle.0,
                body: c.body.0,
                shape: c.shape.clone(),
                offset: c.offset.to_array(),
                material: c.material.clone(),
                is_sensor: c.is_sensor,
                mass: c.mass,
                collision_layer: c.collision_layer,
                collision_mask: c.collision_mask,
            })
            .collect();
        let joints = self
            .joints
            .iter()
            .map(|(h, j)| Joint3dDump {
                handle: h.0,
                body_a: j.body_a.0,
                body_b: j.body_b.0,
                joint_type: j.joint_type.clone(),
                local_anchor_a: j.local_anchor_a.to_array(),
                local_anchor_b: j.local_anchor_b.to_array(),
                reference_rotation: j.reference_rotation.to_array(),
                motor: j.motor.clone(),
                damping: j.damping,
                break_force: j.break_force,
            })
            .collect();
        Backend3dDump {
            body_arena: self.bodies.oracle_dump(),
            bodies,
            collider_arena: self.colliders.oracle_dump(),
            colliders,
            joint_arena: self.joints.oracle_dump(),
            joints,
            body_colliders: self
                .body_colliders
                .iter()
                .map(|(b, cs)| (b.0, cs.iter().map(|c| c.0).collect()))
                .collect(),
            tracker: self.contact_tracker.oracle_dump(),
        }
    }
}

/// The contact the 3D narrowphase generates for one shape pair, as
/// `(normal from A to B, depth, world point)`. Rotations are `[x, y, z, w]`.
#[must_use]
pub fn contact_3d(
    shape_a: &ColliderShape,
    pos_a: [f64; 3],
    rot_a: [f64; 4],
    shape_b: &ColliderShape,
    pos_b: [f64; 3],
    rot_b: [f64; 4],
) -> Option<([f64; 3], f64, [f64; 3])> {
    narrowphase::generate_contact_3d(
        shape_a,
        DVec3::from_array(pos_a),
        DQuat::from_array(rot_a),
        shape_b,
        DVec3::from_array(pos_b),
        DQuat::from_array(rot_b),
    )
    .map(|(n, d, p)| (n.to_array(), d, p.to_array()))
}
