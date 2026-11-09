//! 2D backend side of the oracle hooks (see `crate::oracle`).

use super::narrowphase;
use super::state::PhysicsState2d;
use crate::collider::ColliderShape;
use crate::oracle::{
    Backend2dDump, Body2dDump, ColliderDump, Joint2dDump, Manifold2dDump, ManifoldPoint2dDump,
};

impl PhysicsState2d {
    /// Every stored field of the 2D backend, in arena slot and key order.
    pub(crate) fn oracle_dump(&self) -> Backend2dDump {
        let bodies = self
            .bodies
            .values()
            .map(|b| Body2dDump {
                handle: b.handle.0,
                body_type: b.body_type,
                position: b.position,
                rotation: b.rotation,
                linear_velocity: b.linear_velocity,
                angular_velocity: b.angular_velocity,
                linear_damping: b.linear_damping,
                angular_damping: b.angular_damping,
                fixed_rotation: b.fixed_rotation,
                gravity_scale: b.gravity_scale,
                force_accumulator: b.force_accumulator,
                torque_accumulator: b.torque_accumulator,
                mass: b.mass,
                inv_mass: b.inv_mass,
                inertia: b.inertia,
                inv_inertia: b.inv_inertia,
                pseudo_velocity: b.pseudo_velocity,
                pseudo_angular_velocity: b.pseudo_angular_velocity,
                prev_position: b.prev_position,
                prev_rotation: b.prev_rotation,
                is_sleeping: b.is_sleeping,
                sleep_timer: b.sleep_timer,
                island_id: b.island_id,
            })
            .collect();
        let colliders = self
            .colliders
            .values()
            .map(|c| ColliderDump {
                handle: c.handle.0,
                body: c.body.0,
                shape: c.shape.clone(),
                offset: [c.offset[0], c.offset[1], 0.0],
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
            .map(|(h, j)| Joint2dDump {
                handle: h.0,
                body_a: j.body_a.0,
                body_b: j.body_b.0,
                joint_type: j.joint_type.clone(),
                local_anchor_a: j.local_anchor_a,
                local_anchor_b: j.local_anchor_b,
                motor: j.motor.clone(),
                damping: j.damping,
                break_force: j.break_force,
                reference_angle: j.reference_angle,
            })
            .collect();
        let manifolds = self
            .manifolds
            .values()
            .map(|m| Manifold2dDump {
                collider_a: m.collider_a.0,
                collider_b: m.collider_b.0,
                body_a: m.body_a.0,
                body_b: m.body_b.0,
                normal: m.normal,
                points: m
                    .points
                    .iter()
                    .map(|p| ManifoldPoint2dDump {
                        local_a: p.local_a,
                        local_b: p.local_b,
                        normal_impulse: p.normal_impulse,
                        tangent_impulse: p.tangent_impulse,
                        depth: p.depth,
                    })
                    .collect(),
                rolling_impulse: m.rolling_impulse,
            })
            .collect();
        Backend2dDump {
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
            manifolds,
            tracker: self.contact_tracker.oracle_dump(),
        }
    }
}

/// Every contact the 2D narrowphase generates for one shape pair, as
/// `(normal from A to B, depth, world point)`, in generation order.
#[must_use]
pub fn contacts_2d(
    shape_a: &ColliderShape,
    pos_a: [f64; 2],
    rot_a: f64,
    shape_b: &ColliderShape,
    pos_b: [f64; 2],
    rot_b: f64,
) -> Vec<([f64; 2], f64, [f64; 2])> {
    narrowphase::generate_contacts_multi(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b)
}

/// The 2D overlap test the queries use.
#[must_use]
pub fn shapes_touch_2d(
    shape_a: &ColliderShape,
    pos_a: [f64; 2],
    rot_a: f64,
    shape_b: &ColliderShape,
    pos_b: [f64; 2],
    rot_b: f64,
) -> bool {
    narrowphase::shapes_touch(shape_a, pos_a, rot_a, shape_b, pos_b, rot_b)
}
