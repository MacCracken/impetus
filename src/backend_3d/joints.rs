//! Joint constraint solvers for the 3D physics backend.

use hisab::DVec3;

use crate::arena::ArenaHandle;
use crate::body::BodyHandle;
use crate::joint::JointType;

use super::types::{Joint3d, EPSILON, EPSILON_SQ};
use super::state::PhysicsState3d;
use super::body_ah;

impl PhysicsState3d {
    // -----------------------------------------------------------------------
    // Joint constraint solver
    // -----------------------------------------------------------------------

    pub(super) fn solve_joints(&mut self, dt: f64, iterations: u32) {
        let joints: Vec<(ArenaHandle, Joint3d)> = self.joints.iter()
            .map(|(ah, j)| (ah, j.clone()))
            .collect();

        let mut constraint_forces: Vec<f64> = vec![0.0; joints.len()];

        for _ in 0..iterations {
            for (ji, (_ah, joint)) in joints.iter().enumerate() {
                let force = match &joint.joint_type {
                    JointType::Fixed => self.solve_fixed_joint_3d(joint),
                    JointType::Distance { length } => {
                        self.solve_distance_joint_3d(joint, *length)
                    }
                    JointType::Spring {
                        rest_length,
                        stiffness,
                        damping,
                    } => {
                        self.solve_spring_joint_3d(joint, *rest_length, *stiffness, *damping, dt)
                    }
                    JointType::Wheel { axis, stiffness, damping } => {
                        self.solve_wheel_joint_3d(joint, *axis, *stiffness, *damping, dt)
                    }
                    JointType::Rope { max_length } => {
                        self.solve_rope_joint_3d(joint, *max_length)
                    }
                    JointType::Mouse { target, stiffness, damping, max_force } => {
                        self.solve_mouse_joint_3d(joint, *target, *stiffness, *damping, *max_force, dt)
                    }
                    _ => 0.0, // Revolute/Prismatic: 3D versions need axis definitions, skip for now
                };
                constraint_forces[ji] = constraint_forces[ji].max(force);
                // Apply joint damping for non-Spring joints (Spring has its own damping).
                if joint.damping > 0.0 && !matches!(joint.joint_type, JointType::Spring { .. }) {
                    self.apply_joint_damping_3d(joint, dt);
                }
                // Motors: applied for Revolute/Prismatic when 3D axis support is added
                let _ = &joint.motor;
            }
        }

        // Joint breaking
        let mut broken: Vec<ArenaHandle> = Vec::new();
        for (ji, (ah, joint)) in joints.iter().enumerate() {
            if let Some(bf) = joint.break_force
                && constraint_forces[ji] > bf
            {
                broken.push(*ah);
            }
        }
        for ah in broken {
            self.joints.remove(ah);
        }
    }

    /// Apply velocity damping proportional to relative velocity at anchor points.
    fn apply_joint_damping_3d(&mut self, joint: &Joint3d, dt: f64) {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);

        let (vel_a, angvel_a, pos_a, inv_mass_a) = match self.bodies.get(body_ah(joint.body_a)) {
            Some(b) if b.is_dynamic() => (b.linear_velocity, b.angular_velocity, b.position, b.inv_mass),
            Some(b) => (b.linear_velocity, b.angular_velocity, b.position, 0.0),
            None => return,
        };
        let (vel_b, angvel_b, pos_b, inv_mass_b) = match self.bodies.get(body_ah(joint.body_b)) {
            Some(b) if b.is_dynamic() => (b.linear_velocity, b.angular_velocity, b.position, b.inv_mass),
            Some(b) => (b.linear_velocity, b.angular_velocity, b.position, 0.0),
            None => return,
        };

        let inv_mass_sum = inv_mass_a + inv_mass_b;
        if inv_mass_sum == 0.0 {
            return;
        }

        let ra = anchor_a - pos_a;
        let rb = anchor_b - pos_b;
        let va = vel_a + angvel_a.cross(ra);
        let vb = vel_b + angvel_b.cross(rb);
        let rel_vel = vb - va;

        if rel_vel.dot(rel_vel) < EPSILON_SQ {
            return;
        }

        let impulse = rel_vel * (-joint.damping * dt);

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity -= impulse * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity += impulse * bb.inv_mass;
        }
    }

    fn world_anchor_3d(&self, body: BodyHandle, local: DVec3) -> DVec3 {
        let rb = match self.bodies.get(body_ah(body)) {
            Some(b) => b,
            None => return local,
        };
        rb.position + rb.rotation * local
    }

    fn solve_fixed_joint_3d(&mut self, joint: &Joint3d) -> f64 {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);
        let diff = anchor_b - anchor_a;
        let force = diff.length();

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position += diff * 0.5;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position -= diff * 0.5;
        }
        force
    }

    fn solve_distance_joint_3d(&mut self, joint: &Joint3d, length: f64) -> f64 {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);
        let diff = anchor_b - anchor_a;
        let dist_sq = diff.dot(diff);

        if dist_sq < EPSILON_SQ {
            return 0.0;
        }
        let dist = dist_sq.sqrt();

        let n = diff / dist;
        let correction = (dist - length) * 0.5;

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position += n * correction;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position -= n * correction;
        }
        (dist - length).abs()
    }

    fn solve_spring_joint_3d(
        &mut self,
        joint: &Joint3d,
        rest_length: f64,
        stiffness: f64,
        damping: f64,
        dt: f64,
    ) -> f64 {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);
        let diff = anchor_b - anchor_a;
        let dist_sq = diff.dot(diff);

        if dist_sq < EPSILON_SQ {
            return 0.0;
        }
        let dist = dist_sq.sqrt();

        let n = diff / dist;
        let spring_force = stiffness * (dist - rest_length);

        let vel_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .map(|b| b.linear_velocity)
            .unwrap_or(DVec3::ZERO);
        let vel_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .map(|b| b.linear_velocity)
            .unwrap_or(DVec3::ZERO);
        let rel_vel = vel_b - vel_a;
        let damping_force = damping * rel_vel.dot(n);

        let total_force = spring_force + damping_force;
        let force = n * (total_force * dt);

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity += force * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity -= force * bb.inv_mass;
        }
        total_force.abs()
    }

    /// Wheel joint (3D) — suspension along axis + free rotation.
    fn solve_wheel_joint_3d(
        &mut self,
        joint: &Joint3d,
        axis: [f64; 2],
        stiffness: f64,
        damping: f64,
        dt: f64,
    ) -> f64 {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);
        let diff = anchor_b - anchor_a;

        let ax_2d_len = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
        if ax_2d_len < EPSILON {
            return 0.0;
        }
        let ax = DVec3::new(axis[0] / ax_2d_len, axis[1] / ax_2d_len, 0.0);
        let perp = DVec3::new(-ax.y, ax.x, 0.0);

        // Constrain perpendicular
        let perp_error = diff.dot(perp);
        let correction = perp_error * 0.5;
        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position += perp * correction;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position -= perp * correction;
        }

        // Spring along axis
        let along = diff.dot(ax);
        let spring_force = stiffness * along;

        let vel_a = self.bodies.get(body_ah(joint.body_a)).map(|b| b.linear_velocity).unwrap_or(DVec3::ZERO);
        let vel_b = self.bodies.get(body_ah(joint.body_b)).map(|b| b.linear_velocity).unwrap_or(DVec3::ZERO);
        let rel_vel_along = (vel_b - vel_a).dot(ax);
        let damping_force = damping * rel_vel_along;

        let total_force = spring_force + damping_force;
        let force = ax * (total_force * dt);

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity += force * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity -= force * bb.inv_mass;
        }
        total_force.abs()
    }

    /// Rope joint (3D) — inequality constraint.
    fn solve_rope_joint_3d(&mut self, joint: &Joint3d, max_length: f64) -> f64 {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor_3d(joint.body_b, joint.local_anchor_b);
        let diff = anchor_b - anchor_a;
        let dist_sq = diff.dot(diff);

        if dist_sq < EPSILON_SQ {
            return 0.0;
        }
        let dist = dist_sq.sqrt();

        if dist <= max_length {
            return 0.0;
        }

        let n = diff / dist;
        let correction = (dist - max_length) * 0.5;

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position += n * correction;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position -= n * correction;
        }
        dist - max_length
    }

    /// Mouse joint (3D) — drags body_a toward a world-space target.
    fn solve_mouse_joint_3d(
        &mut self,
        joint: &Joint3d,
        target: [f64; 3],
        stiffness: f64,
        damping: f64,
        max_force: f64,
        dt: f64,
    ) -> f64 {
        let anchor_a = self.world_anchor_3d(joint.body_a, joint.local_anchor_a);
        let target_v = DVec3::from_array(target);
        let diff = target_v - anchor_a;

        if diff.dot(diff) < EPSILON_SQ {
            return 0.0;
        }

        let spring_force = diff * stiffness;
        let vel_a = self.bodies.get(body_ah(joint.body_a)).map(|b| b.linear_velocity).unwrap_or(DVec3::ZERO);
        let damping_force = vel_a * (-damping);
        let mut total = (spring_force + damping_force) * dt;

        let force_mag = total.length();
        let max_impulse = max_force * dt;
        if force_mag > max_impulse && force_mag > EPSILON {
            total *= max_impulse / force_mag;
        }

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity += total * ba.inv_mass;
        }

        let applied = total.length();
        if dt > EPSILON { applied / dt } else { 0.0 }
    }
}
