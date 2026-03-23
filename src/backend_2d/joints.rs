//! Joint constraint solvers for the 2D physics backend.

use crate::arena::ArenaHandle;
use crate::body::BodyHandle;
use crate::joint::{JointMotor, JointType};

use super::body_ah;
use super::state::PhysicsState2d;
use super::types::{EPSILON, EPSILON_SQ, Joint2d};

impl PhysicsState2d {
    // -----------------------------------------------------------------------
    // Joint constraint solver
    // -----------------------------------------------------------------------

    pub(super) fn solve_joints(&mut self, dt: f64, iterations: u32) {
        // Collect (ArenaHandle, Joint2d) pairs so we can track handles for breaking.
        let joints: Vec<(ArenaHandle, Joint2d)> =
            self.joints.iter().map(|(ah, j)| (ah, j.clone())).collect();

        // Track constraint forces for joint breaking.
        // We accumulate the maximum constraint force per joint across iterations.
        let mut constraint_forces: Vec<f64> = vec![0.0; joints.len()];

        for _ in 0..iterations {
            for (ji, (_ah, joint)) in joints.iter().enumerate() {
                let force = match &joint.joint_type {
                    JointType::Fixed => self.solve_fixed_joint(joint),
                    JointType::Distance { length } => self.solve_distance_joint(joint, *length),
                    JointType::Spring {
                        rest_length,
                        stiffness,
                        damping,
                    } => self.solve_spring_joint(joint, *rest_length, *stiffness, *damping, dt),
                    JointType::Revolute { limits, .. } => {
                        let f = self.solve_revolute_joint(joint, limits.as_ref());
                        if let Some(motor) = &joint.motor {
                            self.solve_revolute_motor(joint, motor, dt);
                        }
                        f
                    }
                    JointType::Prismatic { axis, limits } => {
                        let f = self.solve_prismatic_joint(joint, *axis, limits.as_ref());
                        if let Some(motor) = &joint.motor {
                            self.solve_prismatic_motor(joint, *axis, motor, dt);
                        }
                        f
                    }
                    JointType::Wheel {
                        axis,
                        stiffness,
                        damping,
                    } => self.solve_wheel_joint(joint, *axis, *stiffness, *damping, dt),
                    JointType::Rope { max_length } => self.solve_rope_joint(joint, *max_length),
                    JointType::Mouse {
                        target,
                        stiffness,
                        damping,
                        max_force,
                    } => {
                        self.solve_mouse_joint(joint, *target, *stiffness, *damping, *max_force, dt)
                    }
                };
                constraint_forces[ji] = constraint_forces[ji].max(force);
                // Apply joint damping for non-Spring joints (Spring has its own damping).
                if joint.damping > 0.0 && !matches!(joint.joint_type, JointType::Spring { .. }) {
                    self.apply_joint_damping(joint, dt);
                }
            }
        }

        // Joint breaking — remove joints whose constraint force exceeded break_force.
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
    fn apply_joint_damping(&mut self, joint: &Joint2d, dt: f64) {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);

        let (vel_a, angvel_a, pos_a, inv_mass_a) = match self.bodies.get(body_ah(joint.body_a)) {
            Some(b) if b.is_dynamic() => (
                b.linear_velocity,
                b.angular_velocity,
                b.position,
                b.inv_mass,
            ),
            Some(b) => (b.linear_velocity, b.angular_velocity, b.position, 0.0),
            None => return,
        };
        let (vel_b, angvel_b, pos_b, inv_mass_b) = match self.bodies.get(body_ah(joint.body_b)) {
            Some(b) if b.is_dynamic() => (
                b.linear_velocity,
                b.angular_velocity,
                b.position,
                b.inv_mass,
            ),
            Some(b) => (b.linear_velocity, b.angular_velocity, b.position, 0.0),
            None => return,
        };

        let inv_mass_sum = inv_mass_a + inv_mass_b;
        if inv_mass_sum == 0.0 {
            return;
        }

        // Velocity at anchor point = linear_vel + angular_vel x r
        let ra = [anchor_a[0] - pos_a[0], anchor_a[1] - pos_a[1]];
        let rb = [anchor_b[0] - pos_b[0], anchor_b[1] - pos_b[1]];
        let va = [vel_a[0] - angvel_a * ra[1], vel_a[1] + angvel_a * ra[0]];
        let vb = [vel_b[0] - angvel_b * rb[1], vel_b[1] + angvel_b * rb[0]];
        let rel_vel = [vb[0] - va[0], vb[1] - va[1]];

        let rel_speed_sq = rel_vel[0] * rel_vel[0] + rel_vel[1] * rel_vel[1];
        if rel_speed_sq < EPSILON_SQ {
            return;
        }

        // Damping impulse: -damping * relative_velocity * dt, distributed by inverse mass
        let impulse = [
            -joint.damping * rel_vel[0] * dt,
            -joint.damping * rel_vel[1] * dt,
        ];

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity[0] -= impulse[0] * ba.inv_mass;
            ba.linear_velocity[1] -= impulse[1] * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity[0] += impulse[0] * bb.inv_mass;
            bb.linear_velocity[1] += impulse[1] * bb.inv_mass;
        }
    }

    /// Apply a revolute motor: drives relative angular velocity toward `target_velocity`,
    /// clamped by `max_force` (torque).
    fn solve_revolute_motor(&mut self, joint: &Joint2d, motor: &JointMotor, dt: f64) {
        let angvel_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .map(|b| b.angular_velocity)
            .unwrap_or(0.0);
        let angvel_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .map(|b| b.angular_velocity)
            .unwrap_or(0.0);
        let inv_inertia_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .filter(|b| b.is_dynamic())
            .map(|b| b.inv_inertia)
            .unwrap_or(0.0);
        let inv_inertia_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .filter(|b| b.is_dynamic())
            .map(|b| b.inv_inertia)
            .unwrap_or(0.0);

        let inv_inertia_sum = inv_inertia_a + inv_inertia_b;
        if inv_inertia_sum == 0.0 {
            return;
        }

        let rel_angvel = angvel_b - angvel_a;
        let error = motor.target_velocity - rel_angvel;
        // Impulse = error / inv_inertia_sum, clamped by max_force * dt
        let max_impulse = motor.max_force * dt;
        let impulse = (error / inv_inertia_sum).clamp(-max_impulse, max_impulse);

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.angular_velocity -= impulse * ba.inv_inertia;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.angular_velocity += impulse * bb.inv_inertia;
        }
    }

    /// Apply a prismatic motor: drives relative linear velocity along `axis` toward
    /// `target_velocity`, clamped by `max_force`.
    fn solve_prismatic_motor(
        &mut self,
        joint: &Joint2d,
        axis: [f64; 2],
        motor: &JointMotor,
        dt: f64,
    ) {
        let axis_len = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
        if axis_len < EPSILON {
            return;
        }
        let ax = [axis[0] / axis_len, axis[1] / axis_len];

        let vel_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let vel_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let inv_mass_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .filter(|b| b.is_dynamic())
            .map(|b| b.inv_mass)
            .unwrap_or(0.0);
        let inv_mass_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .filter(|b| b.is_dynamic())
            .map(|b| b.inv_mass)
            .unwrap_or(0.0);

        let inv_mass_sum = inv_mass_a + inv_mass_b;
        if inv_mass_sum == 0.0 {
            return;
        }

        let rel_vel = [vel_b[0] - vel_a[0], vel_b[1] - vel_a[1]];
        let rel_speed_along_axis = rel_vel[0] * ax[0] + rel_vel[1] * ax[1];
        let error = motor.target_velocity - rel_speed_along_axis;
        let max_impulse = motor.max_force * dt;
        let impulse = (error / inv_mass_sum).clamp(-max_impulse, max_impulse);

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity[0] -= impulse * ax[0] * ba.inv_mass;
            ba.linear_velocity[1] -= impulse * ax[1] * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity[0] += impulse * ax[0] * bb.inv_mass;
            bb.linear_velocity[1] += impulse * ax[1] * bb.inv_mass;
        }
    }

    pub(super) fn world_anchor(&self, body: BodyHandle, local: [f64; 2]) -> [f64; 2] {
        let rb = match self.bodies.get(body_ah(body)) {
            Some(b) => b,
            None => return local,
        };
        let (sin, cos) = rb.rotation.sin_cos();
        [
            rb.position[0] + cos * local[0] - sin * local[1],
            rb.position[1] + sin * local[0] + cos * local[1],
        ]
    }

    fn solve_fixed_joint(&mut self, joint: &Joint2d) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];
        let force = (diff[0] * diff[0] + diff[1] * diff[1]).sqrt();

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position[0] += diff[0] * 0.5;
            ba.position[1] += diff[1] * 0.5;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position[0] -= diff[0] * 0.5;
            bb.position[1] -= diff[1] * 0.5;
        }
        force
    }

    fn solve_distance_joint(&mut self, joint: &Joint2d, length: f64) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];
        let dist_sq = diff[0] * diff[0] + diff[1] * diff[1];

        if dist_sq < EPSILON_SQ {
            return 0.0;
        }
        let dist = dist_sq.sqrt();
        let force = (dist - length).abs();

        let n = [diff[0] / dist, diff[1] / dist];
        let correction = (dist - length) * 0.5;

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position[0] += n[0] * correction;
            ba.position[1] += n[1] * correction;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position[0] -= n[0] * correction;
            bb.position[1] -= n[1] * correction;
        }
        force
    }

    fn solve_spring_joint(
        &mut self,
        joint: &Joint2d,
        rest_length: f64,
        stiffness: f64,
        damping: f64,
        dt: f64,
    ) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];
        let dist_sq = diff[0] * diff[0] + diff[1] * diff[1];

        if dist_sq < EPSILON_SQ {
            return 0.0;
        }
        let dist = dist_sq.sqrt();

        let n = [diff[0] / dist, diff[1] / dist];
        let spring_force = stiffness * (dist - rest_length);

        let vel_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let vel_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let rel_vel = [vel_b[0] - vel_a[0], vel_b[1] - vel_a[1]];
        let damping_force = damping * (rel_vel[0] * n[0] + rel_vel[1] * n[1]);

        let total_force = spring_force + damping_force;
        let force = [total_force * n[0] * dt, total_force * n[1] * dt];

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity[0] += force[0] * ba.inv_mass;
            ba.linear_velocity[1] += force[1] * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity[0] -= force[0] * bb.inv_mass;
            bb.linear_velocity[1] -= force[1] * bb.inv_mass;
        }
        total_force.abs()
    }

    fn solve_revolute_joint(&mut self, joint: &Joint2d, limits: Option<&[f64; 2]>) -> f64 {
        let force = self.solve_fixed_joint(joint);

        if let Some([lo, hi]) = limits {
            let rot_a = self
                .bodies
                .get(body_ah(joint.body_a))
                .map(|b| b.rotation)
                .unwrap_or(0.0);
            let rot_b = self
                .bodies
                .get(body_ah(joint.body_b))
                .map(|b| b.rotation)
                .unwrap_or(0.0);
            let rel_angle = rot_b - rot_a;

            if rel_angle < *lo {
                let c = (lo - rel_angle) * 0.5;
                if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
                    && ba.is_dynamic()
                {
                    ba.rotation -= c;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
                    && bb.is_dynamic()
                {
                    bb.rotation += c;
                }
            } else if rel_angle > *hi {
                let c = (rel_angle - hi) * 0.5;
                if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
                    && ba.is_dynamic()
                {
                    ba.rotation += c;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
                    && bb.is_dynamic()
                {
                    bb.rotation -= c;
                }
            }
        }
        force
    }

    fn solve_prismatic_joint(
        &mut self,
        joint: &Joint2d,
        axis: [f64; 2],
        limits: Option<&[f64; 2]>,
    ) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];

        let axis_len = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
        if axis_len < EPSILON {
            return 0.0;
        }
        let ax = [axis[0] / axis_len, axis[1] / axis_len];
        let perp = [-ax[1], ax[0]];
        let perp_error = diff[0] * perp[0] + diff[1] * perp[1];
        let correction = perp_error * 0.5;

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position[0] += perp[0] * correction;
            ba.position[1] += perp[1] * correction;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position[0] -= perp[0] * correction;
            bb.position[1] -= perp[1] * correction;
        }

        if let Some([lo, hi]) = limits {
            let along = diff[0] * ax[0] + diff[1] * ax[1];
            if along < *lo {
                let c = (lo - along) * 0.5;
                if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
                    && ba.is_dynamic()
                {
                    ba.position[0] -= ax[0] * c;
                    ba.position[1] -= ax[1] * c;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
                    && bb.is_dynamic()
                {
                    bb.position[0] += ax[0] * c;
                    bb.position[1] += ax[1] * c;
                }
            } else if along > *hi {
                let c = (along - hi) * 0.5;
                if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
                    && ba.is_dynamic()
                {
                    ba.position[0] += ax[0] * c;
                    ba.position[1] += ax[1] * c;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
                    && bb.is_dynamic()
                {
                    bb.position[0] -= ax[0] * c;
                    bb.position[1] -= ax[1] * c;
                }
            }
        }
        perp_error.abs()
    }

    /// Wheel joint — constrains perpendicular to axis (like prismatic), applies
    /// spring force along axis, allows free rotation.
    fn solve_wheel_joint(
        &mut self,
        joint: &Joint2d,
        axis: [f64; 2],
        stiffness: f64,
        damping: f64,
        dt: f64,
    ) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];

        let axis_len = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
        if axis_len < EPSILON {
            return 0.0;
        }
        let ax = [axis[0] / axis_len, axis[1] / axis_len];
        let perp = [-ax[1], ax[0]];

        // 1. Constrain perpendicular movement (like prismatic)
        let perp_error = diff[0] * perp[0] + diff[1] * perp[1];
        let correction = perp_error * 0.5;

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position[0] += perp[0] * correction;
            ba.position[1] += perp[1] * correction;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position[0] -= perp[0] * correction;
            bb.position[1] -= perp[1] * correction;
        }

        // 2. Spring force along the axis (suspension)
        let along = diff[0] * ax[0] + diff[1] * ax[1];
        let spring_force = stiffness * along;

        let vel_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let vel_b = self
            .bodies
            .get(body_ah(joint.body_b))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let rel_vel = [vel_b[0] - vel_a[0], vel_b[1] - vel_a[1]];
        let rel_vel_along = rel_vel[0] * ax[0] + rel_vel[1] * ax[1];
        let damping_force = damping * rel_vel_along;

        let total_force = spring_force + damping_force;
        let force = [total_force * ax[0] * dt, total_force * ax[1] * dt];

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity[0] += force[0] * ba.inv_mass;
            ba.linear_velocity[1] += force[1] * ba.inv_mass;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.linear_velocity[0] -= force[0] * bb.inv_mass;
            bb.linear_velocity[1] -= force[1] * bb.inv_mass;
        }

        // No angular constraint — free rotation
        total_force.abs()
    }

    /// Rope joint — inequality constraint, only prevents exceeding max distance.
    fn solve_rope_joint(&mut self, joint: &Joint2d, max_length: f64) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let anchor_b = self.world_anchor(joint.body_b, joint.local_anchor_b);
        let diff = [anchor_b[0] - anchor_a[0], anchor_b[1] - anchor_a[1]];
        let dist_sq = diff[0] * diff[0] + diff[1] * diff[1];

        if dist_sq < EPSILON_SQ {
            return 0.0;
        }
        let dist = dist_sq.sqrt();

        // Inequality: only act when distance exceeds max_length
        if dist <= max_length {
            return 0.0;
        }

        let n = [diff[0] / dist, diff[1] / dist];
        let correction = (dist - max_length) * 0.5;

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.position[0] += n[0] * correction;
            ba.position[1] += n[1] * correction;
        }
        if let Some(bb) = self.bodies.get_mut(body_ah(joint.body_b))
            && bb.is_dynamic()
        {
            bb.position[0] -= n[0] * correction;
            bb.position[1] -= n[1] * correction;
        }
        dist - max_length
    }

    /// Mouse joint — drags body_a toward a world-space target with clamped spring force.
    fn solve_mouse_joint(
        &mut self,
        joint: &Joint2d,
        target: [f64; 3],
        stiffness: f64,
        damping: f64,
        max_force: f64,
        dt: f64,
    ) -> f64 {
        let anchor_a = self.world_anchor(joint.body_a, joint.local_anchor_a);
        let target_2d = [target[0], target[1]];
        let diff = [target_2d[0] - anchor_a[0], target_2d[1] - anchor_a[1]];
        let dist_sq = diff[0] * diff[0] + diff[1] * diff[1];

        if dist_sq < EPSILON_SQ {
            return 0.0;
        }

        // Spring force toward target
        let spring_force = [stiffness * diff[0], stiffness * diff[1]];

        // Damping force
        let vel_a = self
            .bodies
            .get(body_ah(joint.body_a))
            .map(|b| b.linear_velocity)
            .unwrap_or([0.0, 0.0]);
        let damping_force = [-damping * vel_a[0], -damping * vel_a[1]];

        let mut total = [
            (spring_force[0] + damping_force[0]) * dt,
            (spring_force[1] + damping_force[1]) * dt,
        ];

        // Clamp by max_force
        let force_mag = (total[0] * total[0] + total[1] * total[1]).sqrt();
        let max_impulse = max_force * dt;
        if force_mag > max_impulse && force_mag > EPSILON {
            let scale = max_impulse / force_mag;
            total[0] *= scale;
            total[1] *= scale;
        }

        if let Some(ba) = self.bodies.get_mut(body_ah(joint.body_a))
            && ba.is_dynamic()
        {
            ba.linear_velocity[0] += total[0] * ba.inv_mass;
            ba.linear_velocity[1] += total[1] * ba.inv_mass;
        }

        let applied = (total[0] * total[0] + total[1] * total[1]).sqrt();
        if dt > EPSILON { applied / dt } else { 0.0 }
    }
}
