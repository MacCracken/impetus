//! Step, broadphase, contact solver, position solver, event generation, sleep.

use std::collections::{BTreeMap, BTreeSet};

use hisab::DVec3;

use crate::collider::ColliderHandle;
use crate::event::CollisionEvent;
use crate::spatial_hash::SpatialHashGrid;

use super::narrowphase::*;
use super::state::PhysicsState3d;
use super::types::*;
use super::{body_ah, coll_ah};

impl PhysicsState3d {
    // -----------------------------------------------------------------------
    // Simulation step
    // -----------------------------------------------------------------------

    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        gravity: [f64; 3],
        dt: f64,
        velocity_iterations: u32,
        position_iterations: u32,
        slop: f64,
        correction: f64,
        max_velocity: f64,
    ) -> Vec<CollisionEvent> {
        let g = DVec3::from_array(gravity);
        // 1. Integrate velocities
        for rb in self.bodies.values_mut() {
            rb.integrate_velocities(g, dt, max_velocity);
        }

        // 2-3. Broadphase + narrowphase
        let broad_pairs = self.broadphase();
        let contacts = self.narrowphase(&broad_pairs);

        // 4. Wake sleeping bodies on contact with non-sleeping moving bodies
        for contact in &contacts {
            let a_sleeping = self
                .bodies
                .get(body_ah(contact.body_a))
                .is_some_and(|b| b.is_sleeping);
            let b_sleeping = self
                .bodies
                .get(body_ah(contact.body_b))
                .is_some_and(|b| b.is_sleeping);
            let a_moving = self.bodies.get(body_ah(contact.body_a)).is_some_and(|b| {
                !b.is_sleeping
                    && b.is_dynamic()
                    && (b.linear_velocity.length() > SLEEP_VELOCITY_THRESHOLD_3D
                        || b.angular_velocity.length() > SLEEP_VELOCITY_THRESHOLD_3D)
            });
            let b_moving = self.bodies.get(body_ah(contact.body_b)).is_some_and(|b| {
                !b.is_sleeping
                    && b.is_dynamic()
                    && (b.linear_velocity.length() > SLEEP_VELOCITY_THRESHOLD_3D
                        || b.angular_velocity.length() > SLEEP_VELOCITY_THRESHOLD_3D)
            });
            if a_sleeping
                && b_moving
                && let Some(ba) = self.bodies.get_mut(body_ah(contact.body_a))
            {
                ba.is_sleeping = false;
                ba.sleep_timer = 0.0;
            }
            if b_sleeping
                && a_moving
                && let Some(bb) = self.bodies.get_mut(body_ah(contact.body_b))
            {
                bb.is_sleeping = false;
                bb.sleep_timer = 0.0;
            }
        }

        // 5. Solve velocity constraints
        self.solve_contacts(&contacts, velocity_iterations);

        // 6. Solve joint constraints
        self.solve_joints(dt, velocity_iterations);

        // 7. Positional correction
        self.solve_positions(&contacts, position_iterations, slop, correction);

        // 8. Integrate positions
        for rb in self.bodies.values_mut() {
            rb.integrate_positions(dt);
        }

        // 9. Sleep check: put nearly-stationary dynamic bodies to sleep
        for rb in self.bodies.values_mut() {
            if !rb.is_dynamic() || rb.inv_mass == 0.0 {
                continue;
            }
            let lin_speed = rb.linear_velocity.length();
            let ang_speed = rb.angular_velocity.length();
            if lin_speed < SLEEP_VELOCITY_THRESHOLD_3D && ang_speed < SLEEP_VELOCITY_THRESHOLD_3D {
                rb.sleep_timer += dt;
                if rb.sleep_timer >= SLEEP_TIME_THRESHOLD_3D {
                    rb.is_sleeping = true;
                }
            } else {
                rb.sleep_timer = 0.0;
                rb.is_sleeping = false;
            }
        }

        // 10. Clear forces
        for rb in self.bodies.values_mut() {
            rb.clear_forces();
        }

        // 11. Generate collision events
        self.generate_events(&contacts)
    }

    // -----------------------------------------------------------------------
    // Broadphase
    // -----------------------------------------------------------------------

    fn broadphase(&self) -> Vec<(ColliderHandle, ColliderHandle)> {
        let collider_aabbs: Vec<(ColliderHandle, Aabb3d)> = self
            .colliders
            .values()
            .filter_map(|c| {
                let rb = self.bodies.get(body_ah(c.body))?;
                Some((c.handle, c.world_aabb(rb.position, rb.rotation)))
            })
            .collect();

        let cell_size = SpatialHashGrid::<ColliderHandle>::auto_cell_size(
            collider_aabbs.iter().map(|(_, aabb)| {
                let size = aabb.max - aabb.min;
                size.x.max(size.y).max(size.z)
            }),
            collider_aabbs.len(),
        );
        let mut grid = SpatialHashGrid::new(cell_size);
        for (handle, aabb) in &collider_aabbs {
            grid.insert_3d(*handle, aabb.min.to_array(), aabb.max.to_array());
        }

        let candidates = grid.query_pairs();
        let aabb_map: BTreeMap<ColliderHandle, Aabb3d> = collider_aabbs.into_iter().collect();

        let mut pairs = Vec::with_capacity(candidates.len());
        for (ha, hb) in candidates {
            let ca = match self.colliders.get(coll_ah(ha)) {
                Some(c) => c,
                None => continue,
            };
            let cb = match self.colliders.get(coll_ah(hb)) {
                Some(c) => c,
                None => continue,
            };
            if ca.body == cb.body {
                continue;
            }
            if let (Some(ba), Some(bb)) = (
                self.bodies.get(body_ah(ca.body)),
                self.bodies.get(body_ah(cb.body)),
            ) && ba.is_static()
                && bb.is_static()
            {
                continue;
            }
            if ca.is_sensor && cb.is_sensor {
                continue;
            }
            // Collision layer filtering
            if (ca.collision_layer & cb.collision_mask) == 0
                && (cb.collision_layer & ca.collision_mask) == 0
            {
                continue;
            }
            if let (Some(aabb_a), Some(aabb_b)) = (aabb_map.get(&ha), aabb_map.get(&hb))
                && aabb_a.overlaps(aabb_b)
            {
                pairs.push((ha, hb));
            }
        }
        pairs
    }

    // -----------------------------------------------------------------------
    // Narrowphase
    // -----------------------------------------------------------------------

    fn narrowphase(&self, broad_pairs: &[(ColliderHandle, ColliderHandle)]) -> Vec<Contact3d> {
        let mut contacts = Vec::new();

        for (ha, hb) in broad_pairs {
            let ca = match self.colliders.get(coll_ah(*ha)) {
                Some(c) => c,
                None => continue,
            };
            let cb = match self.colliders.get(coll_ah(*hb)) {
                Some(c) => c,
                None => continue,
            };
            let ba = match self.bodies.get(body_ah(ca.body)) {
                Some(b) => b,
                None => continue,
            };
            let bb = match self.bodies.get(body_ah(cb.body)) {
                Some(b) => b,
                None => continue,
            };

            let pos_a = ba.position + ba.rotation * ca.offset;
            let pos_b = bb.position + bb.rotation * cb.offset;

            if let Some((normal, depth, point)) =
                generate_contact_3d(&ca.shape, pos_a, ba.rotation, &cb.shape, pos_b, bb.rotation)
            {
                contacts.push(Contact3d {
                    collider_a: *ha,
                    collider_b: *hb,
                    body_a: ca.body,
                    body_b: cb.body,
                    normal,
                    depth,
                    point,
                });
            }
        }
        contacts
    }

    // -----------------------------------------------------------------------
    // Contact solver
    // -----------------------------------------------------------------------

    fn solve_contacts(&mut self, contacts: &[Contact3d], iterations: u32) {
        use crate::material::CombineRule;

        const RESTITUTION_VELOCITY_THRESHOLD: f64 = 1.0;

        struct ContactMat {
            restitution: f64,
            friction: f64,
            rolling_friction: f64,
            is_sensor: bool,
        }

        fn combine_property(a: f64, b: f64, rule_a: CombineRule, rule_b: CombineRule) -> f64 {
            let rule = rule_a.max(rule_b);
            rule.combine(a, b)
        }

        let materials: Vec<ContactMat> = contacts
            .iter()
            .map(|c| {
                let (r, f, rf, s) = match (
                    self.colliders.get(coll_ah(c.collider_a)),
                    self.colliders.get(coll_ah(c.collider_b)),
                ) {
                    (Some(a), Some(b)) => (
                        combine_property(
                            a.material.restitution,
                            b.material.restitution,
                            a.material.restitution_combine,
                            b.material.restitution_combine,
                        ),
                        combine_property(
                            a.material.friction,
                            b.material.friction,
                            a.material.friction_combine,
                            b.material.friction_combine,
                        ),
                        (a.material.rolling_friction + b.material.rolling_friction) * 0.5,
                        a.is_sensor || b.is_sensor,
                    ),
                    _ => (0.0, 0.0, 0.0, false),
                };
                ContactMat {
                    restitution: r,
                    friction: f,
                    rolling_friction: rf,
                    is_sensor: s,
                }
            })
            .collect();

        for _ in 0..iterations {
            for (ci, contact) in contacts.iter().enumerate() {
                if materials[ci].is_sensor {
                    continue;
                }

                let (inv_mass_a, inv_inertia_a, vel_a, angvel_a, pos_a) = {
                    let ba = match self.bodies.get(body_ah(contact.body_a)) {
                        Some(b) => b,
                        None => continue,
                    };
                    (
                        ba.inv_mass,
                        ba.inv_inertia,
                        ba.linear_velocity,
                        ba.angular_velocity,
                        ba.position,
                    )
                };
                let (inv_mass_b, inv_inertia_b, vel_b, angvel_b, pos_b) = {
                    let bb = match self.bodies.get(body_ah(contact.body_b)) {
                        Some(b) => b,
                        None => continue,
                    };
                    (
                        bb.inv_mass,
                        bb.inv_inertia,
                        bb.linear_velocity,
                        bb.angular_velocity,
                        bb.position,
                    )
                };

                if inv_mass_a == 0.0 && inv_mass_b == 0.0 {
                    continue;
                }

                let n = contact.normal;
                let cp = contact.point;
                let ra = cp - pos_a;
                let rb = cp - pos_b;

                let vel_a_at_cp = vel_a + angvel_a.cross(ra);
                let vel_b_at_cp = vel_b + angvel_b.cross(rb);
                let rel_vel = vel_b_at_cp - vel_a_at_cp;
                let vel_along_normal = rel_vel.dot(n);

                if vel_along_normal > 0.0 {
                    continue;
                }

                let ra_cross_n = ra.cross(n);
                let rb_cross_n = rb.cross(n);
                let ang_eff_a = ra_cross_n.dot(ra_cross_n * inv_inertia_a);
                let ang_eff_b = rb_cross_n.dot(rb_cross_n * inv_inertia_b);
                let inv_mass_sum = inv_mass_a + inv_mass_b + ang_eff_a + ang_eff_b;

                let restitution = if vel_along_normal.abs() < RESTITUTION_VELOCITY_THRESHOLD {
                    0.0
                } else {
                    materials[ci].restitution
                };
                let j = -(1.0 + restitution) * vel_along_normal / inv_mass_sum;
                let impulse_n = n * j;

                if let Some(ba) = self.bodies.get_mut(body_ah(contact.body_a))
                    && ba.is_dynamic()
                {
                    ba.linear_velocity -= impulse_n * ba.inv_mass;
                    let ang_imp = ra.cross(impulse_n);
                    ba.angular_velocity -= ang_imp * ba.inv_inertia;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(contact.body_b))
                    && bb.is_dynamic()
                {
                    bb.linear_velocity += impulse_n * bb.inv_mass;
                    let ang_imp = rb.cross(impulse_n);
                    bb.angular_velocity += ang_imp * bb.inv_inertia;
                }

                // Friction
                let friction = materials[ci].friction;
                if friction > 0.0 {
                    let tangent_vel = rel_vel - n * vel_along_normal;
                    let tangent_speed = tangent_vel.length();
                    if tangent_speed > EPSILON {
                        let tangent = tangent_vel / tangent_speed;
                        let jt = (-tangent_speed / inv_mass_sum)
                            .clamp(-j.abs() * friction, j.abs() * friction);
                        let impulse_t = tangent * jt;

                        if let Some(ba) = self.bodies.get_mut(body_ah(contact.body_a))
                            && ba.is_dynamic()
                        {
                            ba.linear_velocity -= impulse_t * ba.inv_mass;
                            let ang_t = ra.cross(impulse_t);
                            ba.angular_velocity -= ang_t * ba.inv_inertia;
                        }
                        if let Some(bb) = self.bodies.get_mut(body_ah(contact.body_b))
                            && bb.is_dynamic()
                        {
                            bb.linear_velocity += impulse_t * bb.inv_mass;
                            let ang_t = rb.cross(impulse_t);
                            bb.angular_velocity += ang_t * bb.inv_inertia;
                        }
                    }
                }

                // Rolling friction (3D) — apply torque opposing angular velocity
                let rolling_friction = materials[ci].rolling_friction;
                if rolling_friction > 0.0 {
                    let normal_force = j.abs();
                    let roll_torque = rolling_friction * normal_force;

                    if let Some(ba) = self.bodies.get_mut(body_ah(contact.body_a))
                        && ba.is_dynamic()
                    {
                        let angvel_len = ba.angular_velocity.length();
                        if angvel_len > EPSILON {
                            let dir = ba.angular_velocity / angvel_len;
                            let reduction = (roll_torque * ba.inv_inertia.x).min(angvel_len);
                            ba.angular_velocity -= dir * reduction;
                        }
                    }
                    if let Some(bb) = self.bodies.get_mut(body_ah(contact.body_b))
                        && bb.is_dynamic()
                    {
                        let angvel_len = bb.angular_velocity.length();
                        if angvel_len > EPSILON {
                            let dir = bb.angular_velocity / angvel_len;
                            let reduction = (roll_torque * bb.inv_inertia.x).min(angvel_len);
                            bb.angular_velocity -= dir * reduction;
                        }
                    }
                }
            }
        }
    }

    fn solve_positions(
        &mut self,
        contacts: &[Contact3d],
        iterations: u32,
        slop: f64,
        percent: f64,
    ) {
        for _ in 0..iterations {
            for contact in contacts {
                let is_sensor = match (
                    self.colliders.get(coll_ah(contact.collider_a)),
                    self.colliders.get(coll_ah(contact.collider_b)),
                ) {
                    (Some(a), Some(b)) => a.is_sensor || b.is_sensor,
                    _ => false,
                };
                if is_sensor {
                    continue;
                }

                let inv_mass_a = self
                    .bodies
                    .get(body_ah(contact.body_a))
                    .map(|b| b.inv_mass)
                    .unwrap_or(0.0);
                let inv_mass_b = self
                    .bodies
                    .get(body_ah(contact.body_b))
                    .map(|b| b.inv_mass)
                    .unwrap_or(0.0);
                let inv_mass_sum = inv_mass_a + inv_mass_b;
                if inv_mass_sum == 0.0 {
                    continue;
                }

                let correction_mag = (contact.depth - slop).max(0.0) / inv_mass_sum * percent;
                let correction = contact.normal * correction_mag;

                if let Some(ba) = self.bodies.get_mut(body_ah(contact.body_a))
                    && ba.is_dynamic()
                {
                    ba.position -= correction * ba.inv_mass;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(contact.body_b))
                    && bb.is_dynamic()
                {
                    bb.position += correction * bb.inv_mass;
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Events
    // -----------------------------------------------------------------------

    fn generate_events(&mut self, contacts: &[Contact3d]) -> Vec<CollisionEvent> {
        let mut events = Vec::new();
        let current_pairs: BTreeSet<(ColliderHandle, ColliderHandle)> = contacts
            .iter()
            .map(|c| {
                if c.collider_a.0 < c.collider_b.0 {
                    (c.collider_a, c.collider_b)
                } else {
                    (c.collider_b, c.collider_a)
                }
            })
            .collect();

        for pair in &current_pairs {
            if !self.prev_collision_pairs.contains(pair) {
                events.push(CollisionEvent::Started {
                    collider_a: pair.0,
                    collider_b: pair.1,
                });
            } else {
                events.push(CollisionEvent::Ongoing {
                    collider_a: pair.0,
                    collider_b: pair.1,
                });
            }
        }
        for pair in &self.prev_collision_pairs {
            if !current_pairs.contains(pair) {
                events.push(CollisionEvent::Stopped {
                    collider_a: pair.0,
                    collider_b: pair.1,
                });
            }
        }
        self.prev_collision_pairs = current_pairs;
        events
    }
}
