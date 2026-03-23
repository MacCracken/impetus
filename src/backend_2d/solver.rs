//! Step, broadphase, manifold update, warm start, contact solver, position solver, event generation.

use std::collections::{BTreeMap, BTreeSet};

use crate::collider::ColliderHandle;
use crate::event::CollisionEvent;
use crate::spatial_hash::SpatialHashGrid;

use super::types::*;
use super::state::PhysicsState2d;
use super::narrowphase::*;
use super::{body_ah, coll_ah};

impl PhysicsState2d {
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
        let gravity_2d = [gravity[0], gravity[1]];
        // 1. Integrate velocities
        for rb in self.bodies.values_mut() {
            rb.integrate_velocities(gravity_2d, dt, max_velocity);
        }

        // 2. Broadphase
        let broad_pairs = self.broadphase();

        // 3. Narrowphase — raw contacts
        let contacts = self.narrowphase(&broad_pairs);

        // 4. Update manifold cache (match contacts to existing manifolds)
        self.update_manifolds(&contacts);

        // 5. Wake sleeping bodies on contact with non-sleeping moving bodies
        for manifold in self.manifolds.values() {
            let a_sleeping = self
                .bodies
                .get(body_ah(manifold.body_a))
                .is_some_and(|b| b.is_sleeping);
            let b_sleeping = self
                .bodies
                .get(body_ah(manifold.body_b))
                .is_some_and(|b| b.is_sleeping);
            let a_moving = self.bodies.get(body_ah(manifold.body_a)).is_some_and(|b| {
                !b.is_sleeping
                    && b.is_dynamic()
                    && (b.linear_velocity[0].abs() > SLEEP_VELOCITY_THRESHOLD
                        || b.linear_velocity[1].abs() > SLEEP_VELOCITY_THRESHOLD
                        || b.angular_velocity.abs() > SLEEP_VELOCITY_THRESHOLD)
            });
            let b_moving = self.bodies.get(body_ah(manifold.body_b)).is_some_and(|b| {
                !b.is_sleeping
                    && b.is_dynamic()
                    && (b.linear_velocity[0].abs() > SLEEP_VELOCITY_THRESHOLD
                        || b.linear_velocity[1].abs() > SLEEP_VELOCITY_THRESHOLD
                        || b.angular_velocity.abs() > SLEEP_VELOCITY_THRESHOLD)
            });
            if a_sleeping
                && b_moving
                && let Some(ba) = self.bodies.get_mut(body_ah(manifold.body_a))
            {
                ba.is_sleeping = false;
                ba.sleep_timer = 0.0;
            }
            if b_sleeping
                && a_moving
                && let Some(bb) = self.bodies.get_mut(body_ah(manifold.body_b))
            {
                bb.is_sleeping = false;
                bb.sleep_timer = 0.0;
            }
        }

        // 6. Warm start — apply cached impulses from previous frame
        self.warm_start();

        // 7. Solve velocity constraints (using manifolds with accumulation)
        self.solve_contacts(velocity_iterations);

        // 8. Solve joint constraints
        self.solve_joints(dt, velocity_iterations);

        // 9. Positional correction
        self.solve_positions(position_iterations, slop, correction);

        // 10. Integrate positions
        for rb in self.bodies.values_mut() {
            rb.integrate_positions(dt);
        }

        // 11. Sleep check: put nearly-stationary dynamic bodies to sleep
        for rb in self.bodies.values_mut() {
            if !rb.is_dynamic() || rb.inv_mass == 0.0 {
                continue;
            }
            let lin_speed = (rb.linear_velocity[0] * rb.linear_velocity[0]
                + rb.linear_velocity[1] * rb.linear_velocity[1])
            .sqrt();
            let ang_speed = rb.angular_velocity.abs();
            if lin_speed < SLEEP_VELOCITY_THRESHOLD && ang_speed < SLEEP_VELOCITY_THRESHOLD {
                rb.sleep_timer += dt;
                if rb.sleep_timer >= SLEEP_TIME_THRESHOLD {
                    rb.is_sleeping = true;
                }
            } else {
                rb.sleep_timer = 0.0;
                rb.is_sleeping = false;
            }
        }

        // 12. Clear forces
        for rb in self.bodies.values_mut() {
            rb.clear_forces();
        }

        // 13. Generate collision events (from manifold keys)
        self.generate_events()
    }

    // -----------------------------------------------------------------------
    // Broadphase — spatial hash grid
    // -----------------------------------------------------------------------

    fn broadphase(&self) -> Vec<(ColliderHandle, ColliderHandle)> {
        // Compute AABBs for all colliders
        let collider_aabbs: Vec<(ColliderHandle, Aabb2d)> = self
            .colliders
            .values()
            .filter_map(|c| {
                let rb = self.bodies.get(body_ah(c.body))?;
                Some((c.handle, c.world_aabb(rb.position, rb.rotation)))
            })
            .collect();

        // Build spatial hash
        let cell_size = SpatialHashGrid::<ColliderHandle>::auto_cell_size(
            collider_aabbs.iter().map(|(_, aabb)| {
                let w = aabb.max[0] - aabb.min[0];
                let h = aabb.max[1] - aabb.min[1];
                w.max(h)
            }),
            collider_aabbs.len(),
        );
        let mut grid = SpatialHashGrid::new(cell_size);
        for (handle, aabb) in &collider_aabbs {
            grid.insert_2d(*handle, aabb.min, aabb.max);
        }

        // Collect candidate pairs from shared cells
        let candidates = grid.query_pairs();

        // Build AABB lookup for overlap verification
        let aabb_map: BTreeMap<ColliderHandle, Aabb2d> =
            collider_aabbs.into_iter().collect();

        // Filter candidates
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
            // Skip same-body
            if ca.body == cb.body {
                continue;
            }
            // Skip static-static
            if let (Some(ba), Some(bb)) = (self.bodies.get(body_ah(ca.body)), self.bodies.get(body_ah(cb.body)))
                && ba.is_static() && bb.is_static()
            {
                continue;
            }
            // Skip sensor-sensor
            if ca.is_sensor && cb.is_sensor {
                continue;
            }
            // Skip if collision layers don't match
            if (ca.collision_layer & cb.collision_mask) == 0
                && (cb.collision_layer & ca.collision_mask) == 0
            {
                continue;
            }
            // Verify AABB overlap (spatial hash cells are conservative)
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

    fn narrowphase(
        &self,
        broad_pairs: &[(ColliderHandle, ColliderHandle)],
    ) -> Vec<Contact> {
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

            let pos_a = world_pos(ba.position, ba.rotation, ca.offset);
            let pos_b = world_pos(bb.position, bb.rotation, cb.offset);

            if let Some((normal, depth, point)) =
                generate_contact(&ca.shape, pos_a, ba.rotation, &cb.shape, pos_b, bb.rotation)
            {
                contacts.push(Contact {
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
    // Manifold update — match new contacts to existing manifolds
    // -----------------------------------------------------------------------

    fn update_manifolds(&mut self, contacts: &[Contact]) {
        // Track which manifold keys appear in this frame's contacts
        let mut current_keys: BTreeSet<ManifoldKey> = BTreeSet::new();

        for contact in contacts {
            let key = ordered_manifold_key(contact.collider_a, contact.collider_b);
            current_keys.insert(key);

            // Transform contact point to body-local coordinates
            let (local_a, local_b) = {
                let pos_a = self.bodies.get(body_ah(contact.body_a))
                    .map(|b| (b.position, b.rotation))
                    .unwrap_or(([0.0, 0.0], 0.0));
                let pos_b = self.bodies.get(body_ah(contact.body_b))
                    .map(|b| (b.position, b.rotation))
                    .unwrap_or(([0.0, 0.0], 0.0));
                (
                    world_to_local(contact.point, pos_a.0, pos_a.1),
                    world_to_local(contact.point, pos_b.0, pos_b.1),
                )
            };

            if let Some(manifold) = self.manifolds.get_mut(&key) {
                // Existing manifold — update normal and match points
                manifold.normal = if key.0 == contact.collider_a {
                    contact.normal
                } else {
                    [-contact.normal[0], -contact.normal[1]]
                };
                manifold.body_a = if key.0 == contact.collider_a { contact.body_a } else { contact.body_b };
                manifold.body_b = if key.0 == contact.collider_a { contact.body_b } else { contact.body_a };

                let new_local_a = if key.0 == contact.collider_a { local_a } else { local_b };
                let new_local_b = if key.0 == contact.collider_a { local_b } else { local_a };

                // Try to match the new contact point to an existing manifold point
                let mut best_idx: Option<usize> = None;
                let mut best_dist_sq = MANIFOLD_MATCH_THRESHOLD * MANIFOLD_MATCH_THRESHOLD;
                for (i, mp) in manifold.points.iter().enumerate() {
                    let dx = mp.local_a[0] - new_local_a[0];
                    let dy = mp.local_a[1] - new_local_a[1];
                    let dist_sq = dx * dx + dy * dy;
                    if dist_sq < best_dist_sq {
                        best_dist_sq = dist_sq;
                        best_idx = Some(i);
                    }
                }

                if let Some(idx) = best_idx {
                    // Update the matched point, preserving accumulated impulses
                    manifold.points[idx].local_a = new_local_a;
                    manifold.points[idx].local_b = new_local_b;
                    manifold.points[idx].depth = contact.depth;
                } else {
                    // New point — zero impulses
                    manifold.points.clear(); // single-point manifold: replace
                    manifold.points.push(ManifoldPoint {
                        local_a: new_local_a,
                        local_b: new_local_b,
                        normal_impulse: 0.0,
                        tangent_impulse: 0.0,
                        depth: contact.depth,
                    });
                }
            } else {
                // New manifold
                let (ca, cb, ba, bb, normal, la, lb) = if key.0 == contact.collider_a {
                    (contact.collider_a, contact.collider_b, contact.body_a, contact.body_b, contact.normal, local_a, local_b)
                } else {
                    (contact.collider_b, contact.collider_a, contact.body_b, contact.body_a,
                     [-contact.normal[0], -contact.normal[1]], local_b, local_a)
                };
                self.manifolds.insert(key, ContactManifold {
                    collider_a: ca,
                    collider_b: cb,
                    body_a: ba,
                    body_b: bb,
                    normal,
                    points: vec![ManifoldPoint {
                        local_a: la,
                        local_b: lb,
                        normal_impulse: 0.0,
                        tangent_impulse: 0.0,
                        depth: contact.depth,
                    }],
                });
            }
        }

        // Remove manifolds for pairs no longer in contact
        self.manifolds.retain(|k, _| current_keys.contains(k));
    }

    // -----------------------------------------------------------------------
    // Warm starting — apply cached impulses from previous frame
    // -----------------------------------------------------------------------

    fn warm_start(&mut self) {
        // Collect warm-start data to avoid borrow conflicts
        struct WarmData {
            body_a: crate::body::BodyHandle,
            body_b: crate::body::BodyHandle,
            impulse_n: [f64; 2],
            impulse_t: [f64; 2],
            ra_cross_n: f64,
            rb_cross_n: f64,
            ra_cross_t: f64,
            rb_cross_t: f64,
            j_n: f64,
            jt: f64,
        }

        let warm_data: Vec<WarmData> = self.manifolds.values().flat_map(|manifold| {
            let pos_a = self.bodies.get(body_ah(manifold.body_a))
                .map(|b| b.position)
                .unwrap_or([0.0, 0.0]);
            let pos_b = self.bodies.get(body_ah(manifold.body_b))
                .map(|b| b.position)
                .unwrap_or([0.0, 0.0]);

            // Check if this is a sensor contact
            let is_sensor = match (
                self.colliders.get(coll_ah(manifold.collider_a)),
                self.colliders.get(coll_ah(manifold.collider_b)),
            ) {
                (Some(a), Some(b)) => a.is_sensor || b.is_sensor,
                _ => false,
            };
            if is_sensor {
                return Vec::new();
            }

            let n = manifold.normal;
            let tangent = [-n[1], n[0]];

            manifold.points.iter().filter_map(|mp| {
                let j_n = mp.normal_impulse * WARM_START_FACTOR;
                let jt = mp.tangent_impulse * WARM_START_FACTOR;
                if j_n.abs() < EPSILON && jt.abs() < EPSILON {
                    return None;
                }

                // Reconstruct world-space contact point from body A's local coords
                let rot_a = self.bodies.get(body_ah(manifold.body_a))
                    .map(|b| b.rotation).unwrap_or(0.0);
                let cp = local_to_world(mp.local_a, pos_a, rot_a);

                let ra = [cp[0] - pos_a[0], cp[1] - pos_a[1]];
                let rb = [cp[0] - pos_b[0], cp[1] - pos_b[1]];

                Some(WarmData {
                    body_a: manifold.body_a,
                    body_b: manifold.body_b,
                    impulse_n: [j_n * n[0], j_n * n[1]],
                    impulse_t: [jt * tangent[0], jt * tangent[1]],
                    ra_cross_n: ra[0] * n[1] - ra[1] * n[0],
                    rb_cross_n: rb[0] * n[1] - rb[1] * n[0],
                    ra_cross_t: ra[0] * tangent[1] - ra[1] * tangent[0],
                    rb_cross_t: rb[0] * tangent[1] - rb[1] * tangent[0],
                    j_n,
                    jt,
                })
            }).collect::<Vec<_>>()
        }).collect();

        for wd in &warm_data {
            let total_impulse = [
                wd.impulse_n[0] + wd.impulse_t[0],
                wd.impulse_n[1] + wd.impulse_t[1],
            ];
            if let Some(ba) = self.bodies.get_mut(body_ah(wd.body_a))
                && ba.is_dynamic()
            {
                ba.linear_velocity[0] -= total_impulse[0] * ba.inv_mass;
                ba.linear_velocity[1] -= total_impulse[1] * ba.inv_mass;
                ba.angular_velocity -= (wd.ra_cross_n * wd.j_n + wd.ra_cross_t * wd.jt) * ba.inv_inertia;
            }
            if let Some(bb) = self.bodies.get_mut(body_ah(wd.body_b))
                && bb.is_dynamic()
            {
                bb.linear_velocity[0] += total_impulse[0] * bb.inv_mass;
                bb.linear_velocity[1] += total_impulse[1] * bb.inv_mass;
                bb.angular_velocity += (wd.rb_cross_n * wd.j_n + wd.rb_cross_t * wd.jt) * bb.inv_inertia;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Contact constraint solver with accumulated impulses, friction, angular response
    // -----------------------------------------------------------------------

    fn solve_contacts(&mut self, iterations: u32) {
        use crate::material::CombineRule;

        /// Velocity threshold below which restitution is zeroed to prevent
        /// micro-bouncing of resting objects.
        const RESTITUTION_VELOCITY_THRESHOLD: f64 = 1.0;

        // Pre-extract material properties per manifold to avoid repeated lookups
        struct ManifoldMaterial {
            restitution: f64,
            friction: f64,
            rolling_friction: f64,
            is_sensor: bool,
        }

        /// Pick the higher-priority combine rule, then apply it.
        fn combine_property(a: f64, b: f64, rule_a: CombineRule, rule_b: CombineRule) -> f64 {
            let rule = rule_a.max(rule_b);
            rule.combine(a, b)
        }

        let keys: Vec<ManifoldKey> = self.manifolds.keys().copied().collect();
        let materials: Vec<ManifoldMaterial> = keys.iter().map(|key| {
            let manifold = &self.manifolds[key];
            let (rest, fric, roll_fric, sensor) = match (
                self.colliders.get(coll_ah(manifold.collider_a)),
                self.colliders.get(coll_ah(manifold.collider_b)),
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
            ManifoldMaterial {
                restitution: rest,
                friction: fric,
                rolling_friction: roll_fric,
                is_sensor: sensor,
            }
        }).collect();

        for _ in 0..iterations {
            for (ki, key) in keys.iter().enumerate() {
                if materials[ki].is_sensor {
                    continue;
                }

                let manifold = match self.manifolds.get(key) {
                    Some(m) => m,
                    None => continue,
                };

                let (inv_mass_a, inv_inertia_a, pos_a, rot_a) = {
                    let ba = match self.bodies.get(body_ah(manifold.body_a)) {
                        Some(b) => b,
                        None => continue,
                    };
                    (ba.inv_mass, ba.inv_inertia, ba.position, ba.rotation)
                };
                let (inv_mass_b, inv_inertia_b, pos_b) = {
                    let bb = match self.bodies.get(body_ah(manifold.body_b)) {
                        Some(b) => b,
                        None => continue,
                    };
                    (bb.inv_mass, bb.inv_inertia, bb.position)
                };

                if inv_mass_a == 0.0 && inv_mass_b == 0.0 {
                    continue;
                }

                let n = manifold.normal;
                let body_a_handle = manifold.body_a;
                let body_b_handle = manifold.body_b;
                let num_points = manifold.points.len();

                // Process each manifold point
                for pi in 0..num_points {
                    let mp = &self.manifolds[key].points[pi];

                    // Reconstruct world-space contact point from body A's local coords
                    let cp = local_to_world(mp.local_a, pos_a, rot_a);
                    let ra = [cp[0] - pos_a[0], cp[1] - pos_a[1]];
                    let rb = [cp[0] - pos_b[0], cp[1] - pos_b[1]];

                    // Re-read velocities (they change during iteration)
                    let (vel_a, angvel_a) = {
                        let ba = match self.bodies.get(body_ah(body_a_handle)) {
                            Some(b) => b,
                            None => continue,
                        };
                        (ba.linear_velocity, ba.angular_velocity)
                    };
                    let (vel_b, angvel_b) = {
                        let bb = match self.bodies.get(body_ah(body_b_handle)) {
                            Some(b) => b,
                            None => continue,
                        };
                        (bb.linear_velocity, bb.angular_velocity)
                    };

                    // Relative velocity at contact point (including angular)
                    let vel_a_at_cp = [
                        vel_a[0] - angvel_a * ra[1],
                        vel_a[1] + angvel_a * ra[0],
                    ];
                    let vel_b_at_cp = [
                        vel_b[0] - angvel_b * rb[1],
                        vel_b[1] + angvel_b * rb[0],
                    ];
                    let rel_vel = [vel_b_at_cp[0] - vel_a_at_cp[0], vel_b_at_cp[1] - vel_a_at_cp[1]];
                    let vel_along_normal = rel_vel[0] * n[0] + rel_vel[1] * n[1];

                    // Angular effective mass
                    let ra_cross_n = ra[0] * n[1] - ra[1] * n[0];
                    let rb_cross_n = rb[0] * n[1] - rb[1] * n[0];
                    let inv_mass_sum = inv_mass_a + inv_mass_b
                        + ra_cross_n * ra_cross_n * inv_inertia_a
                        + rb_cross_n * rb_cross_n * inv_inertia_b;

                    // Normal impulse with accumulation
                    // Suppress restitution at low velocities to prevent micro-bouncing.
                    let restitution = if vel_along_normal.abs() < RESTITUTION_VELOCITY_THRESHOLD {
                        0.0
                    } else {
                        materials[ki].restitution
                    };
                    let j_new = -(1.0 + restitution) * vel_along_normal / inv_mass_sum;
                    let j_old = self.manifolds[key].points[pi].normal_impulse;
                    let j_accumulated = (j_old + j_new).max(0.0);
                    let j_applied = j_accumulated - j_old;
                    self.manifolds.get_mut(key).unwrap().points[pi].normal_impulse = j_accumulated;

                    let impulse_n = [j_applied * n[0], j_applied * n[1]];

                    if let Some(ba) = self.bodies.get_mut(body_ah(body_a_handle))
                        && ba.is_dynamic()
                    {
                        ba.linear_velocity[0] -= impulse_n[0] * ba.inv_mass;
                        ba.linear_velocity[1] -= impulse_n[1] * ba.inv_mass;
                        ba.angular_velocity -= ra_cross_n * j_applied * ba.inv_inertia;
                    }
                    if let Some(bb) = self.bodies.get_mut(body_ah(body_b_handle))
                        && bb.is_dynamic()
                    {
                        bb.linear_velocity[0] += impulse_n[0] * bb.inv_mass;
                        bb.linear_velocity[1] += impulse_n[1] * bb.inv_mass;
                        bb.angular_velocity += rb_cross_n * j_applied * bb.inv_inertia;
                    }

                    // Friction impulse with accumulation
                    let friction = materials[ki].friction;
                    if friction > 0.0 {
                        // Re-read velocities after normal impulse application
                        let (vel_a, angvel_a) = {
                            let ba = match self.bodies.get(body_ah(body_a_handle)) {
                                Some(b) => b,
                                None => continue,
                            };
                            (ba.linear_velocity, ba.angular_velocity)
                        };
                        let (vel_b, angvel_b) = {
                            let bb = match self.bodies.get(body_ah(body_b_handle)) {
                                Some(b) => b,
                                None => continue,
                            };
                            (bb.linear_velocity, bb.angular_velocity)
                        };

                        let vel_a_at_cp = [
                            vel_a[0] - angvel_a * ra[1],
                            vel_a[1] + angvel_a * ra[0],
                        ];
                        let vel_b_at_cp = [
                            vel_b[0] - angvel_b * rb[1],
                            vel_b[1] + angvel_b * rb[0],
                        ];
                        let rel_vel = [vel_b_at_cp[0] - vel_a_at_cp[0], vel_b_at_cp[1] - vel_a_at_cp[1]];

                        let tangent = [-n[1], n[0]];
                        let vel_along_tangent = rel_vel[0] * tangent[0] + rel_vel[1] * tangent[1];

                        let ra_cross_t = ra[0] * tangent[1] - ra[1] * tangent[0];
                        let rb_cross_t = rb[0] * tangent[1] - rb[1] * tangent[0];
                        let inv_mass_sum_t = inv_mass_a + inv_mass_b
                            + ra_cross_t * ra_cross_t * inv_inertia_a
                            + rb_cross_t * rb_cross_t * inv_inertia_b;

                        let jt_new = -vel_along_tangent / inv_mass_sum_t;
                        let jt_old = self.manifolds[key].points[pi].tangent_impulse;
                        let max_friction = j_accumulated.abs() * friction;
                        let jt_accumulated = (jt_old + jt_new).clamp(-max_friction, max_friction);
                        let jt_applied = jt_accumulated - jt_old;
                        self.manifolds.get_mut(key).unwrap().points[pi].tangent_impulse = jt_accumulated;

                        let impulse_t = [jt_applied * tangent[0], jt_applied * tangent[1]];

                        if let Some(ba) = self.bodies.get_mut(body_ah(body_a_handle))
                            && ba.is_dynamic()
                        {
                            ba.linear_velocity[0] -= impulse_t[0] * ba.inv_mass;
                            ba.linear_velocity[1] -= impulse_t[1] * ba.inv_mass;
                            ba.angular_velocity -= ra_cross_t * jt_applied * ba.inv_inertia;
                        }
                        if let Some(bb) = self.bodies.get_mut(body_ah(body_b_handle))
                            && bb.is_dynamic()
                        {
                            bb.linear_velocity[0] += impulse_t[0] * bb.inv_mass;
                            bb.linear_velocity[1] += impulse_t[1] * bb.inv_mass;
                            bb.angular_velocity += rb_cross_t * jt_applied * bb.inv_inertia;
                        }
                    }

                    // Rolling friction — apply torque opposing angular velocity
                    let rolling_friction = materials[ki].rolling_friction;
                    if rolling_friction > 0.0 {
                        let normal_force = j_accumulated.abs();
                        let roll_torque = rolling_friction * normal_force;

                        if let Some(ba) = self.bodies.get_mut(body_ah(body_a_handle))
                            && ba.is_dynamic()
                            && ba.angular_velocity.abs() > EPSILON
                        {
                            let old_sign = ba.angular_velocity > 0.0;
                            let sign = if old_sign { -1.0 } else { 1.0 };
                            ba.angular_velocity += sign * roll_torque * ba.inv_inertia;
                            // Don't reverse direction
                            if (ba.angular_velocity > 0.0) != old_sign {
                                ba.angular_velocity = 0.0;
                            }
                        }
                        if let Some(bb) = self.bodies.get_mut(body_ah(body_b_handle))
                            && bb.is_dynamic()
                            && bb.angular_velocity.abs() > EPSILON
                        {
                            let old_sign = bb.angular_velocity > 0.0;
                            let sign = if old_sign { -1.0 } else { 1.0 };
                            bb.angular_velocity += sign * roll_torque * bb.inv_inertia;
                            // Don't reverse direction
                            if (bb.angular_velocity > 0.0) != old_sign {
                                bb.angular_velocity = 0.0;
                            }
                        }
                    }
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Positional correction (Baumgarte stabilization)
    // -----------------------------------------------------------------------

    fn solve_positions(&mut self, iterations: u32, slop: f64, percent: f64) {
        // Collect positional correction data from manifolds
        struct PosCorrection {
            body_a: crate::body::BodyHandle,
            body_b: crate::body::BodyHandle,
            normal: [f64; 2],
            depth: f64,
            is_sensor: bool,
        }

        let corrections: Vec<PosCorrection> = self.manifolds.values().flat_map(|manifold| {
            let is_sensor = match (
                self.colliders.get(coll_ah(manifold.collider_a)),
                self.colliders.get(coll_ah(manifold.collider_b)),
            ) {
                (Some(a), Some(b)) => a.is_sensor || b.is_sensor,
                _ => false,
            };
            manifold.points.iter().map(move |mp| PosCorrection {
                body_a: manifold.body_a,
                body_b: manifold.body_b,
                normal: manifold.normal,
                depth: mp.depth,
                is_sensor,
            })
        }).collect();

        for _ in 0..iterations {
            for corr in &corrections {
                if corr.is_sensor {
                    continue;
                }

                let inv_mass_a = self.bodies.get(body_ah(corr.body_a)).map(|b| b.inv_mass).unwrap_or(0.0);
                let inv_mass_b = self.bodies.get(body_ah(corr.body_b)).map(|b| b.inv_mass).unwrap_or(0.0);
                let inv_mass_sum = inv_mass_a + inv_mass_b;

                if inv_mass_sum == 0.0 {
                    continue;
                }

                let n = corr.normal;
                let correction_mag = (corr.depth - slop).max(0.0) / inv_mass_sum * percent;
                let correction = [correction_mag * n[0], correction_mag * n[1]];

                if let Some(ba) = self.bodies.get_mut(body_ah(corr.body_a))
                    && ba.is_dynamic()
                {
                    ba.position[0] -= correction[0] * ba.inv_mass;
                    ba.position[1] -= correction[1] * ba.inv_mass;
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(corr.body_b))
                    && bb.is_dynamic()
                {
                    bb.position[0] += correction[0] * bb.inv_mass;
                    bb.position[1] += correction[1] * bb.inv_mass;
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Collision event generation
    // -----------------------------------------------------------------------

    fn generate_events(&mut self) -> Vec<CollisionEvent> {
        let mut events = Vec::new();

        let current_keys: BTreeSet<ManifoldKey> = self.manifolds.keys().copied().collect();

        for key in &current_keys {
            if !self.prev_manifold_keys.contains(key) {
                events.push(CollisionEvent::Started {
                    collider_a: key.0,
                    collider_b: key.1,
                });
            } else {
                events.push(CollisionEvent::Ongoing {
                    collider_a: key.0,
                    collider_b: key.1,
                });
            }
        }

        for key in &self.prev_manifold_keys {
            if !current_keys.contains(key) {
                events.push(CollisionEvent::Stopped {
                    collider_a: key.0,
                    collider_b: key.1,
                });
            }
        }

        self.prev_manifold_keys = current_keys;
        events
    }
}
