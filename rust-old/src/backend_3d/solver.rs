//! Step, broadphase, contact solver, position solver, event generation, sleep.

use std::collections::BTreeMap;

use hisab::{DMat3, DVec3};

use crate::body::BodyHandle;
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
        constraint_frequency: f64,
        constraint_damping_ratio: f64,
        broadphase_kind: crate::config::BroadphaseKind,
    ) {
        let g = DVec3::from_array(gravity);
        // 1. Integrate velocities; refresh world-space inverse inertia
        for rb in self.bodies.values_mut() {
            rb.integrate_velocities(g, dt, max_velocity);
            rb.update_inertia_world();
        }

        // 1b. Explicit joint forces (once per sub-step) + joint constraint rows
        let mut joint_rows = self.prepare_joints(dt);

        // 2-3. Broadphase (with speculative velocity expansion) + narrowphase
        let broad_pairs = self.broadphase(broadphase_kind, dt);
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

        // 5. Shock propagation: sort contacts bottom-up (closer to static bodies first)
        let contacts = {
            let mut contacts = contacts;
            let mut body_level: BTreeMap<crate::body::BodyHandle, u32> = BTreeMap::new();
            for rb in self.bodies.values() {
                if rb.is_static() || !rb.is_dynamic() {
                    body_level.insert(rb.handle, 0);
                }
            }
            let mut changed = true;
            while changed {
                changed = false;
                for c in &contacts {
                    let level_a = body_level.get(&c.body_a).copied();
                    let level_b = body_level.get(&c.body_b).copied();
                    match (level_a, level_b) {
                        (Some(la), None) => {
                            body_level.insert(c.body_b, la + 1);
                            changed = true;
                        }
                        (None, Some(lb)) => {
                            body_level.insert(c.body_a, lb + 1);
                            changed = true;
                        }
                        (Some(la), Some(lb)) => {
                            if la + 1 < lb {
                                body_level.insert(c.body_b, la + 1);
                                changed = true;
                            } else if lb + 1 < la {
                                body_level.insert(c.body_a, lb + 1);
                                changed = true;
                            }
                        }
                        (None, None) => {}
                    }
                }
            }
            contacts.sort_by_key(|c| {
                let la = body_level.get(&c.body_a).copied().unwrap_or(u32::MAX);
                let lb = body_level.get(&c.body_b).copied().unwrap_or(u32::MAX);
                la.min(lb)
            });
            contacts
        };

        // 6. Solve velocity constraints (contacts and joints interleaved)
        let mut constraints = self.prepare_contacts(&contacts);
        for _ in 0..velocity_iterations {
            self.solve_contacts(&mut constraints);
            self.solve_joint_velocities(&mut joint_rows, dt);
        }
        self.break_joints(&mut joint_rows, dt);

        // 7. Positional correction (split impulse: soft constraint or Baumgarte)
        for rb in self.bodies.values_mut() {
            rb.pseudo_velocity = DVec3::ZERO;
            rb.pseudo_angular_velocity = DVec3::ZERO;
        }
        if dt > 0.0 {
            let erp = error_reduction(
                constraint_frequency,
                constraint_damping_ratio,
                correction,
                dt,
            );
            for _ in 0..position_iterations {
                self.solve_contact_positions(&mut constraints, slop, erp, dt);
                self.solve_joint_positions(&mut joint_rows, erp, dt);
            }
        }

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
                    // Sleeping bodies hold still: drop the residual velocity.
                    rb.is_sleeping = true;
                    rb.linear_velocity = DVec3::ZERO;
                    rb.angular_velocity = DVec3::ZERO;
                }
            } else {
                rb.sleep_timer = 0.0;
                rb.is_sleeping = false;
            }
        }

        // 10. Record which pairs touch at the end of this sub-step
        self.contact_tracker.record_substep(
            contacts
                .iter()
                .map(|c| crate::contact_tracker::pair_key(c.collider_a, c.collider_b)),
        );
    }

    /// Test helper: one full frame (single sub-step) returning its events.
    #[cfg(test)]
    #[allow(clippy::too_many_arguments)]
    pub fn step_frame(
        &mut self,
        gravity: [f64; 3],
        dt: f64,
        velocity_iterations: u32,
        position_iterations: u32,
        slop: f64,
        correction: f64,
        max_velocity: f64,
        constraint_frequency: f64,
        constraint_damping_ratio: f64,
        broadphase_kind: crate::config::BroadphaseKind,
    ) -> Vec<CollisionEvent> {
        self.step(
            gravity,
            dt,
            velocity_iterations,
            position_iterations,
            slop,
            correction,
            max_velocity,
            constraint_frequency,
            constraint_damping_ratio,
            broadphase_kind,
        );
        self.finish_step()
    }

    /// Finish an outer step: clear accumulated forces (they act for the whole
    /// step, across all sub-steps) and return the frame's collision events.
    pub fn finish_step(&mut self) -> Vec<CollisionEvent> {
        for rb in self.bodies.values_mut() {
            rb.clear_forces();
        }
        self.contact_tracker.finish()
    }

    // -----------------------------------------------------------------------
    // Broadphase
    // -----------------------------------------------------------------------

    #[must_use]
    fn broadphase(
        &self,
        kind: crate::config::BroadphaseKind,
        dt: f64,
    ) -> Vec<(ColliderHandle, ColliderHandle)> {
        use crate::aabb_tree::{AabbTree, TreeAabb};

        let collider_aabbs: Vec<(ColliderHandle, Aabb3d)> = self
            .colliders
            .values()
            .filter_map(|c| {
                let rb = self.bodies.get(body_ah(c.body))?;
                let mut aabb = c.world_aabb(rb.position, rb.rotation);
                // Speculative expansion: expand AABB along velocity direction
                let vel = rb.linear_velocity * dt;
                aabb.min = aabb.min.min(aabb.min + vel);
                aabb.max = aabb.max.max(aabb.max + vel);
                Some((c.handle, aabb))
            })
            .collect();

        let candidates = match kind {
            crate::config::BroadphaseKind::SpatialHash => {
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
                grid.query_pairs()
            }
            crate::config::BroadphaseKind::AabbTree => {
                let mut tree = AabbTree::new();
                for (handle, aabb) in &collider_aabbs {
                    tree.insert(
                        *handle,
                        TreeAabb::from_3d(aabb.min.to_array(), aabb.max.to_array()),
                    );
                }
                tree.query_pairs()
            }
        };

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

    #[must_use]
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

    /// Build per-contact solver data for this sub-step: lever arms, effective
    /// masses, combined materials, and the restitution velocity target taken
    /// from the approach speed *before* the solve.
    #[must_use]
    fn prepare_contacts(&self, contacts: &[Contact3d]) -> Vec<ContactConstraint> {
        use crate::material::CombineRule;

        /// Pick the higher-priority combine rule, then apply it.
        fn combine_property(a: f64, b: f64, rule_a: CombineRule, rule_b: CombineRule) -> f64 {
            rule_a.max(rule_b).combine(a, b)
        }

        let mut out = Vec::with_capacity(contacts.len());
        for contact in contacts {
            let (Some(ca), Some(cb)) = (
                self.colliders.get(coll_ah(contact.collider_a)),
                self.colliders.get(coll_ah(contact.collider_b)),
            ) else {
                continue;
            };
            if ca.is_sensor || cb.is_sensor {
                continue;
            }
            let (Some(ba), Some(bb)) = (
                self.bodies.get(body_ah(contact.body_a)),
                self.bodies.get(body_ah(contact.body_b)),
            ) else {
                continue;
            };
            let (inv_mass_a, inv_i_a) = solver_mass(ba);
            let (inv_mass_b, inv_i_b) = solver_mass(bb);
            if inv_mass_a == 0.0 && inv_mass_b == 0.0 {
                continue;
            }
            let n = contact.normal.normalize_or_zero();
            if n == DVec3::ZERO || !contact.point.is_finite() {
                continue;
            }

            let (ma, mb) = (&ca.material, &cb.material);
            let restitution = sanitize_coefficient(combine_property(
                ma.restitution,
                mb.restitution,
                ma.restitution_combine,
                mb.restitution_combine,
            ));
            let friction = sanitize_coefficient(combine_property(
                ma.friction,
                mb.friction,
                ma.friction_combine,
                mb.friction_combine,
            ));
            let static_friction = sanitize_coefficient(combine_property(
                ma.effective_static_friction(),
                mb.effective_static_friction(),
                ma.friction_combine,
                mb.friction_combine,
            ));
            let rolling_friction =
                sanitize_coefficient((ma.rolling_friction + mb.rolling_friction) * 0.5);

            let ra = contact.point - ba.position;
            let rb = contact.point - bb.position;
            let (t1, t2) = n.any_orthonormal_pair();
            let eff = |d: DVec3| {
                let ka = ra.cross(d);
                let kb = rb.cross(d);
                inv_mass_a + inv_mass_b + ka.dot(inv_i_a * ka) + kb.dot(inv_i_b * kb)
            };
            let inv = |k: f64| if k > EPSILON { 1.0 / k } else { 0.0 };

            let rel_vel = bb.linear_velocity + bb.angular_velocity.cross(rb)
                - ba.linear_velocity
                - ba.angular_velocity.cross(ra);
            let vn0 = rel_vel.dot(n);
            let velocity_bias = if vn0 < -RESTITUTION_VELOCITY_THRESHOLD {
                -restitution * vn0
            } else {
                0.0
            };
            let roll_k = inv_i_a + inv_i_b;

            out.push(ContactConstraint {
                body_a: contact.body_a,
                body_b: contact.body_b,
                normal: n,
                tangents: [t1, t2],
                ra,
                rb,
                inv_mass_a,
                inv_mass_b,
                inv_i_a,
                inv_i_b,
                normal_mass: inv(eff(n)),
                tangent_mass: [inv(eff(t1)), inv(eff(t2))],
                roll_k,
                velocity_bias,
                friction,
                static_friction,
                rolling_friction,
                depth: contact.depth,
                normal_impulse: 0.0,
                tangent_impulse: [0.0, 0.0],
                rolling_impulse: DVec3::ZERO,
                position_impulse: 0.0,
            });
        }
        out
    }

    /// Current (linear, angular) velocity of a body, zero if missing.
    #[inline]
    #[must_use]
    fn body_velocity(&self, h: BodyHandle) -> (DVec3, DVec3) {
        self.bodies
            .get(body_ah(h))
            .map_or((DVec3::ZERO, DVec3::ZERO), |b| {
                (b.linear_velocity, b.angular_velocity)
            })
    }

    /// Apply impulse `p` at the contact: −p on A, +p on B.
    #[inline]
    fn apply_contact_impulse(&mut self, c: &ContactConstraint, p: DVec3) {
        if c.inv_mass_a > 0.0
            && let Some(ba) = self.bodies.get_mut(body_ah(c.body_a))
        {
            ba.linear_velocity -= p * c.inv_mass_a;
            ba.angular_velocity -= c.inv_i_a * c.ra.cross(p);
        }
        if c.inv_mass_b > 0.0
            && let Some(bb) = self.bodies.get_mut(body_ah(c.body_b))
        {
            bb.linear_velocity += p * c.inv_mass_b;
            bb.angular_velocity += c.inv_i_b * c.rb.cross(p);
        }
    }

    /// Relative velocity of B with respect to A at the contact point.
    #[inline]
    #[must_use]
    fn contact_relative_velocity(&self, c: &ContactConstraint) -> DVec3 {
        let (va, wa) = self.body_velocity(c.body_a);
        let (vb, wb) = self.body_velocity(c.body_b);
        vb + wb.cross(c.rb) - va - wa.cross(c.ra)
    }

    /// Sequential-impulse velocity solve for contacts, with accumulated
    /// (clamped) normal, friction and rolling-resistance impulses.
    fn solve_contacts(&mut self, constraints: &mut [ContactConstraint]) {
        for c in constraints.iter_mut() {
            // --- Normal: vn >= restitution target, accumulated impulse >= 0
            let vn = self.contact_relative_velocity(c).dot(c.normal);
            let lambda = -(vn - c.velocity_bias) * c.normal_mass;
            let new_impulse = (c.normal_impulse + lambda).max(0.0);
            let applied = new_impulse - c.normal_impulse;
            c.normal_impulse = new_impulse;
            if applied != 0.0 {
                self.apply_contact_impulse(c, c.normal * applied);
            }

            // --- Friction: two tangent directions, accumulated impulse
            // clamped to the friction disc mu * accumulated normal impulse.
            // Static friction applies while the contact is (nearly) not
            // slipping or can be stopped within the static cone; otherwise
            // the kinetic coefficient limits the impulse.
            if c.friction > 0.0 || c.static_friction > 0.0 {
                let rel = self.contact_relative_velocity(c);
                let vt = [rel.dot(c.tangents[0]), rel.dot(c.tangents[1])];
                let slip = vt[0].hypot(vt[1]);
                let mut new_t = [
                    c.tangent_impulse[0] - vt[0] * c.tangent_mass[0],
                    c.tangent_impulse[1] - vt[1] * c.tangent_mass[1],
                ];
                let len = new_t[0].hypot(new_t[1]);
                let max_static = (c.static_friction * c.normal_impulse).max(0.0);
                let mu = if slip < STATIC_FRICTION_SLIP_SPEED || len <= max_static {
                    c.static_friction
                } else {
                    c.friction
                };
                let max_friction = (mu * c.normal_impulse).max(0.0);
                if len > max_friction {
                    let scale = if len > 0.0 { max_friction / len } else { 0.0 };
                    new_t = [new_t[0] * scale, new_t[1] * scale];
                }
                let d = [
                    new_t[0] - c.tangent_impulse[0],
                    new_t[1] - c.tangent_impulse[1],
                ];
                c.tangent_impulse = new_t;
                let p = c.tangents[0] * d[0] + c.tangents[1] * d[1];
                if p != DVec3::ZERO {
                    self.apply_contact_impulse(c, p);
                }
            }

            // --- Rolling resistance: angular impulse opposing relative
            // rolling (twist about the normal excluded), clamped to
            // rolling_friction * accumulated normal impulse.
            if c.rolling_friction > 0.0 {
                let (_, wa) = self.body_velocity(c.body_a);
                let (_, wb) = self.body_velocity(c.body_b);
                let mut w_rel = wb - wa;
                w_rel -= c.normal * c.normal.dot(w_rel);
                let speed = w_rel.length();
                if speed > EPSILON {
                    let dir = w_rel / speed;
                    let k = dir.dot(c.roll_k * dir);
                    if k > EPSILON {
                        let max_roll = (c.rolling_friction * c.normal_impulse).max(0.0);
                        let mut new_r = c.rolling_impulse - dir * (speed / k);
                        let len = new_r.length();
                        if len > max_roll {
                            new_r *= if len > 0.0 { max_roll / len } else { 0.0 };
                        }
                        let applied = new_r - c.rolling_impulse;
                        c.rolling_impulse = new_r;
                        if c.inv_mass_a > 0.0
                            && let Some(ba) = self.bodies.get_mut(body_ah(c.body_a))
                        {
                            ba.angular_velocity -= c.inv_i_a * applied;
                        }
                        if c.inv_mass_b > 0.0
                            && let Some(bb) = self.bodies.get_mut(body_ah(c.body_b))
                        {
                            bb.angular_velocity += c.inv_i_b * applied;
                        }
                    }
                }
            }
        }
    }

    /// One split-impulse position iteration over the contacts.
    ///
    /// Each contact accumulates a non-negative pseudo-impulse that drives the
    /// pseudo relative normal velocity to `erp * (depth - slop) / dt`, so
    /// several contact points or iterations never multiply the correction.
    /// Pseudo-velocities move positions in `integrate_positions` only and
    /// never feed back into real velocity.
    fn solve_contact_positions(
        &mut self,
        constraints: &mut [ContactConstraint],
        slop: f64,
        erp: f64,
        dt: f64,
    ) {
        for c in constraints.iter_mut() {
            let penetration = (c.depth - slop).max(0.0);
            let bias = erp * penetration / dt;
            let (pva, pwa) = self.body_pseudo_velocity(c.body_a);
            let (pvb, pwb) = self.body_pseudo_velocity(c.body_b);
            let vp = (pvb + pwb.cross(c.rb) - pva - pwa.cross(c.ra)).dot(c.normal);
            let lambda = (bias - vp) * c.normal_mass;
            let new_impulse = (c.position_impulse + lambda).max(0.0);
            let applied = new_impulse - c.position_impulse;
            c.position_impulse = new_impulse;
            if applied == 0.0 {
                continue;
            }
            let p = c.normal * applied;
            if c.inv_mass_a > 0.0
                && let Some(ba) = self.bodies.get_mut(body_ah(c.body_a))
            {
                ba.pseudo_velocity -= p * c.inv_mass_a;
                ba.pseudo_angular_velocity -= c.inv_i_a * c.ra.cross(p);
            }
            if c.inv_mass_b > 0.0
                && let Some(bb) = self.bodies.get_mut(body_ah(c.body_b))
            {
                bb.pseudo_velocity += p * c.inv_mass_b;
                bb.pseudo_angular_velocity += c.inv_i_b * c.rb.cross(p);
            }
        }
    }

    /// Current (linear, angular) pseudo-velocity of a body, zero if missing.
    #[inline]
    #[must_use]
    pub(super) fn body_pseudo_velocity(&self, h: BodyHandle) -> (DVec3, DVec3) {
        self.bodies
            .get(body_ah(h))
            .map_or((DVec3::ZERO, DVec3::ZERO), |b| {
                (b.pseudo_velocity, b.pseudo_angular_velocity)
            })
    }
}

/// Velocity below which a contact is "not sliding" and static friction applies.
const STATIC_FRICTION_SLIP_SPEED: f64 = 0.01;
/// Approach speed below which restitution is ignored (prevents jitter).
const RESTITUTION_VELOCITY_THRESHOLD: f64 = 1.0;

/// Per-contact solver state for one sub-step.
#[derive(Debug, Clone)]
struct ContactConstraint {
    body_a: BodyHandle,
    body_b: BodyHandle,
    /// Unit normal from A to B.
    normal: DVec3,
    tangents: [DVec3; 2],
    ra: DVec3,
    rb: DVec3,
    inv_mass_a: f64,
    inv_mass_b: f64,
    inv_i_a: DMat3,
    inv_i_b: DMat3,
    normal_mass: f64,
    tangent_mass: [f64; 2],
    /// `inv_i_a + inv_i_b`, for rolling resistance.
    roll_k: DMat3,
    /// Target separating normal velocity (restitution).
    velocity_bias: f64,
    friction: f64,
    static_friction: f64,
    rolling_friction: f64,
    depth: f64,
    normal_impulse: f64,
    tangent_impulse: [f64; 2],
    rolling_impulse: DVec3,
    position_impulse: f64,
}

/// Inverse mass and world inverse inertia the solver should use for a body:
/// zero for static/kinematic bodies.
#[inline]
#[must_use]
pub(super) fn solver_mass(b: &RigidBody3d) -> (f64, DMat3) {
    if b.is_dynamic() {
        (b.inv_mass, b.inv_inertia_world)
    } else {
        (0.0, DMat3::ZERO)
    }
}

/// Clamp a material coefficient to `[0, f64::MAX]`, mapping NaN to 0, so
/// products and clamps built from it can't produce NaN or panic.
#[inline]
#[must_use]
fn sanitize_coefficient(x: f64) -> f64 {
    if x.is_nan() {
        0.0
    } else {
        x.clamp(0.0, f64::MAX)
    }
}

/// Error-reduction parameter for position correction: from the soft
/// constraint spring (`frequency` Hz, `damping_ratio`) when `frequency > 0`,
/// else the Baumgarte fraction `correction`. Clamped to `[0, 1]`.
#[inline]
#[must_use]
pub(super) fn error_reduction(frequency: f64, damping_ratio: f64, correction: f64, dt: f64) -> f64 {
    let erp = if frequency > 0.0 && dt > 0.0 {
        let omega = 2.0 * std::f64::consts::PI * frequency;
        let d = 2.0 * damping_ratio * omega;
        let k = omega * omega;
        dt * k / (d + dt * k)
    } else {
        correction
    };
    if erp.is_finite() {
        erp.clamp(0.0, 1.0)
    } else {
        0.0
    }
}
