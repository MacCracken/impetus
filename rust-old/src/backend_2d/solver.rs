//! Step, broadphase, manifold update, warm start, contact solver, position solver, event generation.

use std::collections::BTreeMap;

use crate::collider::ColliderHandle;
use crate::event::CollisionEvent;
use crate::spatial_hash::SpatialHashGrid;

use super::narrowphase::*;
use super::state::PhysicsState2d;
use super::types::*;
use super::{body_ah, coll_ah};

/// Approach speed below which restitution is ignored, so resting contacts
/// don't micro-bounce.
const RESTITUTION_VELOCITY_THRESHOLD: f64 = 1.0;
/// Block solver is used only when the 2x2 effective-mass matrix is well
/// conditioned (Box2D `k_maxConditionNumber`).
const MAX_CONDITION_NUMBER: f64 = 1000.0;

/// Combined material properties of a manifold's collider pair.
struct ManifoldMaterial {
    restitution: f64,
    friction: f64,
    static_friction: f64,
    rolling_friction: f64,
    is_sensor: bool,
}

/// Pick the higher-priority combine rule, then apply it.
fn combine_property(
    a: f64,
    b: f64,
    rule_a: crate::material::CombineRule,
    rule_b: crate::material::CombineRule,
) -> f64 {
    rule_a.max(rule_b).combine(a, b)
}

/// Per-sub-step contact solver data: solve order, materials, restitution targets.
pub(super) struct ContactPrep {
    keys: Vec<ManifoldKey>,
    materials: Vec<ManifoldMaterial>,
    velocity_bias: Vec<[f64; MAX_MANIFOLD_POINTS]>,
}

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
        constraint_frequency: f64,
        constraint_damping_ratio: f64,
        broadphase_kind: crate::config::BroadphaseKind,
        solver_kind: crate::config::SolverKind,
    ) {
        match solver_kind {
            crate::config::SolverKind::Xpbd => self.step_xpbd(
                gravity,
                dt,
                velocity_iterations,
                slop,
                max_velocity,
                constraint_frequency,
                broadphase_kind,
            ),
            _ => self.step_si(
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
            ),
        }
        // Record which pairs touch at the end of this sub-step.
        self.contact_tracker
            .record_substep(self.manifolds.keys().copied());
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
        solver_kind: crate::config::SolverKind,
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
            solver_kind,
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
    // Sequential Impulse step (default)
    // -----------------------------------------------------------------------

    #[allow(clippy::too_many_arguments)]
    fn step_si(
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
        let gravity_2d = [gravity[0], gravity[1]];
        // 1. Integrate velocities
        for rb in self.bodies.values_mut() {
            rb.integrate_velocities(gravity_2d, dt, max_velocity);
        }

        // 2. Broadphase (with speculative expansion by velocity)
        let broad_pairs = self.broadphase(broadphase_kind, dt);

        // 3. Narrowphase — raw contacts
        let contacts = self.narrowphase(&broad_pairs);

        // 4. Update manifold cache (match contacts to existing manifolds)
        self.update_manifolds(&contacts);

        // 5. Wake sleeping bodies on contact with non-sleeping moving bodies
        self.wake_on_contact();

        // 6. Warm start — apply cached impulses from previous frame
        self.warm_start();

        // 7–8. Solve joint and contact velocity constraints, interleaved
        let contact_prep = self.prepare_contacts();
        let mut joints = self.prepare_joints(dt);
        for _ in 0..velocity_iterations {
            self.solve_joints_once(&mut joints);
            self.solve_contacts_once(&contact_prep);
        }
        self.break_joints(&joints, dt);

        // 9. Positional correction (soft constraint or Baumgarte)
        self.solve_positions(
            position_iterations,
            slop,
            correction,
            dt,
            constraint_frequency,
            constraint_damping_ratio,
        );

        // 10. Integrate positions
        for rb in self.bodies.values_mut() {
            rb.integrate_positions(dt);
        }

        // 11. Build simulation islands and sleep check
        self.build_islands_and_sleep(dt);
    }

    // -----------------------------------------------------------------------
    // XPBD step — position-based dynamics with compliance
    // -----------------------------------------------------------------------

    #[allow(clippy::too_many_arguments)]
    fn step_xpbd(
        &mut self,
        gravity: [f64; 3],
        dt: f64,
        iterations: u32,
        slop: f64,
        max_velocity: f64,
        constraint_frequency: f64,
        broadphase_kind: crate::config::BroadphaseKind,
    ) {
        let gravity_2d = [gravity[0], gravity[1]];

        // 1. Store previous positions and integrate velocities
        for rb in self.bodies.values_mut() {
            rb.prev_position = rb.position;
            rb.prev_rotation = rb.rotation;
            rb.integrate_velocities(gravity_2d, dt, max_velocity);
        }

        // 2. Predict positions: x += v * dt (kinematic bodies move along
        //    their set velocity; static, massless and sleeping bodies don't)
        for rb in self.bodies.values_mut() {
            if rb.is_static() || (rb.is_dynamic() && (rb.inv_mass == 0.0 || rb.is_sleeping)) {
                continue;
            }
            rb.position[0] += rb.linear_velocity[0] * dt;
            rb.position[1] += rb.linear_velocity[1] * dt;
            if !rb.fixed_rotation {
                rb.rotation += rb.angular_velocity * dt;
            }
        }

        // 3. Broadphase + narrowphase at predicted positions
        let broad_pairs = self.broadphase(broadphase_kind, dt);
        let contacts = self.narrowphase(&broad_pairs);
        self.update_manifolds(&contacts);

        // 4. Wake on contact
        self.wake_on_contact();

        // 5. XPBD constraint solving — position-level with compliance
        // Compliance: α̃ = 1 / (stiffness * dt²)
        let alpha_tilde = if constraint_frequency > 0.0 && dt > EPSILON {
            let omega = 2.0 * std::f64::consts::PI * constraint_frequency;
            1.0 / (omega * omega * dt * dt)
        } else {
            0.0 // infinite stiffness (or dt too small for compliance)
        };

        // Initialize Lagrange multipliers for each manifold point
        let manifold_keys: Vec<ManifoldKey> = self.manifolds.keys().copied().collect();
        let mut lambdas: BTreeMap<(ManifoldKey, usize), f64> = BTreeMap::new();
        for key in &manifold_keys {
            if let Some(m) = self.manifolds.get(key) {
                for pi in 0..m.points.len() {
                    lambdas.insert((*key, pi), 0.0);
                }
            }
        }

        for _ in 0..iterations {
            // Contact constraints
            self.xpbd_solve_contacts(&manifold_keys, &mut lambdas, alpha_tilde, slop);

            // Joint constraints
            self.xpbd_solve_joints(dt, alpha_tilde);
        }

        // 6. Derive velocities from position change
        let inv_dt = if dt > EPSILON { 1.0 / dt } else { 0.0 };
        for rb in self.bodies.values_mut() {
            if !rb.is_dynamic() || rb.inv_mass == 0.0 || rb.is_sleeping {
                continue;
            }
            rb.linear_velocity[0] = (rb.position[0] - rb.prev_position[0]) * inv_dt;
            rb.linear_velocity[1] = (rb.position[1] - rb.prev_position[1]) * inv_dt;
            if !rb.fixed_rotation {
                rb.angular_velocity = (rb.rotation - rb.prev_rotation) * inv_dt;
            }
        }

        // 7. Velocity-level friction and restitution
        self.xpbd_velocity_solve(&manifold_keys);

        // 8. Islands + sleep
        self.build_islands_and_sleep(dt);
    }

    /// Wake sleeping bodies that are in contact with moving non-sleeping bodies.
    fn wake_on_contact(&mut self) {
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
    }

    // -----------------------------------------------------------------------
    // Broadphase — spatial hash grid
    // -----------------------------------------------------------------------

    fn broadphase(
        &self,
        kind: crate::config::BroadphaseKind,
        dt: f64,
    ) -> Vec<(ColliderHandle, ColliderHandle)> {
        use crate::aabb_tree::{AabbTree, TreeAabb};

        // Compute AABBs for all colliders, expanded by velocity for speculative contacts
        let collider_aabbs: Vec<(ColliderHandle, Aabb2d)> = self
            .colliders
            .values()
            .filter_map(|c| {
                let rb = self.bodies.get(body_ah(c.body))?;
                let mut aabb = c.world_aabb(rb.position, rb.rotation);
                // Speculative expansion: expand AABB along velocity direction
                let dx = rb.linear_velocity[0] * dt;
                let dy = rb.linear_velocity[1] * dt;
                if dx > 0.0 {
                    aabb.max[0] += dx;
                } else {
                    aabb.min[0] += dx;
                }
                if dy > 0.0 {
                    aabb.max[1] += dy;
                } else {
                    aabb.min[1] += dy;
                }
                Some((c.handle, aabb))
            })
            .collect();

        // Get candidate pairs from broadphase algorithm
        let candidates = match kind {
            crate::config::BroadphaseKind::SpatialHash => {
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
                grid.query_pairs()
            }
            crate::config::BroadphaseKind::AabbTree => {
                let mut tree = AabbTree::new();
                for (handle, aabb) in &collider_aabbs {
                    tree.insert(*handle, TreeAabb::from_2d(aabb.min, aabb.max));
                }
                tree.query_pairs()
            }
        };

        // Build AABB lookup for overlap verification
        let aabb_map: BTreeMap<ColliderHandle, Aabb2d> = collider_aabbs.into_iter().collect();

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
            if let (Some(ba), Some(bb)) = (
                self.bodies.get(body_ah(ca.body)),
                self.bodies.get(body_ah(cb.body)),
            ) && ba.is_static()
                && bb.is_static()
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

    fn narrowphase(&self, broad_pairs: &[(ColliderHandle, ColliderHandle)]) -> Vec<Contact> {
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

            // One-shot manifold generation: get all contact points in a single pass
            let multi = generate_contacts_multi(
                &ca.shape,
                pos_a,
                ba.rotation,
                &cb.shape,
                pos_b,
                bb.rotation,
            );
            for (normal, depth, point) in multi {
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

    /// Rebuild the manifold cache from this frame's one-shot contacts, carrying
    /// accumulated impulses over from matching points of last frame's
    /// manifold (warm starting). Rebuilding (rather than accumulating points
    /// across frames) keeps depths current and drops stale points as soon as
    /// the narrowphase stops reporting them.
    fn update_manifolds(&mut self, contacts: &[Contact]) {
        let pose = |h: crate::body::BodyHandle| {
            self.bodies
                .get(body_ah(h))
                .map(|b| (b.position, b.rotation))
                .unwrap_or(([0.0, 0.0], 0.0))
        };

        let mut fresh: BTreeMap<ManifoldKey, ContactManifold> = BTreeMap::new();
        for contact in contacts {
            let key = ordered_manifold_key(contact.collider_a, contact.collider_b);
            let swapped = key.0 != contact.collider_a;
            let (body_a, body_b, normal) = if swapped {
                (
                    contact.body_b,
                    contact.body_a,
                    [-contact.normal[0], -contact.normal[1]],
                )
            } else {
                (contact.body_a, contact.body_b, contact.normal)
            };
            let (pa, ra) = pose(body_a);
            let (pb, rb) = pose(body_b);
            let point = ManifoldPoint {
                local_a: world_to_local(contact.point, pa, ra),
                local_b: world_to_local(contact.point, pb, rb),
                normal_impulse: 0.0,
                tangent_impulse: 0.0,
                depth: contact.depth,
            };
            let m = fresh.entry(key).or_insert_with(|| ContactManifold {
                collider_a: key.0,
                collider_b: key.1,
                body_a,
                body_b,
                normal,
                points: Vec::with_capacity(2),
                rolling_impulse: 0.0,
            });
            if m.points.len() < MAX_MANIFOLD_POINTS {
                m.points.push(point);
            }
        }

        for (key, m) in &mut fresh {
            let Some(old) = self.manifolds.get(key) else {
                continue;
            };
            // Don't carry impulses across a normal flip.
            if old.normal[0] * m.normal[0] + old.normal[1] * m.normal[1] < 0.9 {
                continue;
            }
            m.rolling_impulse = old.rolling_impulse;
            let single = m.points.len() == 1 && old.points.len() == 1;
            let mut used = [false; MAX_MANIFOLD_POINTS];
            for p in &mut m.points {
                let mut best: Option<usize> = None;
                let mut best_d = MANIFOLD_MATCH_THRESHOLD * MANIFOLD_MATCH_THRESHOLD;
                for (i, op) in old.points.iter().enumerate() {
                    if used.get(i).copied().unwrap_or(true) {
                        continue;
                    }
                    // Match in either body's frame: contacts stay fixed on the
                    // body that slides over the other.
                    let da = (op.local_a[0] - p.local_a[0]).powi(2)
                        + (op.local_a[1] - p.local_a[1]).powi(2);
                    let db = (op.local_b[0] - p.local_b[0]).powi(2)
                        + (op.local_b[1] - p.local_b[1]).powi(2);
                    let d = da.min(db);
                    if d < best_d || single {
                        best_d = d;
                        best = Some(i);
                    }
                }
                if let Some(i) = best {
                    used[i] = true;
                    p.normal_impulse = old.points[i].normal_impulse;
                    p.tangent_impulse = old.points[i].tangent_impulse;
                }
            }
        }

        self.manifolds = fresh;
    }

    // -----------------------------------------------------------------------
    // Warm starting — apply cached impulses from previous frame
    // -----------------------------------------------------------------------

    /// Warm starting: scale last frame's accumulated impulses by
    /// `WARM_START_FACTOR`, store the scaled values back (so the accumulators
    /// match what was applied), and apply them.
    fn warm_start(&mut self) {
        struct Warm {
            body_a: crate::body::BodyHandle,
            body_b: crate::body::BodyHandle,
            /// Linear impulse applied to B (and −impulse to A).
            impulse: [f64; 2],
            ang_a: f64,
            ang_b: f64,
        }

        let mut pending: Vec<Warm> = Vec::new();
        for m in self.manifolds.values_mut() {
            m.rolling_impulse *= WARM_START_FACTOR;
            for p in &mut m.points {
                p.normal_impulse *= WARM_START_FACTOR;
                p.tangent_impulse *= WARM_START_FACTOR;
            }
            let is_sensor = match (
                self.colliders.get(coll_ah(m.collider_a)),
                self.colliders.get(coll_ah(m.collider_b)),
            ) {
                (Some(a), Some(b)) => a.is_sensor || b.is_sensor,
                _ => true,
            };
            if is_sensor {
                continue;
            }
            let (Some(ba), Some(bb)) = (
                self.bodies.get(body_ah(m.body_a)),
                self.bodies.get(body_ah(m.body_b)),
            ) else {
                continue;
            };
            let n = m.normal;
            let t = [-n[1], n[0]];
            for p in &m.points {
                if p.normal_impulse.abs() < EPSILON && p.tangent_impulse.abs() < EPSILON {
                    continue;
                }
                let cp = local_to_world(p.local_a, ba.position, ba.rotation);
                let ra = [cp[0] - ba.position[0], cp[1] - ba.position[1]];
                let rb = [cp[0] - bb.position[0], cp[1] - bb.position[1]];
                let impulse = [
                    p.normal_impulse * n[0] + p.tangent_impulse * t[0],
                    p.normal_impulse * n[1] + p.tangent_impulse * t[1],
                ];
                pending.push(Warm {
                    body_a: m.body_a,
                    body_b: m.body_b,
                    impulse,
                    ang_a: cross2(ra, impulse),
                    ang_b: cross2(rb, impulse),
                });
            }
            if m.rolling_impulse != 0.0 {
                pending.push(Warm {
                    body_a: m.body_a,
                    body_b: m.body_b,
                    impulse: [0.0, 0.0],
                    ang_a: m.rolling_impulse,
                    ang_b: m.rolling_impulse,
                });
            }
        }

        for w in &pending {
            if let Some(ba) = self.bodies.get_mut(body_ah(w.body_a)) {
                let (im, ii) = ba.solver_inv_mass();
                ba.linear_velocity[0] -= w.impulse[0] * im;
                ba.linear_velocity[1] -= w.impulse[1] * im;
                ba.angular_velocity -= w.ang_a * ii;
            }
            if let Some(bb) = self.bodies.get_mut(body_ah(w.body_b)) {
                let (im, ii) = bb.solver_inv_mass();
                bb.linear_velocity[0] += w.impulse[0] * im;
                bb.linear_velocity[1] += w.impulse[1] * im;
                bb.angular_velocity += w.ang_b * ii;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Contact constraint solver with accumulated impulses, friction, angular response
    // -----------------------------------------------------------------------

    /// Contact solve order (shock propagation), combined materials and
    /// restitution targets, computed once per sub-step.
    fn prepare_contacts(&self) -> ContactPrep {
        // Shock propagation: order manifolds bottom-up so contacts touching
        // static/kinematic bodies are solved first, improving stacking stability.
        let keys: Vec<ManifoldKey> = {
            let mut keys: Vec<ManifoldKey> = self.manifolds.keys().copied().collect();

            // Compute "level" per body: 0 = static/kinematic, then BFS from those
            let mut body_level: BTreeMap<crate::body::BodyHandle, u32> = BTreeMap::new();
            for rb in self.bodies.values() {
                if rb.is_static() || !rb.is_dynamic() {
                    body_level.insert(rb.handle, 0);
                }
            }

            // BFS propagation through contact graph
            let mut changed = true;
            while changed {
                changed = false;
                for manifold in self.manifolds.values() {
                    let level_a = body_level.get(&manifold.body_a).copied();
                    let level_b = body_level.get(&manifold.body_b).copied();

                    match (level_a, level_b) {
                        (Some(la), None) => {
                            body_level.insert(manifold.body_b, la + 1);
                            changed = true;
                        }
                        (None, Some(lb)) => {
                            body_level.insert(manifold.body_a, lb + 1);
                            changed = true;
                        }
                        (Some(la), Some(lb)) => {
                            if la + 1 < lb {
                                body_level.insert(manifold.body_b, la + 1);
                                changed = true;
                            } else if lb + 1 < la {
                                body_level.insert(manifold.body_a, lb + 1);
                                changed = true;
                            }
                        }
                        (None, None) => {}
                    }
                }
            }

            // Sort manifolds: lower min-level contacts first (closer to ground)
            keys.sort_by_key(|key| {
                let m = &self.manifolds[key];
                let la = body_level.get(&m.body_a).copied().unwrap_or(u32::MAX);
                let lb = body_level.get(&m.body_b).copied().unwrap_or(u32::MAX);
                la.min(lb)
            });

            keys
        };
        let materials: Vec<ManifoldMaterial> = keys
            .iter()
            .map(|key| {
                let manifold = &self.manifolds[key];
                let (rest, fric, sfric, roll_fric, sensor) = match (
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
                        combine_property(
                            a.material.effective_static_friction(),
                            b.material.effective_static_friction(),
                            a.material.friction_combine,
                            b.material.friction_combine,
                        ),
                        (a.material.rolling_friction + b.material.rolling_friction) * 0.5,
                        a.is_sensor || b.is_sensor,
                    ),
                    _ => (0.0, 0.0, 0.0, 0.0, false),
                };
                ManifoldMaterial {
                    restitution: rest,
                    friction: fric,
                    static_friction: sfric,
                    rolling_friction: roll_fric,
                    is_sensor: sensor,
                }
            })
            .collect();

        // Restitution: target separating speed per point, computed once from
        // the approach speed before iterating (re-deriving it from the
        // current velocity each iteration makes bounce depend on the
        // iteration count).
        let mut velocity_bias = vec![[0.0f64; MAX_MANIFOLD_POINTS]; keys.len()];
        for (ki, key) in keys.iter().enumerate() {
            let e = materials[ki].restitution;
            if e <= 0.0 || materials[ki].is_sensor {
                continue;
            }
            let Some(m) = self.manifolds.get(key) else {
                continue;
            };
            let (Some(ba), Some(bb)) = (
                self.bodies.get(body_ah(m.body_a)),
                self.bodies.get(body_ah(m.body_b)),
            ) else {
                continue;
            };
            let va = SolverVel::of(ba);
            let vb = SolverVel::of(bb);
            for (pi, p) in m.points.iter().take(MAX_MANIFOLD_POINTS).enumerate() {
                let cp = local_to_world(p.local_a, ba.position, ba.rotation);
                let ra = [cp[0] - ba.position[0], cp[1] - ba.position[1]];
                let rb = [cp[0] - bb.position[0], cp[1] - bb.position[1]];
                let vn = dot2(sub2(vb.at(rb), va.at(ra)), m.normal);
                if vn < -RESTITUTION_VELOCITY_THRESHOLD {
                    velocity_bias[ki][pi] = -e * vn;
                }
            }
        }

        ContactPrep {
            keys,
            materials,
            velocity_bias,
        }
    }

    /// One Gauss–Seidel pass over every contact manifold: block-solved or
    /// sequential normal impulses, Coulomb friction and rolling resistance.
    fn solve_contacts_once(&mut self, prep: &ContactPrep) {
        {
            for (ki, key) in prep.keys.iter().enumerate() {
                let mat = &prep.materials[ki];
                if mat.is_sensor {
                    continue;
                }
                let Some(m) = self.manifolds.get(key) else {
                    continue;
                };
                let (Some(ba), Some(bb)) = (
                    self.bodies.get(body_ah(m.body_a)),
                    self.bodies.get(body_ah(m.body_b)),
                ) else {
                    continue;
                };
                let (ima, iia) = ba.solver_inv_mass();
                let (imb, iib) = bb.solver_inv_mass();
                if ima == 0.0 && imb == 0.0 {
                    continue;
                }
                let mut va = SolverVel::of(ba);
                let mut vb = SolverVel::of(bb);
                let n = m.normal;
                let t = [-n[1], n[0]];
                let np = m.points.len().min(MAX_MANIFOLD_POINTS);
                let mut ra = [[0.0; 2]; MAX_MANIFOLD_POINTS];
                let mut rb = [[0.0; 2]; MAX_MANIFOLD_POINTS];
                let mut acc_n = [0.0; MAX_MANIFOLD_POINTS];
                let mut acc_t = [0.0; MAX_MANIFOLD_POINTS];
                for pi in 0..np {
                    let p = &m.points[pi];
                    let cp = local_to_world(p.local_a, ba.position, ba.rotation);
                    ra[pi] = [cp[0] - ba.position[0], cp[1] - ba.position[1]];
                    rb[pi] = [cp[0] - bb.position[0], cp[1] - bb.position[1]];
                    acc_n[pi] = p.normal_impulse;
                    acc_t[pi] = p.tangent_impulse;
                }
                let mut acc_roll = m.rolling_impulse;
                let (body_a, body_b) = (m.body_a, m.body_b);
                let a_dynamic = ima > 0.0 || iia > 0.0;
                let b_dynamic = imb > 0.0 || iib > 0.0;
                let bias = &prep.velocity_bias[ki];

                let eff_mass = |r_a: [f64; 2], r_b: [f64; 2], dir: [f64; 2]| {
                    let ca = cross2(r_a, dir);
                    let cb = cross2(r_b, dir);
                    ima + imb + ca * ca * iia + cb * cb * iib
                };

                // --- Normal impulses ---
                let mut solved_block = false;
                if np == 2 {
                    let rn = [
                        (cross2(ra[0], n), cross2(rb[0], n)),
                        (cross2(ra[1], n), cross2(rb[1], n)),
                    ];
                    let k11 = eff_mass(ra[0], rb[0], n);
                    let k22 = eff_mass(ra[1], rb[1], n);
                    let k12 = ima + imb + rn[0].0 * rn[1].0 * iia + rn[0].1 * rn[1].1 * iib;
                    let det = k11 * k22 - k12 * k12;
                    if k11 > EPSILON && k22 > EPSILON && k11 * k11 < MAX_CONDITION_NUMBER * det {
                        solved_block = true;
                        let a0 = acc_n[0];
                        let a1 = acc_n[1];
                        let vn0 = dot2(sub2(vb.at(rb[0]), va.at(ra[0])), n);
                        let vn1 = dot2(sub2(vb.at(rb[1]), va.at(ra[1])), n);
                        let b0 = vn0 - bias[0] - (k11 * a0 + k12 * a1);
                        let b1 = vn1 - bias[1] - (k12 * a0 + k22 * a1);
                        // Find x ≥ 0 with complementary vn ≥ 0 (Box2D block solver).
                        let x = {
                            let x0 = -(k22 * b0 - k12 * b1) / det;
                            let x1 = -(-k12 * b0 + k11 * b1) / det;
                            if x0 >= 0.0 && x1 >= 0.0 {
                                Some([x0, x1])
                            } else {
                                let x0 = -b0 / k11;
                                if x0 >= 0.0 && k12 * x0 + b1 >= 0.0 {
                                    Some([x0, 0.0])
                                } else {
                                    let x1 = -b1 / k22;
                                    if x1 >= 0.0 && k12 * x1 + b0 >= 0.0 {
                                        Some([0.0, x1])
                                    } else if b0 >= 0.0 && b1 >= 0.0 {
                                        Some([0.0, 0.0])
                                    } else {
                                        None
                                    }
                                }
                            }
                        };
                        if let Some(x) = x {
                            for pi in 0..2 {
                                let d = x[pi] - acc_n[pi];
                                let imp = [n[0] * d, n[1] * d];
                                va.apply(-1.0, imp, ra[pi], ima, iia);
                                vb.apply(1.0, imp, rb[pi], imb, iib);
                                acc_n[pi] = x[pi];
                            }
                        }
                    }
                }
                if !solved_block {
                    for pi in 0..np {
                        let k = eff_mass(ra[pi], rb[pi], n);
                        if k < EPSILON {
                            continue;
                        }
                        let vn = dot2(sub2(vb.at(rb[pi]), va.at(ra[pi])), n);
                        let new = (acc_n[pi] - (vn - bias[pi]) / k).max(0.0);
                        let d = new - acc_n[pi];
                        acc_n[pi] = new;
                        let imp = [n[0] * d, n[1] * d];
                        va.apply(-1.0, imp, ra[pi], ima, iia);
                        vb.apply(1.0, imp, rb[pi], imb, iib);
                    }
                }

                // --- Friction (Coulomb, static below a slip threshold) ---
                if mat.friction > 0.0 || mat.static_friction > 0.0 {
                    for pi in 0..np {
                        let k = eff_mass(ra[pi], rb[pi], t);
                        if k < EPSILON {
                            continue;
                        }
                        let vt = dot2(sub2(vb.at(rb[pi]), va.at(ra[pi])), t);
                        let mu = if vt.abs() < 0.01 {
                            mat.static_friction
                        } else {
                            mat.friction
                        };
                        let max_f = (mu * acc_n[pi]).max(0.0);
                        let mut new = (acc_t[pi] - vt / k).clamp(-max_f, max_f);
                        if !new.is_finite() {
                            new = 0.0;
                        }
                        let d = new - acc_t[pi];
                        acc_t[pi] = new;
                        let imp = [t[0] * d, t[1] * d];
                        va.apply(-1.0, imp, ra[pi], ima, iia);
                        vb.apply(1.0, imp, rb[pi], imb, iib);
                    }
                }

                // --- Rolling resistance (one accumulated angular impulse per manifold) ---
                if mat.rolling_friction > 0.0 && iia + iib > EPSILON {
                    let max_r = (mat.rolling_friction * acc_n[..np].iter().sum::<f64>()).max(0.0);
                    let mut new = (acc_roll - (vb.w - va.w) / (iia + iib)).clamp(-max_r, max_r);
                    if !new.is_finite() {
                        new = 0.0;
                    }
                    let d = new - acc_roll;
                    acc_roll = new;
                    va.w -= d * iia;
                    vb.w += d * iib;
                }

                // --- Write back ---
                if let Some(m) = self.manifolds.get_mut(key) {
                    for pi in 0..np {
                        m.points[pi].normal_impulse = acc_n[pi];
                        m.points[pi].tangent_impulse = acc_t[pi];
                    }
                    m.rolling_impulse = acc_roll;
                }
                if a_dynamic && let Some(b) = self.bodies.get_mut(body_ah(body_a)) {
                    va.store(b);
                }
                if b_dynamic && let Some(b) = self.bodies.get_mut(body_ah(body_b)) {
                    vb.store(b);
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Positional correction (split impulse — pseudo-velocity based)
    // -----------------------------------------------------------------------

    /// Split-impulse position correction: solve contact penetration with
    /// pseudo-velocities that move bodies apart without adding real momentum.
    ///
    /// Each point accumulates a clamped (≥ 0) pseudo-impulse toward a target
    /// separation speed, so several points per manifold or several iterations
    /// don't multiply the correction.
    #[allow(clippy::too_many_arguments)]
    fn solve_positions(
        &mut self,
        iterations: u32,
        slop: f64,
        percent: f64,
        dt: f64,
        constraint_frequency: f64,
        constraint_damping_ratio: f64,
    ) {
        struct PosPoint {
            body_a: crate::body::BodyHandle,
            body_b: crate::body::BodyHandle,
            ra: [f64; 2],
            rb: [f64; 2],
            normal: [f64; 2],
            target: f64,
            acc: f64,
        }

        for rb in self.bodies.values_mut() {
            rb.pseudo_velocity = [0.0, 0.0];
            rb.pseudo_angular_velocity = 0.0;
        }
        if dt <= EPSILON {
            return;
        }

        // Fraction of the remaining penetration removed this step: soft
        // constraint (spring-damper ERP) or Baumgarte.
        let fraction = if constraint_frequency > 0.0 {
            let omega = 2.0 * std::f64::consts::PI * constraint_frequency;
            let d = 2.0 * constraint_damping_ratio * omega;
            let k = omega * omega;
            dt * k / (d + dt * k)
        } else {
            percent
        };
        if fraction.is_nan() || fraction <= 0.0 {
            return;
        }
        let fraction = fraction.min(1.0);

        let mut points: Vec<PosPoint> = Vec::new();
        for m in self.manifolds.values() {
            let is_sensor = match (
                self.colliders.get(coll_ah(m.collider_a)),
                self.colliders.get(coll_ah(m.collider_b)),
            ) {
                (Some(a), Some(b)) => a.is_sensor || b.is_sensor,
                _ => true,
            };
            if is_sensor {
                continue;
            }
            let (Some(ba), Some(bb)) = (
                self.bodies.get(body_ah(m.body_a)),
                self.bodies.get(body_ah(m.body_b)),
            ) else {
                continue;
            };
            for p in &m.points {
                let penetration = p.depth - slop;
                if penetration <= 0.0 {
                    continue;
                }
                let cp = local_to_world(p.local_a, ba.position, ba.rotation);
                points.push(PosPoint {
                    body_a: m.body_a,
                    body_b: m.body_b,
                    ra: [cp[0] - ba.position[0], cp[1] - ba.position[1]],
                    rb: [cp[0] - bb.position[0], cp[1] - bb.position[1]],
                    normal: m.normal,
                    target: fraction * penetration / dt,
                    acc: 0.0,
                });
            }
        }

        for _ in 0..iterations.max(1) {
            for pt in &mut points {
                let (Some(ba), Some(bb)) = (
                    self.bodies.get(body_ah(pt.body_a)),
                    self.bodies.get(body_ah(pt.body_b)),
                ) else {
                    continue;
                };
                let (ima, iia) = ba.solver_inv_mass();
                let (imb, iib) = bb.solver_inv_mass();
                let mut va = SolverVel::pseudo_of(ba);
                let mut vb = SolverVel::pseudo_of(bb);
                let n = pt.normal;
                let ca = cross2(pt.ra, n);
                let cb = cross2(pt.rb, n);
                let k = ima + imb + ca * ca * iia + cb * cb * iib;
                if k < EPSILON {
                    continue;
                }
                let vn = dot2(sub2(vb.at(pt.rb), va.at(pt.ra)), n);
                let new = (pt.acc + (pt.target - vn) / k).max(0.0);
                let d = new - pt.acc;
                pt.acc = new;
                let imp = [n[0] * d, n[1] * d];
                va.apply(-1.0, imp, pt.ra, ima, iia);
                vb.apply(1.0, imp, pt.rb, imb, iib);
                if ima > 0.0
                    && let Some(b) = self.bodies.get_mut(body_ah(pt.body_a))
                {
                    va.store_pseudo(b);
                }
                if imb > 0.0
                    && let Some(b) = self.bodies.get_mut(body_ah(pt.body_b))
                {
                    vb.store_pseudo(b);
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // XPBD contact constraint solver (position-level)
    // -----------------------------------------------------------------------

    /// Solve contact constraints using XPBD position corrections with compliance.
    ///
    /// For each contact point, computes the constraint violation C (penetration)
    /// and applies a position correction weighted by inverse mass, accumulating
    /// the Lagrange multiplier λ to prevent over-correction.
    fn xpbd_solve_contacts(
        &mut self,
        keys: &[ManifoldKey],
        lambdas: &mut BTreeMap<(ManifoldKey, usize), f64>,
        alpha_tilde: f64,
        slop: f64,
    ) {
        for key in keys {
            let manifold = match self.manifolds.get(key) {
                Some(m) => m,
                None => continue,
            };

            // Skip sensors
            let is_sensor = match (
                self.colliders.get(coll_ah(manifold.collider_a)),
                self.colliders.get(coll_ah(manifold.collider_b)),
            ) {
                (Some(a), Some(b)) => a.is_sensor || b.is_sensor,
                _ => false,
            };
            if is_sensor {
                continue;
            }

            let body_a = manifold.body_a;
            let body_b = manifold.body_b;
            let normal = manifold.normal;
            let num_points = manifold.points.len();

            let inv_mass_a = self
                .bodies
                .get(body_ah(body_a))
                .map(|b| b.inv_mass)
                .unwrap_or(0.0);
            let inv_mass_b = self
                .bodies
                .get(body_ah(body_b))
                .map(|b| b.inv_mass)
                .unwrap_or(0.0);
            let w_sum = inv_mass_a + inv_mass_b;
            if w_sum < EPSILON {
                continue;
            }

            for pi in 0..num_points {
                let mp = &self.manifolds[key].points[pi];
                let initial_depth = mp.depth;

                // Constraint C = -penetration (negative when overlapping).
                // We want to drive C to 0 from below.
                let lambda_prev = lambdas.get(&(*key, pi)).copied().unwrap_or(0.0);

                // Effective penetration accounting for corrections already applied.
                // Each unit of lambda corresponds to w_sum units of separation gained.
                let current_pen = (initial_depth - lambda_prev * w_sum - slop).max(0.0);
                if current_pen < EPSILON {
                    continue;
                }

                // XPBD: Δλ = (penetration - α̃·λ) / (w + α̃)
                // This drives penetration to zero while respecting compliance.
                let denom = w_sum + alpha_tilde;
                if denom < EPSILON {
                    continue;
                }
                let delta_lambda = (current_pen - alpha_tilde * lambda_prev) / denom;

                // Clamp: accumulated lambda must stay non-negative (only push apart)
                let lambda_new = (lambda_prev + delta_lambda).max(0.0);
                let dl = lambda_new - lambda_prev;
                if dl.abs() < EPSILON {
                    continue;
                }

                lambdas.insert((*key, pi), lambda_new);
                if let Some(ba) = self.bodies.get_mut(body_ah(body_a))
                    && ba.is_dynamic()
                {
                    ba.position[0] -= dl * inv_mass_a * normal[0];
                    ba.position[1] -= dl * inv_mass_a * normal[1];
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(body_b))
                    && bb.is_dynamic()
                {
                    bb.position[0] += dl * inv_mass_b * normal[0];
                    bb.position[1] += dl * inv_mass_b * normal[1];
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // XPBD joint constraint solver (position-level)
    // -----------------------------------------------------------------------

    /// Solve joint constraints at position level for XPBD.
    fn xpbd_solve_joints(&mut self, dt: f64, alpha_tilde: f64) {
        use crate::joint::JointType;

        let joint_data: Vec<_> = self
            .joints
            .iter()
            .map(|(_, j)| {
                (
                    j.body_a,
                    j.body_b,
                    j.joint_type.clone(),
                    j.local_anchor_a,
                    j.local_anchor_b,
                )
            })
            .collect();

        for (body_a, body_b, joint_type, anchor_a, anchor_b) in &joint_data {
            let (pos_a, rot_a, inv_mass_a) = match self.bodies.get(body_ah(*body_a)) {
                Some(b) => (b.position, b.rotation, b.inv_mass),
                None => continue,
            };
            let (pos_b, rot_b, inv_mass_b) = match self.bodies.get(body_ah(*body_b)) {
                Some(b) => (b.position, b.rotation, b.inv_mass),
                None => continue,
            };

            let w_sum = inv_mass_a + inv_mass_b;
            if w_sum < EPSILON {
                continue;
            }

            // World-space anchor positions
            let (sin_a, cos_a) = rot_a.sin_cos();
            let (sin_b, cos_b) = rot_b.sin_cos();
            let wa = [
                pos_a[0] + cos_a * anchor_a[0] - sin_a * anchor_a[1],
                pos_a[1] + sin_a * anchor_a[0] + cos_a * anchor_a[1],
            ];
            let wb = [
                pos_b[0] + cos_b * anchor_b[0] - sin_b * anchor_b[1],
                pos_b[1] + sin_b * anchor_b[0] + cos_b * anchor_b[1],
            ];

            match joint_type {
                JointType::Fixed | JointType::Revolute { .. } => {
                    // Distance constraint: anchors should coincide
                    let dx = wb[0] - wa[0];
                    let dy = wb[1] - wa[1];
                    let dist = (dx * dx + dy * dy).sqrt();
                    if dist < EPSILON {
                        continue;
                    }
                    let n = [dx / dist, dy / dist];
                    let c = dist;
                    let denom = w_sum + alpha_tilde;
                    let delta_lambda = -c / denom;

                    if let Some(ba) = self.bodies.get_mut(body_ah(*body_a))
                        && ba.is_dynamic()
                    {
                        ba.position[0] -= delta_lambda * inv_mass_a * n[0];
                        ba.position[1] -= delta_lambda * inv_mass_a * n[1];
                    }
                    if let Some(bb) = self.bodies.get_mut(body_ah(*body_b))
                        && bb.is_dynamic()
                    {
                        bb.position[0] += delta_lambda * inv_mass_b * n[0];
                        bb.position[1] += delta_lambda * inv_mass_b * n[1];
                    }
                }
                JointType::Distance { length } => {
                    let dx = wb[0] - wa[0];
                    let dy = wb[1] - wa[1];
                    let dist = (dx * dx + dy * dy).sqrt();
                    if dist < EPSILON {
                        continue;
                    }
                    let n = [dx / dist, dy / dist];
                    let c = dist - length;
                    let denom = w_sum + alpha_tilde;
                    let delta_lambda = -c / denom;

                    if let Some(ba) = self.bodies.get_mut(body_ah(*body_a))
                        && ba.is_dynamic()
                    {
                        ba.position[0] -= delta_lambda * inv_mass_a * n[0];
                        ba.position[1] -= delta_lambda * inv_mass_a * n[1];
                    }
                    if let Some(bb) = self.bodies.get_mut(body_ah(*body_b))
                        && bb.is_dynamic()
                    {
                        bb.position[0] += delta_lambda * inv_mass_b * n[0];
                        bb.position[1] += delta_lambda * inv_mass_b * n[1];
                    }
                }
                JointType::Spring {
                    rest_length,
                    stiffness,
                    damping: _,
                } => {
                    let dx = wb[0] - wa[0];
                    let dy = wb[1] - wa[1];
                    let dist = (dx * dx + dy * dy).sqrt();
                    if dist < EPSILON {
                        continue;
                    }
                    let n = [dx / dist, dy / dist];
                    let c = dist - rest_length;
                    // Spring compliance: α̃ = 1/(k·dt²)
                    let spring_alpha = if *stiffness > EPSILON && dt > EPSILON {
                        1.0 / (stiffness * dt * dt)
                    } else {
                        alpha_tilde
                    };
                    let denom = w_sum + spring_alpha;
                    let delta_lambda = -c / denom;

                    if let Some(ba) = self.bodies.get_mut(body_ah(*body_a))
                        && ba.is_dynamic()
                    {
                        ba.position[0] -= delta_lambda * inv_mass_a * n[0];
                        ba.position[1] -= delta_lambda * inv_mass_a * n[1];
                    }
                    if let Some(bb) = self.bodies.get_mut(body_ah(*body_b))
                        && bb.is_dynamic()
                    {
                        bb.position[0] += delta_lambda * inv_mass_b * n[0];
                        bb.position[1] += delta_lambda * inv_mass_b * n[1];
                    }
                }
                JointType::Rope { max_length } => {
                    let dx = wb[0] - wa[0];
                    let dy = wb[1] - wa[1];
                    let dist = (dx * dx + dy * dy).sqrt();
                    if dist <= *max_length || dist < EPSILON {
                        continue;
                    }
                    let n = [dx / dist, dy / dist];
                    let c = dist - max_length;
                    let denom = w_sum + alpha_tilde;
                    let delta_lambda = -c / denom;

                    if let Some(ba) = self.bodies.get_mut(body_ah(*body_a))
                        && ba.is_dynamic()
                    {
                        ba.position[0] -= delta_lambda * inv_mass_a * n[0];
                        ba.position[1] -= delta_lambda * inv_mass_a * n[1];
                    }
                    if let Some(bb) = self.bodies.get_mut(body_ah(*body_b))
                        && bb.is_dynamic()
                    {
                        bb.position[0] += delta_lambda * inv_mass_b * n[0];
                        bb.position[1] += delta_lambda * inv_mass_b * n[1];
                    }
                }
                _ => {
                    // Other joint types: fall through (Mouse, Prismatic, Wheel handled later)
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // XPBD velocity-level friction + restitution
    // -----------------------------------------------------------------------

    /// After XPBD position solve and velocity derivation, apply velocity-level
    /// friction and restitution corrections.
    fn xpbd_velocity_solve(&mut self, keys: &[ManifoldKey]) {
        use crate::material::CombineRule;

        fn combine_property(a: f64, b: f64, rule_a: CombineRule, rule_b: CombineRule) -> f64 {
            let rule = rule_a.max(rule_b);
            rule.combine(a, b)
        }

        for key in keys {
            let manifold = match self.manifolds.get(key) {
                Some(m) => m,
                None => continue,
            };

            let (restitution, friction) = match (
                self.colliders.get(coll_ah(manifold.collider_a)),
                self.colliders.get(coll_ah(manifold.collider_b)),
            ) {
                (Some(a), Some(b)) => {
                    if a.is_sensor || b.is_sensor {
                        continue;
                    }
                    (
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
                    )
                }
                _ => continue,
            };

            let normal = manifold.normal;
            let body_a = manifold.body_a;
            let body_b = manifold.body_b;

            let (vel_a, inv_mass_a) = match self.bodies.get(body_ah(body_a)) {
                Some(b) => (b.linear_velocity, b.inv_mass),
                None => continue,
            };
            let (vel_b, inv_mass_b) = match self.bodies.get(body_ah(body_b)) {
                Some(b) => (b.linear_velocity, b.inv_mass),
                None => continue,
            };

            let w_sum = inv_mass_a + inv_mass_b;
            if w_sum < EPSILON {
                continue;
            }

            // Relative velocity at contact
            let rel_vel = [vel_b[0] - vel_a[0], vel_b[1] - vel_a[1]];
            let vn = rel_vel[0] * normal[0] + rel_vel[1] * normal[1];

            // Restitution: if bodies are approaching along normal, apply bounce.
            // Uses the derived velocity (from position delta) which already
            // reflects the XPBD position correction.
            if vn < -EPSILON {
                let delta_vn = -vn * (1.0 + restitution);
                let j = delta_vn / w_sum;

                if let Some(ba) = self.bodies.get_mut(body_ah(body_a))
                    && ba.is_dynamic()
                {
                    ba.linear_velocity[0] -= j * inv_mass_a * normal[0];
                    ba.linear_velocity[1] -= j * inv_mass_a * normal[1];
                }
                if let Some(bb) = self.bodies.get_mut(body_ah(body_b))
                    && bb.is_dynamic()
                {
                    bb.linear_velocity[0] += j * inv_mass_b * normal[0];
                    bb.linear_velocity[1] += j * inv_mass_b * normal[1];
                }
            }

            // Friction: tangential velocity damping
            if friction > EPSILON {
                // Re-read velocities after restitution
                let vel_a = self
                    .bodies
                    .get(body_ah(body_a))
                    .map(|b| b.linear_velocity)
                    .unwrap_or([0.0, 0.0]);
                let vel_b = self
                    .bodies
                    .get(body_ah(body_b))
                    .map(|b| b.linear_velocity)
                    .unwrap_or([0.0, 0.0]);
                let rel_vel = [vel_b[0] - vel_a[0], vel_b[1] - vel_a[1]];
                let vn_after = rel_vel[0] * normal[0] + rel_vel[1] * normal[1];
                let tangent_vel = [
                    rel_vel[0] - vn_after * normal[0],
                    rel_vel[1] - vn_after * normal[1],
                ];
                let vt = (tangent_vel[0] * tangent_vel[0] + tangent_vel[1] * tangent_vel[1]).sqrt();

                if vt > EPSILON {
                    let tangent = [tangent_vel[0] / vt, tangent_vel[1] / vt];
                    // Coulomb friction: clamp tangential impulse by μ * normal impulse
                    // For XPBD, use a simple friction model: remove tangential velocity
                    // scaled by friction coefficient
                    let delta_vt = (-vt * friction).max(-vt); // don't reverse direction
                    let j = delta_vt / w_sum;

                    if let Some(ba) = self.bodies.get_mut(body_ah(body_a))
                        && ba.is_dynamic()
                    {
                        ba.linear_velocity[0] -= j * inv_mass_a * tangent[0];
                        ba.linear_velocity[1] -= j * inv_mass_a * tangent[1];
                    }
                    if let Some(bb) = self.bodies.get_mut(body_ah(body_b))
                        && bb.is_dynamic()
                    {
                        bb.linear_velocity[0] += j * inv_mass_b * tangent[0];
                        bb.linear_velocity[1] += j * inv_mass_b * tangent[1];
                    }
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Collision event generation
    // -----------------------------------------------------------------------

    // -----------------------------------------------------------------------
    // Simulation islands — union-find based grouping and island-level sleep
    // -----------------------------------------------------------------------

    fn build_islands_and_sleep(&mut self, dt: f64) {
        use std::collections::BTreeMap as Map;

        // Collect all body handles and assign indices
        let body_handles: Vec<crate::body::BodyHandle> =
            self.bodies.iter().map(|(_, b)| b.handle).collect();
        let body_count = body_handles.len();
        if body_count == 0 {
            return;
        }

        // Build handle -> index map
        let mut handle_to_idx: Map<crate::body::BodyHandle, u32> = Map::new();
        for (i, &h) in body_handles.iter().enumerate() {
            handle_to_idx.insert(h, i as u32);
        }

        // Reset island manager
        self.island_manager.reset(body_count);

        // Union dynamic bodies connected by contacts or joints. Static and
        // kinematic bodies don't propagate islands — otherwise everything on
        // a shared ground would be one island and nothing could sleep while
        // any of it moved.
        let is_dynamic = |h: &crate::body::BodyHandle| {
            self.bodies.get(body_ah(*h)).is_some_and(|b| b.is_dynamic())
        };
        let mut links: Vec<(u32, u32)> = Vec::new();
        let pairs = self
            .manifolds
            .values()
            .map(|m| (m.body_a, m.body_b))
            .chain(self.joints.iter().map(|(_, j)| (j.body_a, j.body_b)));
        for (a, b) in pairs {
            if !is_dynamic(&a) || !is_dynamic(&b) {
                continue;
            }
            if let (Some(&ia), Some(&ib)) = (handle_to_idx.get(&a), handle_to_idx.get(&b)) {
                links.push((ia, ib));
            }
        }
        for (ia, ib) in links {
            self.island_manager.union(ia, ib);
        }

        // Assign island IDs and collect per-island info
        // island_root -> list of body indices in that island
        let mut islands: Map<u32, Vec<u32>> = Map::new();
        for i in 0..body_count {
            let root = self.island_manager.find(i as u32);
            islands.entry(root).or_default().push(i as u32);
        }

        // For each island, check if ALL dynamic bodies are below velocity threshold
        for members in islands.values() {
            let mut all_slow = true;
            let mut has_dynamic = false;

            for &idx in members {
                let handle = body_handles[idx as usize];
                let rb = match self.bodies.get(body_ah(handle)) {
                    Some(b) => b,
                    None => continue,
                };
                if !rb.is_dynamic() || rb.inv_mass == 0.0 {
                    continue;
                }
                has_dynamic = true;
                let lin_speed = (rb.linear_velocity[0] * rb.linear_velocity[0]
                    + rb.linear_velocity[1] * rb.linear_velocity[1])
                    .sqrt();
                let ang_speed = rb.angular_velocity.abs();
                if lin_speed >= SLEEP_VELOCITY_THRESHOLD || ang_speed >= SLEEP_VELOCITY_THRESHOLD {
                    all_slow = false;
                    break;
                }
            }

            if !has_dynamic {
                continue;
            }

            // Update sleep timers for the whole island atomically: the island
            // sleeps only once its most recently active member has rested
            // long enough, and any fast member wakes all of it.
            let mut min_timer = f64::INFINITY;
            for &idx in members {
                let island = self.island_manager.find(idx);
                let Some(rb) = self.bodies.get_mut(body_ah(body_handles[idx as usize])) else {
                    continue;
                };
                if !rb.is_dynamic() || rb.inv_mass == 0.0 {
                    continue;
                }
                rb.island_id = island;
                if all_slow {
                    rb.sleep_timer += dt;
                    min_timer = min_timer.min(rb.sleep_timer);
                } else {
                    rb.sleep_timer = 0.0;
                    rb.is_sleeping = false;
                }
            }
            if all_slow && min_timer >= SLEEP_TIME_THRESHOLD {
                for &idx in members {
                    if let Some(rb) = self.bodies.get_mut(body_ah(body_handles[idx as usize]))
                        && rb.is_dynamic()
                        && !rb.is_sleeping
                    {
                        // Sleeping bodies must not keep drifting on residual velocity.
                        rb.is_sleeping = true;
                        rb.linear_velocity = [0.0, 0.0];
                        rb.angular_velocity = 0.0;
                    }
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Solver helpers
// ---------------------------------------------------------------------------

#[inline]
fn cross2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

#[inline]
fn dot2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

#[inline]
fn sub2(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

/// Working copy of a body's (pseudo-)velocity during a constraint solve.
#[derive(Clone, Copy)]
struct SolverVel {
    v: [f64; 2],
    w: f64,
}

impl SolverVel {
    #[inline]
    fn of(b: &RigidBody2d) -> Self {
        Self {
            v: b.linear_velocity,
            w: b.angular_velocity,
        }
    }

    #[inline]
    fn pseudo_of(b: &RigidBody2d) -> Self {
        Self {
            v: b.pseudo_velocity,
            w: b.pseudo_angular_velocity,
        }
    }

    /// Velocity of the point at offset `r` from the body centre.
    #[inline]
    fn at(&self, r: [f64; 2]) -> [f64; 2] {
        [self.v[0] - self.w * r[1], self.v[1] + self.w * r[0]]
    }

    /// Apply `sign · impulse` at offset `r`.
    #[inline]
    fn apply(
        &mut self,
        sign: f64,
        impulse: [f64; 2],
        r: [f64; 2],
        inv_mass: f64,
        inv_inertia: f64,
    ) {
        self.v[0] += sign * impulse[0] * inv_mass;
        self.v[1] += sign * impulse[1] * inv_mass;
        self.w += sign * cross2(r, impulse) * inv_inertia;
    }

    #[inline]
    fn store(self, b: &mut RigidBody2d) {
        b.linear_velocity = self.v;
        b.angular_velocity = self.w;
    }

    #[inline]
    fn store_pseudo(self, b: &mut RigidBody2d) {
        b.pseudo_velocity = self.v;
        b.pseudo_angular_velocity = self.w;
    }
}
