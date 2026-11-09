//! Oracle hooks for the Cyrius port's golden-value generator.
//!
//! **Not public API.** This module exists only behind the non-default `oracle`
//! feature, is hidden from the docs, and carries no stability promise. It lets
//! `scripts/oracle/` (see `docs/development/impetus-port-plan.md`, M0 step 0)
//! read state that the public API does not expose:
//! - the full world state, for single-step parity rows;
//! - contact generation for one shape pair;
//! - broadphase pair sets.
//!
//! Everything here is read-only. Enabling the feature changes no simulation
//! behaviour.

use crate::body::BodyType;
use crate::collider::ColliderShape;
use crate::config::WorldConfig;
use crate::event::CollisionEvent;
use crate::joint::{JointMotor, JointType};
use crate::material::PhysicsMaterial;
use crate::particle::{ForceField, Particle, ParticleEmitter};

#[cfg(all(feature = "2d", not(feature = "3d")))]
pub use crate::backend_2d::oracle::{contacts_2d, shapes_touch_2d};
#[cfg(feature = "3d")]
pub use crate::backend_3d::oracle::contact_3d;

// ---------------------------------------------------------------------------
// Shared dump types
// ---------------------------------------------------------------------------

/// One slot of a generational arena.
#[derive(Debug, Clone, PartialEq)]
pub struct SlotDump {
    /// Whether the slot holds a value.
    pub occupied: bool,
    /// The slot's current generation.
    pub generation: u32,
    /// For a free slot, the next slot on the free list.
    pub next_free: Option<u32>,
}

/// A generational arena's allocation state, which decides the handles it
/// gives out next.
#[derive(Debug, Clone, PartialEq)]
pub struct ArenaDump {
    /// Every slot, by index.
    pub slots: Vec<SlotDump>,
    /// Head of the free list.
    pub free_head: Option<u32>,
    /// Whether the free list must be rebuilt before the next insert.
    pub free_list_stale: bool,
}

/// The frame-level contact-event bookkeeping. Pairs are `(collider, collider)`
/// handle words, sorted and unique.
#[derive(Debug, Clone, PartialEq)]
pub struct TrackerDump {
    /// Pairs touching at the end of the previous frame.
    pub prev: Vec<(u64, u64)>,
    /// Pairs touching at the end of the most recent sub-step.
    pub current: Vec<(u64, u64)>,
    /// Pairs touching at the end of an earlier sub-step this frame.
    pub seen: Vec<(u64, u64)>,
    /// Whether `current` holds a sub-step recorded this frame.
    pub recorded: bool,
}

/// A collider's stored description. Vectors are 2- or 3-wide by backend; the
/// 2D backend's `offset` has `z = 0`.
#[derive(Debug, Clone, PartialEq)]
pub struct ColliderDump {
    /// Handle word (index in the low 32 bits, generation in the high 32).
    pub handle: u64,
    /// Owning body's handle word.
    pub body: u64,
    /// Shape.
    pub shape: ColliderShape,
    /// Offset from the body origin, in the body frame.
    pub offset: [f64; 3],
    /// Material.
    pub material: PhysicsMaterial,
    /// Sensor flag.
    pub is_sensor: bool,
    /// Mass override.
    pub mass: Option<f64>,
    /// Collision layer bits.
    pub collision_layer: u32,
    /// Collision mask bits.
    pub collision_mask: u32,
}

/// The full world state at the moment of the dump.
#[derive(Debug, Clone, PartialEq)]
pub struct WorldDump {
    /// The world's configuration.
    pub config: WorldConfig,
    /// Next particle id.
    pub next_particle_id: u64,
    /// Next emitter id.
    pub next_emitter_id: u64,
    /// Events of the last step.
    pub collision_events: Vec<CollisionEvent>,
    /// Live particles, in storage order.
    pub particles: Vec<Particle>,
    /// Emitters, in storage order, with their spawn state.
    pub emitters: Vec<ParticleEmitter>,
    /// Force fields, in storage order.
    pub force_fields: Vec<ForceField>,
    /// The 2D backend's state.
    #[cfg(all(feature = "2d", not(feature = "3d")))]
    pub backend: Backend2dDump,
    /// The 3D backend's state.
    #[cfg(feature = "3d")]
    pub backend: Backend3dDump,
}

// ---------------------------------------------------------------------------
// 2D backend dump
// ---------------------------------------------------------------------------

/// A 2D rigid body, every stored field.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[derive(Debug, Clone, PartialEq)]
#[allow(missing_docs)]
pub struct Body2dDump {
    pub handle: u64,
    pub body_type: BodyType,
    pub position: [f64; 2],
    pub rotation: f64,
    pub linear_velocity: [f64; 2],
    pub angular_velocity: f64,
    pub linear_damping: f64,
    pub angular_damping: f64,
    pub fixed_rotation: bool,
    pub gravity_scale: f64,
    pub force_accumulator: [f64; 2],
    pub torque_accumulator: f64,
    pub mass: f64,
    pub inv_mass: f64,
    pub inertia: f64,
    pub inv_inertia: f64,
    pub pseudo_velocity: [f64; 2],
    pub pseudo_angular_velocity: f64,
    pub prev_position: [f64; 2],
    pub prev_rotation: f64,
    pub is_sleeping: bool,
    pub sleep_timer: f64,
    pub island_id: u32,
}

/// A 2D joint, every stored field.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[derive(Debug, Clone, PartialEq)]
#[allow(missing_docs)]
pub struct Joint2dDump {
    pub handle: u64,
    pub body_a: u64,
    pub body_b: u64,
    pub joint_type: JointType,
    pub local_anchor_a: [f64; 2],
    pub local_anchor_b: [f64; 2],
    pub motor: Option<JointMotor>,
    pub damping: f64,
    pub break_force: Option<f64>,
    pub reference_angle: f64,
}

/// A cached 2D manifold point with its warm-start impulses.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[derive(Debug, Clone, PartialEq)]
#[allow(missing_docs)]
pub struct ManifoldPoint2dDump {
    pub local_a: [f64; 2],
    pub local_b: [f64; 2],
    pub normal_impulse: f64,
    pub tangent_impulse: f64,
    pub depth: f64,
}

/// A persistent 2D contact manifold.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[derive(Debug, Clone, PartialEq)]
#[allow(missing_docs)]
pub struct Manifold2dDump {
    pub collider_a: u64,
    pub collider_b: u64,
    pub body_a: u64,
    pub body_b: u64,
    pub normal: [f64; 2],
    pub points: Vec<ManifoldPoint2dDump>,
    pub rolling_impulse: f64,
}

/// The 2D backend's state. Bodies, colliders and joints are in arena slot
/// order; manifolds in key order.
#[cfg(all(feature = "2d", not(feature = "3d")))]
#[derive(Debug, Clone, PartialEq)]
#[allow(missing_docs)]
pub struct Backend2dDump {
    pub body_arena: ArenaDump,
    pub bodies: Vec<Body2dDump>,
    pub collider_arena: ArenaDump,
    pub colliders: Vec<ColliderDump>,
    pub joint_arena: ArenaDump,
    pub joints: Vec<Joint2dDump>,
    /// Each body's colliders, in attachment order (it sets the mass sum order).
    pub body_colliders: Vec<(u64, Vec<u64>)>,
    pub manifolds: Vec<Manifold2dDump>,
    pub tracker: TrackerDump,
}

// ---------------------------------------------------------------------------
// 3D backend dump
// ---------------------------------------------------------------------------

/// A 3D rigid body, every stored field. Quaternions are `[x, y, z, w]`;
/// matrices are column-major `[[col0], [col1], [col2]]`.
#[cfg(feature = "3d")]
#[derive(Debug, Clone, PartialEq)]
#[allow(missing_docs)]
pub struct Body3dDump {
    pub handle: u64,
    pub body_type: BodyType,
    pub position: [f64; 3],
    pub rotation: [f64; 4],
    pub linear_velocity: [f64; 3],
    pub angular_velocity: [f64; 3],
    pub linear_damping: f64,
    pub angular_damping: f64,
    pub fixed_rotation: bool,
    pub gravity_scale: f64,
    pub force_accumulator: [f64; 3],
    pub torque_accumulator: [f64; 3],
    pub mass: f64,
    pub inv_mass: f64,
    pub inertia: [[f64; 3]; 3],
    pub inv_inertia: [[f64; 3]; 3],
    pub inv_inertia_world: [[f64; 3]; 3],
    pub pseudo_velocity: [f64; 3],
    pub pseudo_angular_velocity: [f64; 3],
    pub is_sleeping: bool,
    pub sleep_timer: f64,
}

/// A 3D joint, every stored field.
#[cfg(feature = "3d")]
#[derive(Debug, Clone, PartialEq)]
#[allow(missing_docs)]
pub struct Joint3dDump {
    pub handle: u64,
    pub body_a: u64,
    pub body_b: u64,
    pub joint_type: JointType,
    pub local_anchor_a: [f64; 3],
    pub local_anchor_b: [f64; 3],
    pub reference_rotation: [f64; 4],
    pub motor: Option<JointMotor>,
    pub damping: f64,
    pub break_force: Option<f64>,
}

/// The 3D backend's state, in arena slot order. The 3D backend keeps no
/// manifolds between steps.
#[cfg(feature = "3d")]
#[derive(Debug, Clone, PartialEq)]
#[allow(missing_docs)]
pub struct Backend3dDump {
    pub body_arena: ArenaDump,
    pub bodies: Vec<Body3dDump>,
    pub collider_arena: ArenaDump,
    pub colliders: Vec<ColliderDump>,
    pub joint_arena: ArenaDump,
    pub joints: Vec<Joint3dDump>,
    /// Each body's colliders, in attachment order (it sets the mass sum order).
    pub body_colliders: Vec<(u64, Vec<u64>)>,
    pub tracker: TrackerDump,
}

// ---------------------------------------------------------------------------
// Broadphase pair sets
// ---------------------------------------------------------------------------

fn sorted_pairs(set: std::collections::BTreeSet<(u32, u32)>) -> Vec<(u32, u32)> {
    set.into_iter().collect()
}

/// Candidate pairs the dynamic AABB tree reports (from its fattened leaves)
/// for 2D boxes inserted in order; keys are input indices.
#[must_use]
pub fn aabb_tree_pairs_2d(aabbs: &[([f64; 2], [f64; 2])]) -> Vec<(u32, u32)> {
    let mut tree = crate::aabb_tree::AabbTree::new();
    for (i, (min, max)) in (0u32..).zip(aabbs) {
        tree.insert(i, crate::aabb_tree::TreeAabb::from_2d(*min, *max));
    }
    sorted_pairs(tree.query_pairs())
}

/// Candidate pairs the dynamic AABB tree reports (from its fattened leaves)
/// for 3D boxes inserted in order; keys are input indices.
#[must_use]
pub fn aabb_tree_pairs_3d(aabbs: &[([f64; 3], [f64; 3])]) -> Vec<(u32, u32)> {
    let mut tree = crate::aabb_tree::AabbTree::new();
    for (i, (min, max)) in (0u32..).zip(aabbs) {
        tree.insert(i, crate::aabb_tree::TreeAabb::from_3d(*min, *max));
    }
    sorted_pairs(tree.query_pairs())
}

/// Candidate pairs the spatial hash reports (boxes sharing a cell) for 2D
/// boxes inserted in order; keys are input indices.
#[must_use]
pub fn spatial_hash_pairs_2d(cell_size: f64, aabbs: &[([f64; 2], [f64; 2])]) -> Vec<(u32, u32)> {
    let mut grid = crate::spatial_hash::SpatialHashGrid::new(cell_size);
    for (i, (min, max)) in (0u32..).zip(aabbs) {
        grid.insert_2d(i, *min, *max);
    }
    sorted_pairs(grid.query_pairs())
}

/// Candidate pairs the spatial hash reports (boxes sharing a cell) for 3D
/// boxes inserted in order; keys are input indices.
#[must_use]
pub fn spatial_hash_pairs_3d(cell_size: f64, aabbs: &[([f64; 3], [f64; 3])]) -> Vec<(u32, u32)> {
    let mut grid = crate::spatial_hash::SpatialHashGrid::new(cell_size);
    for (i, (min, max)) in (0u32..).zip(aabbs) {
        grid.insert_3d(i, *min, *max);
    }
    sorted_pairs(grid.query_pairs())
}

/// The spatial hash's automatic cell size for the given AABB extents.
#[must_use]
pub fn spatial_hash_auto_cell_size(max_dims: &[f64]) -> f64 {
    crate::spatial_hash::SpatialHashGrid::<u32>::auto_cell_size(
        max_dims.iter().copied(),
        max_dims.len(),
    )
}
