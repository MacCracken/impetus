//! Native 2D physics backend.
//!
//! Implements broadphase (AABB overlap), narrowphase (shape-vs-shape contact
//! generation), and a sequential impulse constraint solver with friction and
//! angular response. All geometry uses f64 precision.

mod types;
mod state;
mod solver;
mod joints;
mod narrowphase;
mod raycast;
#[cfg(test)]
mod tests;

use crate::arena::ArenaHandle;
use crate::body::BodyHandle;
use crate::collider::ColliderHandle;
use crate::joint::JointHandle;

pub(crate) use state::PhysicsState2d;

// ---------------------------------------------------------------------------
// Handle <-> ArenaHandle conversions (zero-cost -- same u64 layout)
// ---------------------------------------------------------------------------

#[inline(always)]
fn body_ah(h: BodyHandle) -> ArenaHandle { ArenaHandle(h.0) }
#[inline(always)]
fn body_from(ah: ArenaHandle) -> BodyHandle { BodyHandle(ah.0) }
#[inline(always)]
fn coll_ah(h: ColliderHandle) -> ArenaHandle { ArenaHandle(h.0) }
#[inline(always)]
fn coll_from(ah: ArenaHandle) -> ColliderHandle { ColliderHandle(ah.0) }
#[inline(always)]
fn joint_ah(h: JointHandle) -> ArenaHandle { ArenaHandle(h.0) }
#[inline(always)]
fn joint_from(ah: ArenaHandle) -> JointHandle { JointHandle(ah.0) }
