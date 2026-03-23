//! Native 3D physics backend.
//!
//! Implements broadphase (spatial hash), narrowphase (shape-vs-shape contact
//! generation), and a sequential impulse constraint solver with friction and
//! angular response. All geometry uses f64 precision.

mod joints;
mod narrowphase;
mod raycast;
mod solver;
mod state;
#[cfg(test)]
mod tests;
mod types;

use crate::arena::ArenaHandle;
use crate::body::BodyHandle;
use crate::collider::ColliderHandle;
use crate::joint::JointHandle;

pub(crate) use state::PhysicsState3d;

// ---------------------------------------------------------------------------
// Handle <-> ArenaHandle conversions (zero-cost -- same u64 layout)
// ---------------------------------------------------------------------------

#[inline(always)]
pub(super) fn body_ah(h: BodyHandle) -> ArenaHandle {
    ArenaHandle(h.0)
}
#[inline(always)]
pub(super) fn body_from(ah: ArenaHandle) -> BodyHandle {
    BodyHandle(ah.0)
}
#[inline(always)]
pub(super) fn coll_ah(h: ColliderHandle) -> ArenaHandle {
    ArenaHandle(h.0)
}
#[inline(always)]
pub(super) fn coll_from(ah: ArenaHandle) -> ColliderHandle {
    ColliderHandle(ah.0)
}
#[inline(always)]
#[allow(dead_code)]
pub(super) fn joint_ah(h: JointHandle) -> ArenaHandle {
    ArenaHandle(h.0)
}
#[inline(always)]
pub(super) fn joint_from(ah: ArenaHandle) -> JointHandle {
    JointHandle(ah.0)
}
