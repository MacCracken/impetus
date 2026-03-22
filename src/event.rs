//! Collision events and contact data.

use crate::collider::ColliderHandle;
use serde::{Deserialize, Serialize};

/// A collision event from the physics step.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum CollisionEvent {
    /// Two colliders started touching.
    Started {
        collider_a: ColliderHandle,
        collider_b: ColliderHandle,
    },
    /// Two colliders stopped touching.
    Stopped {
        collider_a: ColliderHandle,
        collider_b: ColliderHandle,
    },
}

/// Detailed contact data for a collision pair.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContactData {
    pub collider_a: ColliderHandle,
    pub collider_b: ColliderHandle,
    pub normal: [f64; 2],
    pub depth: f64,
    pub points: Vec<ContactPoint>,
}

/// A single contact point.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ContactPoint {
    pub local_a: [f64; 2],
    pub local_b: [f64; 2],
    pub impulse: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collision_event_serde() {
        let event = CollisionEvent::Started {
            collider_a: ColliderHandle(0),
            collider_b: ColliderHandle(1),
        };
        let json = serde_json::to_string(&event).unwrap();
        let back: CollisionEvent = serde_json::from_str(&json).unwrap();
        assert!(matches!(back, CollisionEvent::Started { .. }));
    }
}
