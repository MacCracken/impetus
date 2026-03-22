//! Collision events and contact data.

use crate::collider::ColliderHandle;
use serde::{Deserialize, Serialize};

/// A collision event from the physics step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactData {
    pub collider_a: ColliderHandle,
    pub collider_b: ColliderHandle,
    pub normal: [f64; 2],
    pub depth: f64,
    pub points: Vec<ContactPoint>,
}

/// A single contact point.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ContactPoint {
    pub local_a: [f64; 2],
    pub local_b: [f64; 2],
    pub impulse: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collision_started_serde() {
        let event = CollisionEvent::Started {
            collider_a: ColliderHandle(0),
            collider_b: ColliderHandle(1),
        };
        let json = serde_json::to_string(&event).unwrap();
        let back: CollisionEvent = serde_json::from_str(&json).unwrap();
        assert_eq!(event, back);
    }

    #[test]
    fn collision_stopped_serde() {
        let event = CollisionEvent::Stopped {
            collider_a: ColliderHandle(3),
            collider_b: ColliderHandle(7),
        };
        let json = serde_json::to_string(&event).unwrap();
        let back: CollisionEvent = serde_json::from_str(&json).unwrap();
        assert_eq!(event, back);
    }

    #[test]
    fn contact_data_serde() {
        let data = ContactData {
            collider_a: ColliderHandle(0),
            collider_b: ColliderHandle(1),
            normal: [0.0, 1.0],
            depth: 0.01,
            points: vec![ContactPoint {
                local_a: [0.5, 0.0],
                local_b: [0.5, 1.0],
                impulse: 42.0,
            }],
        };
        let json = serde_json::to_string(&data).unwrap();
        let back: ContactData = serde_json::from_str(&json).unwrap();
        assert_eq!(data, back);
    }

    #[test]
    fn contact_data_empty_points() {
        let data = ContactData {
            collider_a: ColliderHandle(0),
            collider_b: ColliderHandle(1),
            normal: [1.0, 0.0],
            depth: 0.0,
            points: vec![],
        };
        assert!(data.points.is_empty());
    }

    #[test]
    fn contact_point_serde() {
        let pt = ContactPoint {
            local_a: [1.0, 2.0],
            local_b: [3.0, 4.0],
            impulse: 10.5,
        };
        let json = serde_json::to_string(&pt).unwrap();
        let back: ContactPoint = serde_json::from_str(&json).unwrap();
        assert_eq!(pt, back);
    }
}
