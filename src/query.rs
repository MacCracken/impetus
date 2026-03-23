//! Spatial queries — raycast, shape cast, point queries.

use crate::collider::ColliderHandle;
use serde::{Deserialize, Serialize};

/// Result of a raycast query.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RayHit {
    pub collider: ColliderHandle,
    pub point: [f64; 3],
    pub normal: [f64; 3],
    pub distance: f64,
}

/// Result of a point query.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PointQuery {
    pub collider: ColliderHandle,
    pub distance: f64,
    pub is_inside: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ray_hit_serde() {
        let hit = RayHit {
            collider: ColliderHandle(5),
            point: [1.0, 2.0, 0.0],
            normal: [0.0, 1.0, 0.0],
            distance: 3.5,
        };
        let json = serde_json::to_string(&hit).unwrap();
        let back: RayHit = serde_json::from_str(&json).unwrap();
        assert_eq!(hit, back);
    }

    #[test]
    fn point_query_inside() {
        let pq = PointQuery {
            collider: ColliderHandle(2),
            distance: 0.0,
            is_inside: true,
        };
        let json = serde_json::to_string(&pq).unwrap();
        let back: PointQuery = serde_json::from_str(&json).unwrap();
        assert_eq!(pq, back);
        assert!(back.is_inside);
    }

    #[test]
    fn point_query_outside() {
        let pq = PointQuery {
            collider: ColliderHandle(3),
            distance: 1.5,
            is_inside: false,
        };
        let json = serde_json::to_string(&pq).unwrap();
        let back: PointQuery = serde_json::from_str(&json).unwrap();
        assert_eq!(pq, back);
        assert!(!back.is_inside);
    }
}
