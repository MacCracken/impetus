//! Spatial queries — raycast, shape cast, point queries.

use crate::collider::ColliderHandle;
use serde::{Deserialize, Serialize};

/// Result of a raycast query.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RayHit {
    pub collider: ColliderHandle,
    pub point: [f64; 2],
    pub normal: [f64; 2],
    pub distance: f64,
}

/// Result of a point query.
#[derive(Debug, Clone, Serialize, Deserialize)]
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
            point: [1.0, 2.0],
            normal: [0.0, 1.0],
            distance: 3.5,
        };
        let json = serde_json::to_string(&hit).unwrap();
        let back: RayHit = serde_json::from_str(&json).unwrap();
        assert_eq!(back.distance, 3.5);
    }
}
