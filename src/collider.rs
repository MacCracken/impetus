//! Collider shapes — box, sphere, capsule, convex, trimesh, heightfield.

use crate::material::PhysicsMaterial;
use serde::{Deserialize, Serialize};

/// Unique handle to a collider.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ColliderHandle(pub u64);

/// Collider shape.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ColliderShape {
    /// Axis-aligned box.
    Box { half_extents: [f64; 2] },
    /// Circle (2D) / Sphere (3D).
    Ball { radius: f64 },
    /// Capsule defined by half-height and radius.
    Capsule { half_height: f64, radius: f64 },
    /// Convex polygon from vertices.
    ConvexHull { points: Vec<[f64; 2]> },
    /// Triangle mesh (3D only).
    TriMesh {
        vertices: Vec<[f64; 3]>,
        indices: Vec<[u32; 3]>,
    },
    /// Heightfield (3D only).
    Heightfield {
        heights: Vec<f64>,
        scale: [f64; 2],
    },
    /// Line segment.
    Segment { a: [f64; 2], b: [f64; 2] },
}

/// Descriptor for creating a collider.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ColliderDesc {
    pub shape: ColliderShape,
    #[serde(default)]
    pub offset: [f64; 2],
    #[serde(default)]
    pub material: PhysicsMaterial,
    #[serde(default)]
    pub is_sensor: bool,
    #[serde(default)]
    pub mass: Option<f64>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collider_shapes() {
        let shapes = vec![
            ColliderShape::Box {
                half_extents: [1.0, 1.0],
            },
            ColliderShape::Ball { radius: 0.5 },
            ColliderShape::Capsule {
                half_height: 1.0,
                radius: 0.3,
            },
        ];
        for shape in &shapes {
            let json = serde_json::to_string(shape).unwrap();
            assert!(!json.is_empty());
        }
    }

    #[test]
    fn sensor_collider() {
        let desc = ColliderDesc {
            shape: ColliderShape::Ball { radius: 5.0 },
            offset: [0.0, 0.0],
            material: PhysicsMaterial::default(),
            is_sensor: true,
            mass: None,
        };
        assert!(desc.is_sensor);
    }
}
