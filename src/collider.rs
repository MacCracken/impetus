//! Collider shapes — box, sphere, capsule, convex, trimesh, heightfield.

use crate::material::PhysicsMaterial;
use serde::{Deserialize, Serialize};

/// Unique handle to a collider.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ColliderHandle(pub u64);

/// Collider shape.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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
    fn collider_shapes_serde() {
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
            let back: ColliderShape = serde_json::from_str(&json).unwrap();
            assert_eq!(shape, &back);
        }
    }

    #[test]
    fn convex_hull_serde() {
        let shape = ColliderShape::ConvexHull {
            points: vec![[0.0, 0.0], [1.0, 0.0], [0.5, 1.0]],
        };
        let json = serde_json::to_string(&shape).unwrap();
        let back: ColliderShape = serde_json::from_str(&json).unwrap();
        assert_eq!(shape, back);
    }

    #[test]
    fn trimesh_serde() {
        let shape = ColliderShape::TriMesh {
            vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            indices: vec![[0, 1, 2]],
        };
        let json = serde_json::to_string(&shape).unwrap();
        let back: ColliderShape = serde_json::from_str(&json).unwrap();
        assert_eq!(shape, back);
    }

    #[test]
    fn heightfield_serde() {
        let shape = ColliderShape::Heightfield {
            heights: vec![0.0, 1.0, 0.5, 2.0],
            scale: [1.0, 1.0],
        };
        let json = serde_json::to_string(&shape).unwrap();
        let back: ColliderShape = serde_json::from_str(&json).unwrap();
        assert_eq!(shape, back);
    }

    #[test]
    fn segment_serde() {
        let shape = ColliderShape::Segment {
            a: [0.0, 0.0],
            b: [5.0, 5.0],
        };
        let json = serde_json::to_string(&shape).unwrap();
        let back: ColliderShape = serde_json::from_str(&json).unwrap();
        assert_eq!(shape, back);
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

    #[test]
    fn collider_desc_serde() {
        let desc = ColliderDesc {
            shape: ColliderShape::Box {
                half_extents: [2.0, 3.0],
            },
            offset: [1.0, 1.0],
            material: PhysicsMaterial::steel(),
            is_sensor: false,
            mass: Some(10.0),
        };
        let json = serde_json::to_string(&desc).unwrap();
        let back: ColliderDesc = serde_json::from_str(&json).unwrap();
        assert_eq!(desc, back);
    }
}
