//! Rigid bodies — static, dynamic, kinematic.

use serde::{Deserialize, Serialize};

/// Unique handle to a rigid body.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct BodyHandle(pub u64);

/// Body type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum BodyType {
    /// Immovable, infinite mass.
    Static,
    /// Affected by forces and collisions.
    Dynamic,
    /// Moved by user code, affects dynamic bodies but isn't affected by them.
    Kinematic,
}

/// Descriptor for creating a rigid body.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyDesc {
    pub body_type: BodyType,
    pub position: [f64; 3],
    pub rotation: f64,
    #[serde(default)]
    pub linear_velocity: [f64; 3],
    #[serde(default)]
    pub angular_velocity: f64,
    #[serde(default)]
    pub linear_damping: f64,
    #[serde(default)]
    pub angular_damping: f64,
    #[serde(default)]
    pub fixed_rotation: bool,
    #[serde(default)]
    pub gravity_scale: Option<f64>,
}

impl Default for BodyDesc {
    fn default() -> Self {
        Self {
            body_type: BodyType::Dynamic,
            position: [0.0, 0.0, 0.0],
            rotation: 0.0,
            linear_velocity: [0.0, 0.0, 0.0],
            angular_velocity: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            fixed_rotation: false,
            gravity_scale: None,
        }
    }
}

/// Runtime state of a body (read from simulation).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BodyState {
    pub handle: BodyHandle,
    pub body_type: BodyType,
    pub position: [f64; 3],
    pub rotation: f64,
    pub linear_velocity: [f64; 3],
    pub angular_velocity: f64,
    pub is_sleeping: bool,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_body_desc() {
        let desc = BodyDesc::default();
        assert_eq!(desc.body_type, BodyType::Dynamic);
        assert_eq!(desc.position, [0.0, 0.0, 0.0]);
        assert_eq!(desc.rotation, 0.0);
        assert_eq!(desc.linear_damping, 0.0);
        assert_eq!(desc.angular_damping, 0.0);
        assert!(!desc.fixed_rotation);
        assert_eq!(desc.gravity_scale, None);
    }

    #[test]
    fn body_desc_serde() {
        let desc = BodyDesc {
            body_type: BodyType::Static,
            position: [5.0, 3.0, 0.0],
            rotation: 1.57,
            ..Default::default()
        };
        let json = serde_json::to_string(&desc).unwrap();
        let back: BodyDesc = serde_json::from_str(&json).unwrap();
        assert_eq!(desc, back);
    }

    #[test]
    fn body_desc_kinematic_serde() {
        let desc = BodyDesc {
            body_type: BodyType::Kinematic,
            position: [1.0, 2.0, 0.0],
            linear_velocity: [3.0, 4.0, 0.0],
            angular_velocity: 0.5,
            fixed_rotation: true,
            gravity_scale: Some(0.5),
            ..Default::default()
        };
        let json = serde_json::to_string(&desc).unwrap();
        let back: BodyDesc = serde_json::from_str(&json).unwrap();
        assert_eq!(desc, back);
    }

    #[test]
    fn body_handle_eq() {
        assert_eq!(BodyHandle(1), BodyHandle(1));
        assert_ne!(BodyHandle(1), BodyHandle(2));
    }

    #[test]
    fn body_state_serde() {
        let state = BodyState {
            handle: BodyHandle(42),
            body_type: BodyType::Dynamic,
            position: [1.0, 2.0, 0.0],
            rotation: 0.5,
            linear_velocity: [3.0, 4.0, 0.0],
            angular_velocity: 1.0,
            is_sleeping: false,
        };
        let json = serde_json::to_string(&state).unwrap();
        let back: BodyState = serde_json::from_str(&json).unwrap();
        assert_eq!(state, back);
    }

    #[test]
    fn body_state_sleeping() {
        let state = BodyState {
            handle: BodyHandle(0),
            body_type: BodyType::Static,
            position: [0.0, 0.0, 0.0],
            rotation: 0.0,
            linear_velocity: [0.0, 0.0, 0.0],
            angular_velocity: 0.0,
            is_sleeping: true,
        };
        assert!(state.is_sleeping);
    }
}
