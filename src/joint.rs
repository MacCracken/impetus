//! Joints and constraints — fixed, revolute, prismatic, spring.

use crate::body::BodyHandle;
use serde::{Deserialize, Serialize};

/// Unique handle to a joint.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
pub struct JointHandle(pub u64);

/// Joint type.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum JointType {
    /// Fixed joint — bodies maintain relative position/rotation.
    Fixed,
    /// Revolute — bodies rotate around a shared anchor point.
    Revolute {
        anchor: [f64; 2],
        limits: Option<[f64; 2]>,
    },
    /// Prismatic — bodies slide along an axis.
    Prismatic {
        axis: [f64; 2],
        limits: Option<[f64; 2]>,
    },
    /// Spring — damped spring between two points.
    Spring {
        rest_length: f64,
        stiffness: f64,
        damping: f64,
    },
    /// Distance — maintains fixed distance between anchors.
    Distance { length: f64 },
}

/// Motor parameters for revolute and prismatic joints.
///
/// A motor drives the joint toward a target velocity, applying up to
/// `max_force` (torque for revolute, force for prismatic) each step.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointMotor {
    /// Target velocity (rad/s for revolute, m/s for prismatic).
    pub target_velocity: f64,
    /// Maximum force/torque the motor can apply per step.
    pub max_force: f64,
}

/// Descriptor for creating a joint.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JointDesc {
    pub body_a: BodyHandle,
    pub body_b: BodyHandle,
    pub joint_type: JointType,
    pub local_anchor_a: [f64; 2],
    pub local_anchor_b: [f64; 2],
    /// Optional motor (only used for Revolute and Prismatic joints).
    #[serde(default)]
    pub motor: Option<JointMotor>,
    /// Velocity damping applied to relative motion at anchor points.
    /// For Spring joints, use the Spring variant's own `damping` field instead.
    #[serde(default)]
    pub damping: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_joint_serde() {
        let desc = JointDesc {
            body_a: BodyHandle(0),
            body_b: BodyHandle(1),
            joint_type: JointType::Fixed,
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [1.0, 0.0],
            motor: None,
            damping: 0.0,
        };
        let json = serde_json::to_string(&desc).unwrap();
        let back: JointDesc = serde_json::from_str(&json).unwrap();
        assert_eq!(desc, back);
    }

    #[test]
    fn spring_joint_serde() {
        let desc = JointDesc {
            body_a: BodyHandle(0),
            body_b: BodyHandle(1),
            joint_type: JointType::Spring {
                rest_length: 2.0,
                stiffness: 100.0,
                damping: 5.0,
            },
            local_anchor_a: [0.0, 0.0],
            local_anchor_b: [0.0, 0.0],
            motor: None,
            damping: 0.0,
        };
        let json = serde_json::to_string(&desc).unwrap();
        let back: JointDesc = serde_json::from_str(&json).unwrap();
        assert_eq!(desc, back);
    }

    #[test]
    fn revolute_with_limits() {
        let jt = JointType::Revolute {
            anchor: [1.0, 0.0],
            limits: Some([-1.57, 1.57]),
        };
        let json = serde_json::to_string(&jt).unwrap();
        let back: JointType = serde_json::from_str(&json).unwrap();
        assert_eq!(jt, back);
    }

    #[test]
    fn revolute_without_limits() {
        let jt = JointType::Revolute {
            anchor: [0.0, 0.0],
            limits: None,
        };
        let json = serde_json::to_string(&jt).unwrap();
        let back: JointType = serde_json::from_str(&json).unwrap();
        assert_eq!(jt, back);
    }

    #[test]
    fn prismatic_joint_serde() {
        let jt = JointType::Prismatic {
            axis: [1.0, 0.0],
            limits: Some([-5.0, 5.0]),
        };
        let json = serde_json::to_string(&jt).unwrap();
        let back: JointType = serde_json::from_str(&json).unwrap();
        assert_eq!(jt, back);
    }

    #[test]
    fn distance_joint_serde() {
        let jt = JointType::Distance { length: 3.0 };
        let json = serde_json::to_string(&jt).unwrap();
        let back: JointType = serde_json::from_str(&json).unwrap();
        assert_eq!(jt, back);
    }

    #[test]
    fn joint_handle_eq() {
        assert_eq!(JointHandle(0), JointHandle(0));
        assert_ne!(JointHandle(0), JointHandle(1));
    }
}
