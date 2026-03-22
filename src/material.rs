//! Physics materials — friction, restitution, density.

use serde::{Deserialize, Serialize};

/// Physics material properties.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhysicsMaterial {
    /// Coefficient of friction (0.0 = ice, 1.0+ = rubber).
    pub friction: f64,
    /// Coefficient of restitution / bounciness (0.0 = clay, 1.0 = superball).
    pub restitution: f64,
    /// Density in kg/m² (2D) or kg/m³ (3D). 0.0 = use shape default.
    pub density: f64,
}

impl Default for PhysicsMaterial {
    fn default() -> Self {
        Self {
            friction: 0.5,
            restitution: 0.0,
            density: 1.0,
        }
    }
}

/// Named material presets.
impl PhysicsMaterial {
    pub fn ice() -> Self {
        Self {
            friction: 0.05,
            restitution: 0.1,
            density: 0.9,
        }
    }
    pub fn rubber() -> Self {
        Self {
            friction: 1.0,
            restitution: 0.8,
            density: 1.1,
        }
    }
    pub fn wood() -> Self {
        Self {
            friction: 0.4,
            restitution: 0.2,
            density: 0.6,
        }
    }
    pub fn steel() -> Self {
        Self {
            friction: 0.6,
            restitution: 0.3,
            density: 7.8,
        }
    }
    pub fn bouncy() -> Self {
        Self {
            friction: 0.3,
            restitution: 1.0,
            density: 1.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_material() {
        let mat = PhysicsMaterial::default();
        assert_eq!(mat.friction, 0.5);
        assert_eq!(mat.restitution, 0.0);
        assert_eq!(mat.density, 1.0);
    }

    #[test]
    fn presets() {
        assert!(PhysicsMaterial::ice().friction < PhysicsMaterial::rubber().friction);
        assert!(PhysicsMaterial::bouncy().restitution > PhysicsMaterial::steel().restitution);
        assert!(PhysicsMaterial::steel().density > PhysicsMaterial::wood().density);
    }

    #[test]
    fn preset_values() {
        let ice = PhysicsMaterial::ice();
        assert_eq!(ice.friction, 0.05);
        assert_eq!(ice.restitution, 0.1);
        assert_eq!(ice.density, 0.9);

        let rubber = PhysicsMaterial::rubber();
        assert_eq!(rubber.friction, 1.0);
        assert_eq!(rubber.restitution, 0.8);
    }

    #[test]
    fn material_serde() {
        let mat = PhysicsMaterial::rubber();
        let json = serde_json::to_string(&mat).unwrap();
        let back: PhysicsMaterial = serde_json::from_str(&json).unwrap();
        assert_eq!(mat, back);
    }

    #[test]
    fn material_equality() {
        assert_eq!(PhysicsMaterial::steel(), PhysicsMaterial::steel());
        assert_ne!(PhysicsMaterial::ice(), PhysicsMaterial::rubber());
    }
}
