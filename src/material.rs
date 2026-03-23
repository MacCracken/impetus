//! Physics materials — friction, restitution, density.

use serde::{Deserialize, Serialize};

/// Rule for combining material properties between two colliding bodies.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[non_exhaustive]
pub enum CombineRule {
    /// Use the minimum of the two values.
    Min,
    /// Use the average of the two values.
    Average,
    /// Multiply the two values.
    Multiply,
    /// Use the maximum of the two values.
    Max,
}

impl Default for CombineRule {
    fn default() -> Self {
        Self::Average
    }
}

impl CombineRule {
    /// Combine two values using the given rule.
    pub fn combine(self, a: f64, b: f64) -> f64 {
        match self {
            Self::Average => (a + b) * 0.5,
            Self::Min => a.min(b),
            Self::Max => a.max(b),
            Self::Multiply => a * b,
        }
    }
}

/// Physics material properties.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PhysicsMaterial {
    /// Coefficient of friction (0.0 = ice, 1.0+ = rubber).
    pub friction: f64,
    /// Coefficient of restitution / bounciness (0.0 = clay, 1.0 = superball).
    pub restitution: f64,
    /// Density in kg/m² (2D) or kg/m³ (3D). 0.0 = use shape default.
    pub density: f64,
    /// Coefficient of static friction. If `None`, defaults to `friction * 1.5`.
    /// Static friction is higher than kinetic friction and determines the
    /// threshold force needed to start sliding.
    #[serde(default)]
    pub static_friction: Option<f64>,
    /// Coefficient of rolling friction (0.0 = no rolling resistance).
    #[serde(default)]
    pub rolling_friction: f64,
    /// Rule for combining friction between two colliding materials.
    /// When two materials collide, the higher-priority rule wins
    /// (Max > Multiply > Average > Min).
    #[serde(default)]
    pub friction_combine: CombineRule,
    /// Rule for combining restitution between two colliding materials.
    /// When two materials collide, the higher-priority rule wins.
    #[serde(default = "default_restitution_combine")]
    pub restitution_combine: CombineRule,
}

fn default_restitution_combine() -> CombineRule {
    CombineRule::Min
}

impl Default for PhysicsMaterial {
    fn default() -> Self {
        Self {
            friction: 0.5,
            restitution: 0.0,
            density: 1.0,
            static_friction: None,
            rolling_friction: 0.0,
            friction_combine: CombineRule::Average,
            restitution_combine: CombineRule::Min,
        }
    }
}

impl PhysicsMaterial {
    /// Returns the effective static friction coefficient.
    /// If `static_friction` is `None`, returns `friction * 1.5`.
    pub fn effective_static_friction(&self) -> f64 {
        self.static_friction.unwrap_or(self.friction * 1.5)
    }
}

/// Named material presets.
impl PhysicsMaterial {
    /// Low friction, low bounce surface (ice rink).
    pub fn ice() -> Self {
        Self {
            friction: 0.05,
            restitution: 0.1,
            density: 0.9,
            ..Default::default()
        }
    }
    /// High friction, bouncy surface (rubber).
    pub fn rubber() -> Self {
        Self {
            friction: 1.0,
            restitution: 0.8,
            density: 1.1,
            ..Default::default()
        }
    }
    /// Moderate friction, low density surface (wood).
    pub fn wood() -> Self {
        Self {
            friction: 0.4,
            restitution: 0.2,
            density: 0.6,
            ..Default::default()
        }
    }
    /// Moderate friction, high density surface (steel).
    pub fn steel() -> Self {
        Self {
            friction: 0.6,
            restitution: 0.3,
            density: 7.8,
            ..Default::default()
        }
    }
    /// Low friction, maximum bounce surface (super ball).
    pub fn bouncy() -> Self {
        Self {
            friction: 0.3,
            restitution: 1.0,
            density: 1.0,
            ..Default::default()
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

    #[test]
    fn default_combine_rules() {
        let mat = PhysicsMaterial::default();
        assert_eq!(mat.friction_combine, CombineRule::Average);
        assert_eq!(mat.restitution_combine, CombineRule::Min);
        assert_eq!(mat.rolling_friction, 0.0);
    }

    #[test]
    fn combine_rule_average() {
        assert!((CombineRule::Average.combine(0.4, 0.6) - 0.5).abs() < 1e-10);
    }

    #[test]
    fn combine_rule_min() {
        assert!((CombineRule::Min.combine(0.3, 0.7) - 0.3).abs() < 1e-10);
    }

    #[test]
    fn combine_rule_max() {
        assert!((CombineRule::Max.combine(0.3, 0.7) - 0.7).abs() < 1e-10);
    }

    #[test]
    fn combine_rule_multiply() {
        assert!((CombineRule::Multiply.combine(0.5, 0.4) - 0.2).abs() < 1e-10);
    }

    #[test]
    fn combine_rule_ordering() {
        // Max > Multiply > Average > Min
        assert!(CombineRule::Max > CombineRule::Multiply);
        assert!(CombineRule::Multiply > CombineRule::Average);
        assert!(CombineRule::Average > CombineRule::Min);
    }

    #[test]
    fn rolling_friction_serde() {
        let mat = PhysicsMaterial {
            rolling_friction: 0.01,
            ..Default::default()
        };
        let json = serde_json::to_string(&mat).unwrap();
        let back: PhysicsMaterial = serde_json::from_str(&json).unwrap();
        assert_eq!(mat, back);
        assert_eq!(back.rolling_friction, 0.01);
    }

    #[test]
    fn combine_rule_serde() {
        let mat = PhysicsMaterial {
            friction_combine: CombineRule::Max,
            restitution_combine: CombineRule::Multiply,
            ..Default::default()
        };
        let json = serde_json::to_string(&mat).unwrap();
        let back: PhysicsMaterial = serde_json::from_str(&json).unwrap();
        assert_eq!(back.friction_combine, CombineRule::Max);
        assert_eq!(back.restitution_combine, CombineRule::Multiply);
    }

    #[test]
    fn serde_defaults_for_new_fields() {
        // Old-style JSON without new fields should deserialize with defaults
        let json = r#"{"friction":0.5,"restitution":0.0,"density":1.0}"#;
        let mat: PhysicsMaterial = serde_json::from_str(json).unwrap();
        assert_eq!(mat.rolling_friction, 0.0);
        assert_eq!(mat.friction_combine, CombineRule::Average);
        assert_eq!(mat.restitution_combine, CombineRule::Min);
    }
}
