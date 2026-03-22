//! Unit-aware physics quantities.
//!
//! Bridges abaco-core's UnitCategory with physics quantities so forces are in
//! Newtons, distances in meters, masses in kilograms — not raw floats.

use serde::{Deserialize, Serialize};

/// A quantity with a unit, for type-safe physics.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Quantity {
    pub value: f64,
    pub unit: PhysicsUnit,
}

/// Physics-relevant units (subset of abaco-core UnitCategory).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PhysicsUnit {
    // Length
    Meters,
    // Mass
    Kilograms,
    // Time
    Seconds,
    // Velocity
    MetersPerSecond,
    // Acceleration
    MetersPerSecondSquared,
    // Force
    Newtons,
    // Torque
    NewtonMeters,
    // Energy
    Joules,
    // Angle
    Radians,
    Degrees,
    // Angular velocity
    RadiansPerSecond,
    // Density
    KgPerCubicMeter,
    KgPerSquareMeter,
    // Pressure
    Pascals,
}

impl Quantity {
    pub fn new(value: f64, unit: PhysicsUnit) -> Self {
        Self { value, unit }
    }

    pub fn meters(value: f64) -> Self {
        Self::new(value, PhysicsUnit::Meters)
    }
    pub fn kilograms(value: f64) -> Self {
        Self::new(value, PhysicsUnit::Kilograms)
    }
    pub fn seconds(value: f64) -> Self {
        Self::new(value, PhysicsUnit::Seconds)
    }
    pub fn newtons(value: f64) -> Self {
        Self::new(value, PhysicsUnit::Newtons)
    }
    pub fn joules(value: f64) -> Self {
        Self::new(value, PhysicsUnit::Joules)
    }
    pub fn radians(value: f64) -> Self {
        Self::new(value, PhysicsUnit::Radians)
    }
    pub fn degrees(value: f64) -> Self {
        Self::new(value, PhysicsUnit::Degrees)
    }

    /// Convert degrees to radians.
    pub fn to_radians(&self) -> f64 {
        match self.unit {
            PhysicsUnit::Degrees => self.value * std::f64::consts::PI / 180.0,
            PhysicsUnit::Radians => self.value,
            _ => self.value,
        }
    }
}

impl std::fmt::Display for Quantity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let unit_str = match self.unit {
            PhysicsUnit::Meters => "m",
            PhysicsUnit::Kilograms => "kg",
            PhysicsUnit::Seconds => "s",
            PhysicsUnit::MetersPerSecond => "m/s",
            PhysicsUnit::MetersPerSecondSquared => "m/s\u{b2}",
            PhysicsUnit::Newtons => "N",
            PhysicsUnit::NewtonMeters => "N\u{b7}m",
            PhysicsUnit::Joules => "J",
            PhysicsUnit::Radians => "rad",
            PhysicsUnit::Degrees => "\u{b0}",
            PhysicsUnit::RadiansPerSecond => "rad/s",
            PhysicsUnit::KgPerCubicMeter => "kg/m\u{b3}",
            PhysicsUnit::KgPerSquareMeter => "kg/m\u{b2}",
            PhysicsUnit::Pascals => "Pa",
        };
        write!(f, "{} {}", self.value, unit_str)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quantity_display() {
        assert_eq!(Quantity::newtons(9.81).to_string(), "9.81 N");
        assert_eq!(Quantity::meters(5.0).to_string(), "5 m");
        assert_eq!(Quantity::degrees(90.0).to_string(), "90 \u{b0}");
    }

    #[test]
    fn degrees_to_radians() {
        let q = Quantity::degrees(180.0);
        assert!((q.to_radians() - std::f64::consts::PI).abs() < 1e-10);
    }

    #[test]
    fn radians_passthrough() {
        let q = Quantity::radians(1.0);
        assert_eq!(q.to_radians(), 1.0);
    }

    #[test]
    fn quantity_serde() {
        let q = Quantity::newtons(42.0);
        let json = serde_json::to_string(&q).unwrap();
        let back: Quantity = serde_json::from_str(&json).unwrap();
        assert_eq!(back.value, 42.0);
        assert_eq!(back.unit, PhysicsUnit::Newtons);
    }
}
