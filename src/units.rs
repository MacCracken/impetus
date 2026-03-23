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
#[non_exhaustive]
pub enum PhysicsUnit {
    Meters,
    Kilograms,
    Seconds,
    MetersPerSecond,
    MetersPerSecondSquared,
    Newtons,
    NewtonMeters,
    Joules,
    Radians,
    Degrees,
    RadiansPerSecond,
    KgPerCubicMeter,
    KgPerSquareMeter,
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
    pub fn meters_per_second(value: f64) -> Self {
        Self::new(value, PhysicsUnit::MetersPerSecond)
    }
    pub fn pascals(value: f64) -> Self {
        Self::new(value, PhysicsUnit::Pascals)
    }

    /// Convert degrees to radians.
    #[must_use]
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
    fn quantity_display_all_units() {
        let cases = vec![
            (Quantity::kilograms(1.0), "1 kg"),
            (Quantity::seconds(2.5), "2.5 s"),
            (Quantity::meters_per_second(10.0), "10 m/s"),
            (Quantity::new(9.81, PhysicsUnit::MetersPerSecondSquared), "9.81 m/s\u{b2}"),
            (Quantity::new(5.0, PhysicsUnit::NewtonMeters), "5 N\u{b7}m"),
            (Quantity::joules(100.0), "100 J"),
            (Quantity::radians(2.5), "2.5 rad"),
            (Quantity::new(1.0, PhysicsUnit::RadiansPerSecond), "1 rad/s"),
            (Quantity::new(1000.0, PhysicsUnit::KgPerCubicMeter), "1000 kg/m\u{b3}"),
            (Quantity::new(500.0, PhysicsUnit::KgPerSquareMeter), "500 kg/m\u{b2}"),
            (Quantity::pascals(101325.0), "101325 Pa"),
        ];
        for (q, expected) in cases {
            assert_eq!(q.to_string(), expected);
        }
    }

    #[test]
    fn degrees_to_radians() {
        let q = Quantity::degrees(180.0);
        assert!((q.to_radians() - std::f64::consts::PI).abs() < 1e-10);
    }

    #[test]
    fn degrees_to_radians_90() {
        let q = Quantity::degrees(90.0);
        assert!((q.to_radians() - std::f64::consts::FRAC_PI_2).abs() < 1e-10);
    }

    #[test]
    fn degrees_to_radians_zero() {
        let q = Quantity::degrees(0.0);
        assert_eq!(q.to_radians(), 0.0);
    }

    #[test]
    fn radians_passthrough() {
        let q = Quantity::radians(1.0);
        assert_eq!(q.to_radians(), 1.0);
    }

    #[test]
    fn non_angle_to_radians_passthrough() {
        let q = Quantity::meters(5.0);
        assert_eq!(q.to_radians(), 5.0);
    }

    #[test]
    fn quantity_serde() {
        let q = Quantity::newtons(42.0);
        let json = serde_json::to_string(&q).unwrap();
        let back: Quantity = serde_json::from_str(&json).unwrap();
        assert_eq!(back.value, 42.0);
        assert_eq!(back.unit, PhysicsUnit::Newtons);
    }

    #[test]
    fn physics_unit_equality() {
        assert_eq!(PhysicsUnit::Meters, PhysicsUnit::Meters);
        assert_ne!(PhysicsUnit::Meters, PhysicsUnit::Seconds);
    }
}
