//! Forces, impulses, and torques.

use serde::{Deserialize, Serialize};

/// A continuous force applied over time.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Force {
    /// Force vector [x, y, z] in Newtons.
    pub vector: [f64; 3],
    /// Application point relative to body center (None = center of mass).
    pub point: Option<[f64; 3]>,
}

impl Force {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            vector: [x, y, z],
            point: None,
        }
    }

    pub fn at_point(x: f64, y: f64, z: f64, px: f64, py: f64, pz: f64) -> Self {
        Self {
            vector: [x, y, z],
            point: Some([px, py, pz]),
        }
    }

    /// Gravity force for a given mass.
    pub fn gravity(mass: f64, g: f64) -> Self {
        Self::new(0.0, -mass * g, 0.0)
    }

    /// Magnitude of the force.
    pub fn magnitude(&self) -> f64 {
        (self.vector[0].powi(2) + self.vector[1].powi(2) + self.vector[2].powi(2)).sqrt()
    }
}

/// An instantaneous impulse (changes velocity directly).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Impulse {
    /// Impulse vector [x, y, z] in Newton-seconds.
    pub vector: [f64; 3],
    /// Application point (None = center of mass).
    pub point: Option<[f64; 3]>,
}

impl Impulse {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            vector: [x, y, z],
            point: None,
        }
    }

    pub fn at_point(x: f64, y: f64, z: f64, px: f64, py: f64, pz: f64) -> Self {
        Self {
            vector: [x, y, z],
            point: Some([px, py, pz]),
        }
    }

    /// Magnitude of the impulse.
    pub fn magnitude(&self) -> f64 {
        (self.vector[0].powi(2) + self.vector[1].powi(2) + self.vector[2].powi(2)).sqrt()
    }
}

/// A torque (rotational force).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Torque {
    /// Torque in Newton-meters (positive = counter-clockwise).
    pub value: f64,
}

impl Torque {
    pub fn new(value: f64) -> Self {
        Self { value }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn force_magnitude() {
        let f = Force::new(3.0, 4.0, 0.0);
        assert!((f.magnitude() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn force_zero() {
        let f = Force::new(0.0, 0.0, 0.0);
        assert_eq!(f.magnitude(), 0.0);
        assert_eq!(f.point, None);
    }

    #[test]
    fn gravity_force() {
        let f = Force::gravity(10.0, 9.81);
        assert!((f.vector[1] - (-98.1)).abs() < 1e-10);
        assert_eq!(f.vector[0], 0.0);
        assert_eq!(f.vector[2], 0.0);
    }

    #[test]
    fn force_at_point() {
        let f = Force::at_point(1.0, 0.0, 0.0, 0.5, 0.5, 0.0);
        assert_eq!(f.point, Some([0.5, 0.5, 0.0]));
        assert_eq!(f.vector, [1.0, 0.0, 0.0]);
    }

    #[test]
    fn force_serde() {
        let f = Force::at_point(3.0, 4.0, 0.0, 1.0, 2.0, 0.0);
        let json = serde_json::to_string(&f).unwrap();
        let back: Force = serde_json::from_str(&json).unwrap();
        assert_eq!(f, back);
    }

    #[test]
    fn impulse_new() {
        let i = Impulse::new(5.0, 10.0, 0.0);
        assert_eq!(i.vector, [5.0, 10.0, 0.0]);
        assert_eq!(i.point, None);
    }

    #[test]
    fn impulse_at_point() {
        let i = Impulse::at_point(1.0, 2.0, 0.0, 0.5, 0.5, 0.0);
        assert_eq!(i.point, Some([0.5, 0.5, 0.0]));
    }

    #[test]
    fn impulse_magnitude() {
        let i = Impulse::new(3.0, 4.0, 0.0);
        assert!((i.magnitude() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn impulse_serde() {
        let i = Impulse::at_point(1.0, 2.0, 0.0, 3.0, 4.0, 0.0);
        let json = serde_json::to_string(&i).unwrap();
        let back: Impulse = serde_json::from_str(&json).unwrap();
        assert_eq!(i, back);
    }

    #[test]
    fn torque_new() {
        let t = Torque::new(5.0);
        assert_eq!(t.value, 5.0);
    }

    #[test]
    fn torque_serde() {
        let t = Torque::new(-3.5);
        let json = serde_json::to_string(&t).unwrap();
        let back: Torque = serde_json::from_str(&json).unwrap();
        assert_eq!(t, back);
    }
}
