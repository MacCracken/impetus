//! Forces and impulses.

use serde::{Deserialize, Serialize};

/// A continuous force applied over time.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Force {
    /// Force vector [x, y] in Newtons.
    pub vector: [f64; 2],
    /// Application point relative to body center (None = center of mass).
    pub point: Option<[f64; 2]>,
}

impl Force {
    pub fn new(x: f64, y: f64) -> Self {
        Self {
            vector: [x, y],
            point: None,
        }
    }

    pub fn at_point(x: f64, y: f64, px: f64, py: f64) -> Self {
        Self {
            vector: [x, y],
            point: Some([px, py]),
        }
    }

    /// Gravity force for a given mass.
    pub fn gravity(mass: f64, g: f64) -> Self {
        Self::new(0.0, -mass * g)
    }

    /// Magnitude of the force.
    pub fn magnitude(&self) -> f64 {
        (self.vector[0].powi(2) + self.vector[1].powi(2)).sqrt()
    }
}

/// An instantaneous impulse (changes velocity directly).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Impulse {
    /// Impulse vector [x, y] in Newton-seconds.
    pub vector: [f64; 2],
    /// Application point (None = center of mass).
    pub point: Option<[f64; 2]>,
}

impl Impulse {
    pub fn new(x: f64, y: f64) -> Self {
        Self {
            vector: [x, y],
            point: None,
        }
    }
}

/// A torque (rotational force).
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Torque {
    /// Torque in Newton-meters (positive = counter-clockwise).
    pub value: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn force_magnitude() {
        let f = Force::new(3.0, 4.0);
        assert!((f.magnitude() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn gravity_force() {
        let f = Force::gravity(10.0, 9.81);
        assert!((f.vector[1] - (-98.1)).abs() < 1e-10);
        assert_eq!(f.vector[0], 0.0);
    }

    #[test]
    fn force_at_point() {
        let f = Force::at_point(1.0, 0.0, 0.5, 0.5);
        assert!(f.point.is_some());
    }
}
