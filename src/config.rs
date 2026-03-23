//! World configuration — timestep, gravity, solver settings.

use serde::{Deserialize, Serialize};

/// Physics world configuration.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct WorldConfig {
    /// Fixed timestep in seconds (default: 1/60).
    pub timestep: f64,
    /// Gravity (default: [0, -9.81, 0]).
    pub gravity: [f64; 3],
    /// Solver velocity iterations (default: 4).
    pub velocity_iterations: u32,
    /// Solver position iterations (default: 1).
    pub position_iterations: u32,
    /// Enable deterministic mode (default: true).
    pub deterministic: bool,
    /// Simulation step counter.
    pub step: u64,
    /// Baumgarte positional correction: allowed penetration before correction (default: 0.01).
    #[serde(default = "default_slop")]
    pub position_slop: f64,
    /// Baumgarte positional correction: fraction of penetration corrected per iteration (default: 0.2).
    #[serde(default = "default_correction")]
    pub position_correction: f64,
}

fn default_slop() -> f64 {
    0.01
}

fn default_correction() -> f64 {
    0.2
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            timestep: 1.0 / 60.0,
            gravity: [0.0, -9.81, 0.0],
            velocity_iterations: 4,
            position_iterations: 1,
            deterministic: true,
            step: 0,
            position_slop: default_slop(),
            position_correction: default_correction(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config() {
        let config = WorldConfig::default();
        assert_eq!(config.timestep, 1.0 / 60.0);
        assert_eq!(config.gravity, [0.0, -9.81, 0.0]);
        assert!(config.deterministic);
        assert_eq!(config.velocity_iterations, 4);
        assert_eq!(config.position_iterations, 1);
        assert_eq!(config.step, 0);
    }

    #[test]
    fn config_serde_roundtrip() {
        let config = WorldConfig::default();
        let json = serde_json::to_string(&config).unwrap();
        let back: WorldConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(config, back);
    }

    #[test]
    fn custom_config_serde() {
        let config = WorldConfig {
            timestep: 1.0 / 120.0,
            gravity: [0.0, -10.0, 0.0],
            velocity_iterations: 8,
            position_iterations: 3,
            deterministic: false,
            step: 0,
            position_slop: 0.02,
            position_correction: 0.3,
        };
        let json = serde_json::to_string(&config).unwrap();
        let back: WorldConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(config, back);
    }

    #[test]
    fn zero_gravity_config() {
        let config = WorldConfig {
            gravity: [0.0, 0.0, 0.0],
            ..Default::default()
        };
        assert_eq!(config.gravity, [0.0, 0.0, 0.0]);
    }
}
