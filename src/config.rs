//! World configuration — timestep, gravity, solver settings.

use serde::{Deserialize, Serialize};

/// Physics world configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WorldConfig {
    /// Fixed timestep in seconds (default: 1/60).
    pub timestep: f64,
    /// Gravity (default: [0, -9.81] for 2D).
    pub gravity: [f64; 2],
    /// Solver velocity iterations (default: 4).
    pub velocity_iterations: u32,
    /// Solver position iterations (default: 1).
    pub position_iterations: u32,
    /// Enable deterministic mode (default: true).
    pub deterministic: bool,
    /// Simulation step counter.
    pub step: u64,
}

impl Default for WorldConfig {
    fn default() -> Self {
        Self {
            timestep: 1.0 / 60.0,
            gravity: [0.0, -9.81],
            velocity_iterations: 4,
            position_iterations: 1,
            deterministic: true,
            step: 0,
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
        assert_eq!(config.gravity, [0.0, -9.81]);
        assert!(config.deterministic);
    }

    #[test]
    fn config_serde_roundtrip() {
        let config = WorldConfig::default();
        let json = serde_json::to_string(&config).unwrap();
        let back: WorldConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(config.timestep, back.timestep);
    }
}
