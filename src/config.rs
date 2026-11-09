//! World configuration — timestep, gravity, solver settings.

use serde::{Deserialize, Serialize};

/// Broadphase algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum BroadphaseKind {
    /// Spatial hash grid (default). Good for uniform-sized objects.
    SpatialHash,
    /// Dynamic AABB tree (BVH). Better for heterogeneous object sizes.
    AabbTree,
}

/// Constraint solver algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum SolverKind {
    /// Sequential impulse solver (default). Classic velocity-based constraint solver
    /// with warm starting and accumulated impulses.
    SequentialImpulse,
    /// Extended Position-Based Dynamics (XPBD). Position-based solver with compliant
    /// constraints.
    ///
    /// **2D only and experimental**: the 3D backend always uses
    /// `SequentialImpulse`. In 2D, XPBD supports contacts and the Fixed,
    /// Revolute, Distance, Spring and Rope joints; Prismatic, Wheel and Mouse
    /// joints, motors, limits and `break_force` have no effect under XPBD.
    Xpbd,
}

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
    /// Reserved; the simulation is always deterministic (default: true).
    pub deterministic: bool,
    /// Simulation step counter.
    pub step: u64,
    /// Baumgarte positional correction: allowed penetration before correction (default: 0.01).
    #[serde(default = "default_slop")]
    pub position_slop: f64,
    /// Baumgarte positional correction: fraction of penetration corrected per step
    /// when `constraint_frequency` is 0 (default: 0.2).
    #[serde(default = "default_correction")]
    pub position_correction: f64,
    /// Maximum velocity magnitude for CCD — clamps velocity to prevent tunneling (default: 100.0).
    #[serde(default = "default_max_velocity")]
    pub max_velocity: f64,
    /// Number of sub-steps per timestep for improved stability (default: 1).
    /// Higher values improve stacking stability at a cost of performance.
    /// Forces applied before `step()` act over the whole timestep, and
    /// collision events are reported once per `step()`. `0` is treated as 1.
    #[serde(default = "default_sub_steps")]
    pub sub_steps: u32,
    /// Soft constraint frequency in Hz (default: 30.0).
    /// Controls how stiff contact constraints are — higher values make them stiffer.
    /// Set to 0.0 to disable soft constraints and use Baumgarte correction.
    #[serde(default = "default_constraint_frequency")]
    pub constraint_frequency: f64,
    /// Soft constraint damping ratio (default: 1.0 = critically damped).
    /// Values < 1.0 are under-damped (bouncy), > 1.0 are over-damped.
    #[serde(default = "default_constraint_damping_ratio")]
    pub constraint_damping_ratio: f64,
    /// Broadphase algorithm (default: SpatialHash).
    #[serde(default = "default_broadphase")]
    pub broadphase: BroadphaseKind,
    /// Constraint solver algorithm (default: SequentialImpulse).
    #[serde(default = "default_solver")]
    pub solver: SolverKind,
}

fn default_slop() -> f64 {
    0.01
}

fn default_correction() -> f64 {
    0.2
}

fn default_max_velocity() -> f64 {
    100.0
}

fn default_sub_steps() -> u32 {
    1
}

fn default_constraint_frequency() -> f64 {
    30.0
}

fn default_constraint_damping_ratio() -> f64 {
    1.0
}

fn default_broadphase() -> BroadphaseKind {
    BroadphaseKind::SpatialHash
}

fn default_solver() -> SolverKind {
    SolverKind::SequentialImpulse
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
            max_velocity: default_max_velocity(),
            sub_steps: default_sub_steps(),
            constraint_frequency: default_constraint_frequency(),
            constraint_damping_ratio: default_constraint_damping_ratio(),
            broadphase: default_broadphase(),
            solver: default_solver(),
        }
    }
}

impl WorldConfig {
    /// Largest accepted `sub_steps`.
    pub const MAX_SUB_STEPS: u32 = 4096;
    /// Largest accepted `velocity_iterations` / `position_iterations`.
    pub const MAX_ITERATIONS: u32 = 10_000;

    /// Check that every setting is usable: a finite positive timestep,
    /// finite gravity, `sub_steps ≤ MAX_SUB_STEPS`, iteration counts
    /// `≤ MAX_ITERATIONS`, non-negative finite slop,
    /// correction in `[0, 1]`, positive `max_velocity` (`∞` disables the
    /// clamp), and non-negative finite constraint frequency and damping ratio.
    ///
    /// `PhysicsWorld::new` accepts any config; use
    /// [`PhysicsWorld::try_new`](crate::PhysicsWorld::try_new) to validate.
    ///
    /// # Errors
    /// [`ImpetusError::InvalidConfig`](crate::ImpetusError::InvalidConfig)
    /// naming the first offending field.
    pub fn validate(&self) -> crate::Result<()> {
        let bad = |field: &str, value: &dyn std::fmt::Debug| {
            Err(crate::ImpetusError::InvalidConfig(format!(
                "{field} = {value:?}"
            )))
        };
        if !(self.timestep.is_finite() && self.timestep > 0.0) {
            return bad("timestep", &self.timestep);
        }
        if !self.gravity.iter().all(|g| g.is_finite()) {
            return bad("gravity", &self.gravity);
        }
        if self.sub_steps > Self::MAX_SUB_STEPS {
            return bad("sub_steps", &self.sub_steps);
        }
        if self.velocity_iterations > Self::MAX_ITERATIONS {
            return bad("velocity_iterations", &self.velocity_iterations);
        }
        if self.position_iterations > Self::MAX_ITERATIONS {
            return bad("position_iterations", &self.position_iterations);
        }
        if !(self.position_slop.is_finite() && self.position_slop >= 0.0) {
            return bad("position_slop", &self.position_slop);
        }
        if !(0.0..=1.0).contains(&self.position_correction) {
            return bad("position_correction", &self.position_correction);
        }
        if self.max_velocity.is_nan() || self.max_velocity <= 0.0 {
            return bad("max_velocity", &self.max_velocity);
        }
        if !(self.constraint_frequency.is_finite() && self.constraint_frequency >= 0.0) {
            return bad("constraint_frequency", &self.constraint_frequency);
        }
        if !(self.constraint_damping_ratio.is_finite() && self.constraint_damping_ratio >= 0.0) {
            return bad("constraint_damping_ratio", &self.constraint_damping_ratio);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn validate_accepts_default_and_rejects_bad_values() {
        assert!(WorldConfig::default().validate().is_ok());
        let bad = [
            WorldConfig {
                timestep: -1.0,
                ..WorldConfig::default()
            },
            WorldConfig {
                timestep: f64::NAN,
                ..WorldConfig::default()
            },
            WorldConfig {
                gravity: [0.0, f64::NAN, 0.0],
                ..WorldConfig::default()
            },
            WorldConfig {
                sub_steps: u32::MAX,
                ..WorldConfig::default()
            },
            WorldConfig {
                position_correction: 2.0,
                ..WorldConfig::default()
            },
            WorldConfig {
                max_velocity: 0.0,
                ..WorldConfig::default()
            },
            WorldConfig {
                constraint_frequency: -1.0,
                ..WorldConfig::default()
            },
        ];
        for c in bad {
            assert!(
                matches!(c.validate(), Err(crate::ImpetusError::InvalidConfig(_))),
                "{c:?}"
            );
        }
        let unclamped = WorldConfig {
            max_velocity: f64::INFINITY,
            ..WorldConfig::default()
        };
        assert!(unclamped.validate().is_ok());
    }

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
            max_velocity: 100.0,
            sub_steps: 1,
            constraint_frequency: 30.0,
            constraint_damping_ratio: 1.0,
            broadphase: BroadphaseKind::SpatialHash,
            solver: SolverKind::SequentialImpulse,
        };
        let json = serde_json::to_string(&config).unwrap();
        let back: WorldConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(config, back);
    }

    #[test]
    fn solver_kind_serde() {
        let config = WorldConfig {
            solver: SolverKind::Xpbd,
            ..Default::default()
        };
        let json = serde_json::to_string(&config).unwrap();
        let back: WorldConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(config.solver, back.solver);
        assert_eq!(back.solver, SolverKind::Xpbd);
    }

    #[test]
    fn solver_kind_default_is_sequential_impulse() {
        let config = WorldConfig::default();
        assert_eq!(config.solver, SolverKind::SequentialImpulse);
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
