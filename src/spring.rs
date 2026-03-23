//! Standalone spring physics for animation.
//!
//! Damped harmonic oscillator — no world, no bodies, no colliders needed.
//! Designed for UI/compositor use cases: window animations, snap-to-grid,
//! scroll physics, drag-and-release bounce.
//!
//! ```ignore
//! let mut spring = Spring::new(0.0, 100.0, 300.0, 15.0);
//! loop {
//!     spring.step(1.0 / 60.0);
//!     draw_at(spring.position());
//!     if spring.is_settled() { break; }
//! }
//! ```

use serde::{Deserialize, Serialize};

/// A 1D damped spring that animates a value toward a target.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Spring {
    /// Current position.
    position: f64,
    /// Current velocity.
    velocity: f64,
    /// Target position the spring is pulling toward.
    target: f64,
    /// Spring stiffness (higher = snappier). Units: force per unit displacement.
    stiffness: f64,
    /// Damping coefficient (higher = less oscillation). Critical damping = 2*sqrt(stiffness).
    damping: f64,
    /// Threshold below which the spring is considered settled.
    settle_threshold: f64,
}

impl Spring {
    /// Create a new spring.
    ///
    /// - `from`: initial position
    /// - `to`: target position
    /// - `stiffness`: spring constant (try 100-500 for UI)
    /// - `damping`: damping ratio (try 10-30 for UI; use `critically_damped` for no overshoot)
    pub fn new(from: f64, to: f64, stiffness: f64, damping: f64) -> Self {
        Self {
            position: from,
            velocity: 0.0,
            target: to,
            stiffness,
            damping,
            settle_threshold: 0.01,
        }
    }

    /// Create a spring that's critically damped (no overshoot).
    pub fn critically_damped(from: f64, to: f64, stiffness: f64) -> Self {
        Self::new(from, to, stiffness, 2.0 * stiffness.sqrt())
    }

    /// Create an over-damped spring (slow approach, no overshoot).
    pub fn over_damped(from: f64, to: f64, stiffness: f64) -> Self {
        Self::new(from, to, stiffness, 3.0 * stiffness.sqrt())
    }

    /// Create an under-damped spring (bouncy overshoot).
    pub fn under_damped(from: f64, to: f64, stiffness: f64) -> Self {
        Self::new(from, to, stiffness, 0.5 * stiffness.sqrt())
    }

    /// Step the spring by `dt` seconds (semi-implicit Euler).
    pub fn step(&mut self, dt: f64) {
        let displacement = self.position - self.target;
        let force = -self.stiffness * displacement - self.damping * self.velocity;
        self.velocity += force * dt;
        self.position += self.velocity * dt;
    }

    /// Current position.
    #[must_use]
    pub fn position(&self) -> f64 {
        self.position
    }

    /// Current velocity.
    #[must_use]
    pub fn velocity(&self) -> f64 {
        self.velocity
    }

    /// Target position.
    #[must_use]
    pub fn target(&self) -> f64 {
        self.target
    }

    /// Set a new target (e.g., user drags window to new position).
    pub fn set_target(&mut self, target: f64) {
        self.target = target;
    }

    /// Set the current position directly (e.g., teleport).
    pub fn set_position(&mut self, position: f64) {
        self.position = position;
    }

    /// Add velocity (e.g., fling gesture).
    pub fn add_velocity(&mut self, v: f64) {
        self.velocity += v;
    }

    /// Set velocity directly.
    pub fn set_velocity(&mut self, v: f64) {
        self.velocity = v;
    }

    /// Whether the spring has settled (position ≈ target and velocity ≈ 0).
    #[must_use]
    pub fn is_settled(&self) -> bool {
        (self.position - self.target).abs() < self.settle_threshold
            && self.velocity.abs() < self.settle_threshold
    }

    /// Set the settle threshold.
    pub fn set_settle_threshold(&mut self, threshold: f64) {
        self.settle_threshold = threshold;
    }

    /// Snap to target immediately (no animation).
    pub fn snap(&mut self) {
        self.position = self.target;
        self.velocity = 0.0;
    }

    /// Stiffness value.
    #[must_use]
    pub fn stiffness(&self) -> f64 {
        self.stiffness
    }

    /// Damping value.
    #[must_use]
    pub fn damping(&self) -> f64 {
        self.damping
    }

    /// Set stiffness.
    pub fn set_stiffness(&mut self, stiffness: f64) {
        self.stiffness = stiffness;
    }

    /// Set damping.
    pub fn set_damping(&mut self, damping: f64) {
        self.damping = damping;
    }
}

/// A 2D damped spring (two independent axes).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Spring2d {
    pub x: Spring,
    pub y: Spring,
}

impl Spring2d {
    /// Create a 2D spring.
    pub fn new(from: [f64; 2], to: [f64; 2], stiffness: f64, damping: f64) -> Self {
        Self {
            x: Spring::new(from[0], to[0], stiffness, damping),
            y: Spring::new(from[1], to[1], stiffness, damping),
        }
    }

    /// Critically damped 2D spring.
    pub fn critically_damped(from: [f64; 2], to: [f64; 2], stiffness: f64) -> Self {
        Self {
            x: Spring::critically_damped(from[0], to[0], stiffness),
            y: Spring::critically_damped(from[1], to[1], stiffness),
        }
    }

    /// Step both axes.
    pub fn step(&mut self, dt: f64) {
        self.x.step(dt);
        self.y.step(dt);
    }

    /// Current position as [x, y].
    #[must_use]
    pub fn position(&self) -> [f64; 2] {
        [self.x.position(), self.y.position()]
    }

    /// Set target position.
    pub fn set_target(&mut self, target: [f64; 2]) {
        self.x.set_target(target[0]);
        self.y.set_target(target[1]);
    }

    /// Whether both axes have settled.
    #[must_use]
    pub fn is_settled(&self) -> bool {
        self.x.is_settled() && self.y.is_settled()
    }

    /// Snap to target.
    pub fn snap(&mut self) {
        self.x.snap();
        self.y.snap();
    }

    /// Add velocity.
    pub fn add_velocity(&mut self, v: [f64; 2]) {
        self.x.add_velocity(v[0]);
        self.y.add_velocity(v[1]);
    }
}

/// A 3D damped spring.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Spring3d {
    pub x: Spring,
    pub y: Spring,
    pub z: Spring,
}

impl Spring3d {
    /// Create a 3D spring.
    pub fn new(from: [f64; 3], to: [f64; 3], stiffness: f64, damping: f64) -> Self {
        Self {
            x: Spring::new(from[0], to[0], stiffness, damping),
            y: Spring::new(from[1], to[1], stiffness, damping),
            z: Spring::new(from[2], to[2], stiffness, damping),
        }
    }

    /// Critically damped 3D spring.
    pub fn critically_damped(from: [f64; 3], to: [f64; 3], stiffness: f64) -> Self {
        Self {
            x: Spring::critically_damped(from[0], to[0], stiffness),
            y: Spring::critically_damped(from[1], to[1], stiffness),
            z: Spring::critically_damped(from[2], to[2], stiffness),
        }
    }

    /// Step all axes.
    pub fn step(&mut self, dt: f64) {
        self.x.step(dt);
        self.y.step(dt);
        self.z.step(dt);
    }

    /// Current position.
    #[must_use]
    pub fn position(&self) -> [f64; 3] {
        [self.x.position(), self.y.position(), self.z.position()]
    }

    /// Set target.
    pub fn set_target(&mut self, target: [f64; 3]) {
        self.x.set_target(target[0]);
        self.y.set_target(target[1]);
        self.z.set_target(target[2]);
    }

    /// Whether all axes have settled.
    #[must_use]
    pub fn is_settled(&self) -> bool {
        self.x.is_settled() && self.y.is_settled() && self.z.is_settled()
    }

    /// Snap to target.
    pub fn snap(&mut self) {
        self.x.snap();
        self.y.snap();
        self.z.snap();
    }

    /// Add velocity.
    pub fn add_velocity(&mut self, v: [f64; 3]) {
        self.x.add_velocity(v[0]);
        self.y.add_velocity(v[1]);
        self.z.add_velocity(v[2]);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spring_moves_toward_target() {
        let mut s = Spring::new(0.0, 100.0, 300.0, 15.0);
        for _ in 0..600 {
            s.step(1.0 / 60.0);
        }
        assert!((s.position() - 100.0).abs() < 0.1);
    }

    #[test]
    fn spring_critically_damped_no_overshoot() {
        let mut s = Spring::critically_damped(0.0, 100.0, 300.0);
        let mut max_pos = 0.0_f64;
        for _ in 0..600 {
            s.step(1.0 / 60.0);
            max_pos = max_pos.max(s.position());
        }
        // Critically damped should not significantly overshoot
        assert!(max_pos < 105.0, "max={max_pos}, should not overshoot much");
    }

    #[test]
    fn spring_under_damped_overshoots() {
        let mut s = Spring::under_damped(0.0, 100.0, 300.0);
        let mut max_pos = 0.0_f64;
        for _ in 0..600 {
            s.step(1.0 / 60.0);
            max_pos = max_pos.max(s.position());
        }
        assert!(max_pos > 100.0, "under-damped should overshoot");
    }

    #[test]
    fn spring_settles() {
        let mut s = Spring::new(0.0, 50.0, 200.0, 20.0);
        assert!(!s.is_settled());
        for _ in 0..600 {
            s.step(1.0 / 60.0);
        }
        assert!(s.is_settled());
    }

    #[test]
    fn spring_snap() {
        let mut s = Spring::new(0.0, 100.0, 300.0, 15.0);
        s.snap();
        assert_eq!(s.position(), 100.0);
        assert_eq!(s.velocity(), 0.0);
        assert!(s.is_settled());
    }

    #[test]
    fn spring_set_target() {
        let mut s = Spring::new(0.0, 100.0, 300.0, 15.0);
        s.set_target(200.0);
        assert_eq!(s.target(), 200.0);
        for _ in 0..600 {
            s.step(1.0 / 60.0);
        }
        assert!((s.position() - 200.0).abs() < 0.1);
    }

    #[test]
    fn spring_fling() {
        let mut s = Spring::new(50.0, 50.0, 300.0, 15.0);
        s.add_velocity(500.0);
        s.step(1.0 / 60.0);
        assert!(s.position() > 50.0, "should have moved from fling");
    }

    #[test]
    fn spring_serde() {
        let s = Spring::new(10.0, 90.0, 200.0, 12.0);
        let json = serde_json::to_string(&s).unwrap();
        let back: Spring = serde_json::from_str(&json).unwrap();
        assert_eq!(s, back);
    }

    #[test]
    fn spring_2d_moves() {
        let mut s = Spring2d::new([0.0, 0.0], [100.0, 200.0], 300.0, 20.0);
        for _ in 0..600 {
            s.step(1.0 / 60.0);
        }
        let pos = s.position();
        assert!((pos[0] - 100.0).abs() < 0.1);
        assert!((pos[1] - 200.0).abs() < 0.1);
        assert!(s.is_settled());
    }

    #[test]
    fn spring_2d_snap() {
        let mut s = Spring2d::new([0.0, 0.0], [50.0, 75.0], 300.0, 20.0);
        s.snap();
        assert_eq!(s.position(), [50.0, 75.0]);
    }

    #[test]
    fn spring_3d_moves() {
        let mut s = Spring3d::new([0.0, 0.0, 0.0], [10.0, 20.0, 30.0], 300.0, 20.0);
        for _ in 0..600 {
            s.step(1.0 / 60.0);
        }
        let pos = s.position();
        assert!((pos[0] - 10.0).abs() < 0.1);
        assert!((pos[1] - 20.0).abs() < 0.1);
        assert!((pos[2] - 30.0).abs() < 0.1);
    }

    #[test]
    fn spring_3d_set_target() {
        let mut s = Spring3d::new([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 300.0, 20.0);
        s.set_target([5.0, 10.0, 15.0]);
        for _ in 0..600 {
            s.step(1.0 / 60.0);
        }
        assert!(s.is_settled());
    }

    #[test]
    fn spring_stiffness_damping_accessors() {
        let mut s = Spring::new(0.0, 100.0, 300.0, 15.0);
        assert_eq!(s.stiffness(), 300.0);
        assert_eq!(s.damping(), 15.0);
        s.set_stiffness(500.0);
        s.set_damping(25.0);
        assert_eq!(s.stiffness(), 500.0);
        assert_eq!(s.damping(), 25.0);
    }
}
