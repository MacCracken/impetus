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
    /// Damping coefficient (higher = less oscillation). Critical damping = 2·√stiffness.
    damping: f64,
    /// Threshold below which the spring is considered settled.
    settle_threshold: f64,
    /// Cached step transition for the last `(dt, stiffness, damping)`.
    #[serde(skip)]
    cache: StepCache,
}

/// The exact step is linear in (displacement, velocity): `[y; v] = M·[y0; v0]`.
/// `M` depends only on `dt`, stiffness and damping, so it's cached for the
/// common fixed-timestep case. Ignored by equality and serialization.
#[derive(Clone, Copy, Default)]
struct StepCache {
    key: Option<(f64, f64, f64)>,
    m: [f64; 4],
}

impl PartialEq for StepCache {
    fn eq(&self, _: &Self) -> bool {
        true
    }
}

impl std::fmt::Debug for StepCache {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("StepCache")
    }
}

/// Transition matrix `[m00, m01, m10, m11]` of the damped oscillator
/// `y'' = −k·y − c·y'` over `dt`.
fn transition(k: f64, c: f64, dt: f64) -> [f64; 4] {
    let alpha = 0.5 * c;
    let disc = alpha * alpha - k;
    let scale = alpha * alpha + k.abs();
    if disc.abs() <= 1e-9 * scale || scale == 0.0 {
        // Critically damped (or free motion when k = c = 0).
        let e = (-alpha * dt).exp();
        [
            e * (1.0 + alpha * dt),
            e * dt,
            -e * alpha * alpha * dt,
            e * (1.0 - alpha * dt),
        ]
    } else if disc < 0.0 {
        // Under-damped.
        let wd = (-disc).sqrt();
        let e = (-alpha * dt).exp();
        let (sin, cos) = (wd * dt).sin_cos();
        [
            e * (cos + alpha / wd * sin),
            e * sin / wd,
            -e * k / wd * sin,
            e * (cos - alpha / wd * sin),
        ]
    } else {
        // Over-damped: two real roots.
        let beta = disc.sqrt();
        let (r1, r2) = (-alpha + beta, -alpha - beta);
        let d = r1 - r2;
        let (e1, e2) = ((r1 * dt).exp(), (r2 * dt).exp());
        [
            (r1 * e2 - r2 * e1) / d,
            (e1 - e2) / d,
            r1 * r2 * (e2 - e1) / d,
            (r1 * e1 - r2 * e2) / d,
        ]
    }
}

impl Spring {
    /// Create a new spring.
    ///
    /// - `from`: initial position
    /// - `to`: target position
    /// - `stiffness`: spring constant (try 100-500 for UI)
    /// - `damping`: damping coefficient (critical damping is `2·√stiffness`;
    ///   try 10-30 for UI, or use `critically_damped` for no overshoot)
    #[must_use]
    pub fn new(from: f64, to: f64, stiffness: f64, damping: f64) -> Self {
        Self {
            position: from,
            velocity: 0.0,
            target: to,
            stiffness,
            damping,
            settle_threshold: 0.01,
            cache: StepCache::default(),
        }
    }

    /// Create a spring that's critically damped (no overshoot).
    /// A negative `stiffness` is treated as its magnitude.
    #[must_use]
    pub fn critically_damped(from: f64, to: f64, stiffness: f64) -> Self {
        let k = stiffness.abs();
        Self::new(from, to, k, 2.0 * k.sqrt())
    }

    /// Create an over-damped spring (slow approach, no overshoot).
    /// A negative `stiffness` is treated as its magnitude.
    #[must_use]
    pub fn over_damped(from: f64, to: f64, stiffness: f64) -> Self {
        let k = stiffness.abs();
        Self::new(from, to, k, 3.0 * k.sqrt())
    }

    /// Create an under-damped spring (bouncy overshoot).
    /// A negative `stiffness` is treated as its magnitude.
    #[must_use]
    pub fn under_damped(from: f64, to: f64, stiffness: f64) -> Self {
        let k = stiffness.abs();
        Self::new(from, to, k, 0.5 * k.sqrt())
    }

    /// Advance the spring by `dt` seconds.
    ///
    /// Uses the exact solution of the damped oscillator
    /// `x'' = −k·(x − target) − c·x'`, so the result is the same whether a
    /// second is covered by one step or by many, and it stays stable for any
    /// `dt` and stiffness (a long frame hitch can't make it explode).
    /// A non-positive or NaN `dt` does nothing.
    #[inline]
    pub fn step(&mut self, dt: f64) {
        if dt.is_nan() || dt <= 0.0 {
            return;
        }
        if dt.is_infinite() {
            if self.damping > 0.0 && self.stiffness >= 0.0 {
                self.snap();
            }
            return;
        }
        let key = (dt, self.stiffness, self.damping);
        if self.cache.key != Some(key) {
            self.cache = StepCache {
                key: Some(key),
                m: transition(self.stiffness, self.damping, dt),
            };
        }
        let m = self.cache.m;
        let y0 = self.position - self.target;
        let v0 = self.velocity;
        let y = m[0] * y0 + m[1] * v0;
        let v = m[2] * y0 + m[3] * v0;
        if y.is_finite() && v.is_finite() {
            self.position = self.target + y;
            self.velocity = v;
        }
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
    #[must_use]
    pub fn new(from: [f64; 2], to: [f64; 2], stiffness: f64, damping: f64) -> Self {
        Self {
            x: Spring::new(from[0], to[0], stiffness, damping),
            y: Spring::new(from[1], to[1], stiffness, damping),
        }
    }

    /// Critically damped 2D spring.
    #[must_use]
    pub fn critically_damped(from: [f64; 2], to: [f64; 2], stiffness: f64) -> Self {
        Self {
            x: Spring::critically_damped(from[0], to[0], stiffness),
            y: Spring::critically_damped(from[1], to[1], stiffness),
        }
    }

    /// Step both axes.
    #[inline]
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
    #[must_use]
    pub fn new(from: [f64; 3], to: [f64; 3], stiffness: f64, damping: f64) -> Self {
        Self {
            x: Spring::new(from[0], to[0], stiffness, damping),
            y: Spring::new(from[1], to[1], stiffness, damping),
            z: Spring::new(from[2], to[2], stiffness, damping),
        }
    }

    /// Critically damped 3D spring.
    #[must_use]
    pub fn critically_damped(from: [f64; 3], to: [f64; 3], stiffness: f64) -> Self {
        Self {
            x: Spring::critically_damped(from[0], to[0], stiffness),
            y: Spring::critically_damped(from[1], to[1], stiffness),
            z: Spring::critically_damped(from[2], to[2], stiffness),
        }
    }

    /// Step all axes.
    #[inline]
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

    #[test]
    fn step_is_stable_for_large_dt_and_stiffness() {
        // A 0.5 s frame hitch used to throw this spring to ~7500.
        let mut s = Spring::critically_damped(0.0, 100.0, 300.0);
        s.step(0.5);
        assert!(
            s.position() <= 100.0 + 1e-9 && s.position() > 99.0,
            "{}",
            s.position()
        );
        // Very stiff spring at 60 fps used to diverge.
        let mut s = Spring::critically_damped(0.0, 100.0, 2500.0);
        for _ in 0..120 {
            s.step(1.0 / 60.0);
            assert!(s.position().abs() < 101.0);
        }
        assert!(s.is_settled());
    }

    #[test]
    fn step_is_exact_regardless_of_step_count() {
        for (k, c) in [(300.0, 15.0), (400.0, 40.0), (100.0, 60.0), (0.0, 5.0)] {
            let mut one = Spring::new(0.0, 10.0, k, c);
            one.set_velocity(3.0);
            let mut many = one.clone();
            one.step(0.8);
            for _ in 0..800 {
                many.step(0.001);
            }
            assert!(
                (one.position() - many.position()).abs() < 1e-9,
                "k={k} c={c}"
            );
            assert!(
                (one.velocity() - many.velocity()).abs() < 1e-9,
                "k={k} c={c}"
            );
        }
    }

    #[test]
    fn bad_dt_is_ignored() {
        let mut s = Spring::new(0.0, 1.0, 100.0, 10.0);
        for dt in [0.0, -1.0, f64::NAN] {
            s.step(dt);
            assert_eq!(s.position(), 0.0);
        }
        s.step(f64::INFINITY);
        assert_eq!(s.position(), 1.0);
    }
}
