//! `spring.tsv` — the standalone springs (M1b).
//!
//! - `Spring::new` and the three damping presets (class A: `sqrt` is correctly rounded);
//! - `Spring::is_settled` (class A);
//! - `Spring::step`, n steps of one dt from a state (class C: the transition calls exp, sin and cos;
//!   the cached steps after it are × and + only). The grid: the three regimes and the band around
//!   the critical branch point (|disc| ≤ 1e-9·scale) × dt ∈ {1e-6, 1/60, 1, 100}, random springs,
//!   and the edges (free motion, negative stiffness or damping, overflow, bad dt).
//!
//! Each step row's note carries its sensitivity `S`: the most the outputs move, in ulps, when one
//! class-C result on the path moves by one ulp. It comes from a transcription of `transition` and
//! `step` that is checked bit for bit against the oracle on every row, and sets the class-C bound
//! by the port-parity rule (`docs/development/port-parity.md` §2) before the first comparison.

use impetus::Spring;

use crate::flat;
use crate::grids::Rng;
use crate::tsv::{Class, Module, b, f, i};

/// The class-C results on the transition's path: (branch, values). Branch 0 critical (e), 1 under
/// (e, sin, cos), 2 over (e1, e2).
fn class_c(k: f64, c: f64, dt: f64) -> (u8, Vec<f64>) {
    let alpha = 0.5 * c;
    let disc = alpha * alpha - k;
    let scale = alpha * alpha + k.abs();
    if disc.abs() <= 1e-9 * scale || scale == 0.0 {
        (0, vec![(-alpha * dt).exp()])
    } else if disc < 0.0 {
        let wd = (-disc).sqrt();
        let (sin, cos) = (wd * dt).sin_cos();
        (1, vec![(-alpha * dt).exp(), sin, cos])
    } else {
        let beta = disc.sqrt();
        let (r1, r2) = (-alpha + beta, -alpha - beta);
        (2, vec![(r1 * dt).exp(), (r2 * dt).exp()])
    }
}

/// `transition` with its class-C results given (Rust's expressions, in Rust's order).
fn transition(k: f64, c: f64, dt: f64, branch: u8, x: &[f64]) -> [f64; 4] {
    let alpha = 0.5 * c;
    match branch {
        0 => {
            let e = x[0];
            [
                e * (1.0 + alpha * dt),
                e * dt,
                -e * alpha * alpha * dt,
                e * (1.0 - alpha * dt),
            ]
        }
        1 => {
            let wd = (-(alpha * alpha - k)).sqrt();
            let (e, sin, cos) = (x[0], x[1], x[2]);
            [
                e * (cos + alpha / wd * sin),
                e * sin / wd,
                -e * k / wd * sin,
                e * (cos - alpha / wd * sin),
            ]
        }
        _ => {
            let beta = (alpha * alpha - k).sqrt();
            let (r1, r2) = (-alpha + beta, -alpha - beta);
            let d = r1 - r2;
            let (e1, e2) = (x[0], x[1]);
            [
                (r1 * e2 - r2 * e1) / d,
                (e1 - e2) / d,
                r1 * r2 * (e2 - e1) / d,
                (r1 * e1 - r2 * e2) / d,
            ]
        }
    }
}

/// `step` n times from (p, v) toward t with matrix m: the final (position, velocity).
fn run(m: [f64; 4], mut p: f64, mut v: f64, t: f64, n: u32) -> (f64, f64) {
    for _ in 0..n {
        let y0 = p - t;
        let v0 = v;
        let y = m[0] * y0 + m[1] * v0;
        let vv = m[2] * y0 + m[3] * v0;
        if y.is_finite() && vv.is_finite() {
            p = t + y;
            v = vv;
        }
    }
    (p, v)
}

/// The ulp distance on the ordered-integer line; NaN is 0 from NaN and the maximum from anything
/// else.
fn ulps(a: f64, b: f64) -> u64 {
    if a.is_nan() || b.is_nan() {
        return if a.is_nan() && b.is_nan() {
            0
        } else {
            u64::MAX
        };
    }
    let key = |x: f64| {
        let bits = x.to_bits().cast_signed();
        if bits < 0 { i64::MIN - bits } else { bits }
    };
    key(a).abs_diff(key(b))
}

/// The row's sensitivity: the largest output move, in ulps, over ±1 ulp on each class-C result.
fn sensitivity(k: f64, c: f64, dt: f64, p: f64, v: f64, t: f64, n: u32) -> u64 {
    let (branch, x) = class_c(k, c, dt);
    let base = run(transition(k, c, dt, branch, &x), p, v, t, n);
    let mut worst = 0;
    for j in 0..x.len() {
        for up in [true, false] {
            let mut y = x.clone();
            y[j] = if up { y[j].next_up() } else { y[j].next_down() };
            let out = run(transition(k, c, dt, branch, &y), p, v, t, n);
            worst = worst.max(ulps(out.0, base.0)).max(ulps(out.1, base.1));
        }
    }
    worst
}

struct Case {
    p: f64,
    v: f64,
    t: f64,
    k: f64,
    c: f64,
    dt: f64,
    n: u32,
}

fn step_row(m: &mut Module, x: &Case) {
    let mut s = Spring::new(x.p, x.t, x.k, x.c);
    s.set_velocity(x.v);
    for _ in 0..x.n {
        s.step(x.dt);
    }
    let ins = [f(x.p), f(x.v), f(x.t), f(x.k), f(x.c), f(x.dt), i(x.n)];
    let outs = [f(s.position()), f(s.velocity())];
    let regular = !x.dt.is_nan() && x.dt > 0.0 && x.dt.is_finite();
    if !regular {
        m.row("Spring::step", Class::C, &ins, &outs);
        return;
    }
    // The transcription must be the oracle's own arithmetic, bit for bit.
    let (branch, cv) = class_c(x.k, x.c, x.dt);
    let mine = run(transition(x.k, x.c, x.dt, branch, &cv), x.p, x.v, x.t, x.n);
    assert!(
        mine.0.to_bits() == s.position().to_bits() && mine.1.to_bits() == s.velocity().to_bits(),
        "spring transcription differs from the oracle at k={} c={} dt={}",
        x.k,
        x.c,
        x.dt
    );
    let sens = sensitivity(x.k, x.c, x.dt, x.p, x.v, x.t, x.n);
    let regime = ["critical", "under", "over"][usize::from(branch)];
    m.row_noted(
        "Spring::step",
        Class::C,
        &ins,
        &outs,
        &format!("{regime} S={sens}"),
    );
}

#[allow(clippy::too_many_lines)] // the grid, then the edges
pub fn emit() -> Module {
    let mut m = Module::new("spring");
    let mut rng = Rng::new(0x5_1219);
    let ks = [
        0.0,
        1.0,
        100.0,
        300.0,
        2500.0,
        1e6,
        -50.0,
        f64::NAN,
        f64::INFINITY,
    ];

    // ── constructors ──
    for &k in &ks {
        for &c in &[0.0, 15.0, -3.0, f64::NAN] {
            let s = Spring::new(0.0, 100.0, k, c);
            m.row(
                "Spring::new",
                Class::A,
                &[f(0.0), f(100.0), f(k), f(c)],
                &flat::spring(&s),
            );
        }
        for (name, ctor) in [
            (
                "Spring::critically_damped",
                Spring::critically_damped as fn(f64, f64, f64) -> Spring,
            ),
            ("Spring::over_damped", Spring::over_damped),
            ("Spring::under_damped", Spring::under_damped),
        ] {
            let s = ctor(-1.5, 2.5, k);
            m.row(name, Class::A, &[f(-1.5), f(2.5), f(k)], &flat::spring(&s));
        }
    }
    for _ in 0..30 {
        let (from, to, k) = (
            rng.uniform(-1e3, 1e3),
            rng.uniform(-1e3, 1e3),
            rng.log_uniform(-3.0, 6.0),
        );
        for (name, ctor) in [
            (
                "Spring::critically_damped",
                Spring::critically_damped as fn(f64, f64, f64) -> Spring,
            ),
            ("Spring::over_damped", Spring::over_damped),
            ("Spring::under_damped", Spring::under_damped),
        ] {
            m.row(
                name,
                Class::A,
                &[f(from), f(to), f(k)],
                &flat::spring(&ctor(from, to, k)),
            );
        }
    }

    // ── is_settled ──
    let mut settle = vec![
        (100.0, 0.0, 100.0, 0.01),
        (100.009, 0.009, 100.0, 0.01),
        (100.01, 0.0, 100.0, 0.01),
        (100.0, -0.01, 100.0, 0.01),
        (f64::NAN, 0.0, 100.0, 0.01),
        (0.0, 0.0, 0.0, 0.0),
        (1.0, 1.0, 1.0, f64::INFINITY),
    ];
    for _ in 0..40 {
        let t = rng.uniform(-10.0, 10.0);
        settle.push((
            t + rng.signed_log_uniform(-4.0, 0.0),
            rng.signed_log_uniform(-4.0, 0.0),
            t,
            rng.log_uniform(-4.0, 0.0),
        ));
    }
    for (p, v, t, thr) in settle {
        let mut s = Spring::new(p, t, 1.0, 1.0);
        s.set_velocity(v);
        s.set_settle_threshold(thr);
        m.row(
            "Spring::is_settled",
            Class::A,
            &[f(p), f(v), f(t), f(thr)],
            &[b(s.is_settled())],
        );
    }

    // ── step: regimes × dt ──
    let zetas = [
        0.0,
        0.1,
        0.5,
        0.999,
        1.0,
        1.001,
        1.5,
        3.0,
        10.0, // regimes
        0.999_999,
        0.999_999_99,
        1.000_000_01,
        1.000_001,
        1.0001, // the critical band
    ];
    for &k in &[1.0, 100.0, 300.0, 2500.0, 1e6] {
        for &z in &zetas {
            let c = if z == 1.0 {
                2.0 * f64::sqrt(k)
            } else {
                z * 2.0 * f64::sqrt(k)
            };
            for &dt in &[1e-6, 1.0 / 60.0, 1.0, 100.0] {
                for n in [1, 60] {
                    step_row(
                        &mut m,
                        &Case {
                            p: 0.0,
                            v: 3.0,
                            t: 100.0,
                            k,
                            c,
                            dt,
                            n,
                        },
                    );
                }
            }
        }
    }
    // ── step: random springs ──
    for _ in 0..200 {
        let x = Case {
            p: rng.uniform(-1e3, 1e3),
            v: rng.uniform(-1e2, 1e2),
            t: rng.uniform(-1e3, 1e3),
            k: rng.log_uniform(-2.0, 5.0),
            c: rng.log_uniform(-2.0, 3.0),
            dt: rng.log_uniform(-6.0, 1.0),
            n: u32::try_from(rng.int(1, 100)).unwrap_or(1),
        };
        step_row(&mut m, &x);
    }
    // ── step: edges ──
    let edges = [
        (0.0, 0.0, 0.5),      // free motion: k = c = 0
        (0.0, 5.0, 0.8),      // pure damping
        (-50.0, 1.0, 0.1),    // negative stiffness: exponential growth
        (100.0, -2.0, 0.1),   // negative damping: growing oscillation
        (100.0, 10.0, 1e300), // overflow: the state is left unchanged
        (1e300, 1.0, 1.0),    // huge stiffness
        (100.0, 10.0, 0.0),   // dt 0
        (100.0, 10.0, -1.0),  // dt < 0
        (100.0, 10.0, f64::NAN),
        (100.0, 10.0, f64::INFINITY), // damped: snaps
        (100.0, 0.0, f64::INFINITY),  // undamped: does nothing
        (-1.0, 10.0, f64::INFINITY),  // negative stiffness: does nothing
        (f64::NAN, 10.0, 0.1),
        (100.0, f64::NAN, 0.1),
    ];
    for (k, c, dt) in edges {
        for n in [1, 10] {
            step_row(
                &mut m,
                &Case {
                    p: 0.0,
                    v: 3.0,
                    t: 10.0,
                    k,
                    c,
                    dt,
                    n,
                },
            );
        }
    }
    // The Rust tests' scenarios, as rows: 600 frames at 60 fps, and one 0.8 s step against 800.
    for (k, c) in [(300.0, 15.0), (400.0, 40.0), (100.0, 60.0), (0.0, 5.0)] {
        step_row(
            &mut m,
            &Case {
                p: 0.0,
                v: 3.0,
                t: 10.0,
                k,
                c,
                dt: 0.8,
                n: 1,
            },
        );
        step_row(
            &mut m,
            &Case {
                p: 0.0,
                v: 3.0,
                t: 10.0,
                k,
                c,
                dt: 0.001,
                n: 800,
            },
        );
        step_row(
            &mut m,
            &Case {
                p: 0.0,
                v: 0.0,
                t: 100.0,
                k,
                c,
                dt: 1.0 / 60.0,
                n: 600,
            },
        );
    }
    m
}
