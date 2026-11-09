//! `math.tsv` — what Rust **std** returns for every f64 method impetus 1.4.0's library code
//! calls: the reference for the M0 step 8 baselines (port plan §4 step 5, §7.1).
//!
//! The calls, counted in `rust-old/src` outside test code (2026-11-09): abs 284, max 74,
//! is_finite 42, sqrt 41, min 32, clamp 28, powi 19 (exponent 2 or 3, always a literal),
//! sin_cos 15, hypot 15, is_nan 14, to_radians 5, exp 4 (the spring transition), atan2 2
//! (a quaternion's z/w back to an angle), floor 2, ceil 2, sin 1, cos 1, powf 1 (a force
//! field's falloff).
//!
//! Where each result comes from (rustc 1.99.1, x86_64-unknown-linux-gnu):
//! - **glibc's libm** (dynamic): sin, cos (and `sin_cos`, which std writes as the pair),
//!   atan2, hypot, exp, pow. These rows are glibc's bits (`glibc.rs`): the header records the
//!   glibc, and `--check` refuses another.
//! - **inline code**: sqrt (`sqrtsd`), abs, floor, ceil (`roundsd`), min, max, clamp,
//!   is_nan, is_finite, to_radians (one multiply by the folded π/180), and powi with a literal
//!   exponent (LLVM expands `x.powi(2)` to `x·x` and `x.powi(3)` to products).
//!
//! Every input passes through `black_box`, so each row is a run-time call, never a fold. The
//! exponent of `powi` is *not* black-boxed: impetus always writes it as a literal, so the
//! rows record the expansion its code actually runs (`f64::powi2`, `f64::powi3`).
//!
//! Classes:
//! - **A**: exact on both sides — sqrt, abs, floor, ceil, min, max, clamp, is_nan, is_finite.
//! - **B**: arithmetic — to_radians, powi2, powi3.
//! - **C**: glibc — sin_cos, sin, cos, atan2, hypot, exp, powf.
//!
//! Inputs are the special values, the ranges impetus feeds each function, and seeded sweeps.

use std::hint::black_box;

use crate::grids::{Rng, pow2, specials};
use crate::tsv::{Class, Module, f, i};

const PI: f64 = core::f64::consts::PI;

fn unary(m: &mut Module, name: &str, class: Class, fun: fn(f64) -> f64, xs: &[f64]) {
    for &x in xs {
        let y = fun(black_box(x));
        m.row(name, class, &[f(x)], &[f(y)]);
    }
}

fn binary(m: &mut Module, name: &str, class: Class, fun: fn(f64, f64) -> f64, xy: &[(f64, f64)]) {
    for &(x, y) in xy {
        let r = fun(black_box(x), black_box(y));
        m.row(name, class, &[f(x), f(y)], &[f(r)]);
    }
}

/// Every ordered pair of `xs`.
fn grid(xs: &[f64]) -> Vec<(f64, f64)> {
    xs.iter()
        .flat_map(|&x| xs.iter().map(move |&y| (x, y)))
        .collect()
}

/// Specials, ±2^k at every 16th binade, and a wide signed log-uniform sweep.
fn wide(seed: u64, n: usize) -> Vec<f64> {
    let mut v = specials();
    for k in (-1074..=1023).step_by(16) {
        let x = pow2(k);
        v.push(x);
        v.push(-x);
    }
    let mut rng = Rng::new(seed);
    v.extend((0..n).map(|_| rng.signed_log_uniform(-300.0, 300.0)));
    v
}

fn sweep(seed: u64, n: usize, lo: f64, hi: f64) -> Vec<f64> {
    let mut rng = Rng::new(seed);
    (0..n).map(|_| rng.uniform(lo, hi)).collect()
}

/// x, its two neighbours, and the three negated.
fn around(x: f64) -> [f64; 6] {
    [
        x,
        x.next_up(),
        x.next_down(),
        -x,
        -x.next_up(),
        -x.next_down(),
    ]
}

/// The correctly rounded sin and cos's thresholds (ADR 0011, src/crmath.cyr), each straddled: the
/// tiny-argument cut 2^-27, the slow path's no-reduction cut 0.78, the fast path's 2^20, the
/// cancellation cut on a reduced 2^-24 (the doubles nearest kπ/2 for k = 1 … 8 reduce below it),
/// and the double closest to a multiple of π/2 (6381956970095103·2^797, 2^-60.9 away).
fn sin_cos_edges() -> Vec<f64> {
    let mut v = Vec::new();
    let pio2 = f64::from_bits(0x3FF9_21FB_5444_2D18);
    for x in [
        2f64.powi(-27),
        0.78,
        2f64.powi(20),
        6_381_956_970_095_103.0 * 2f64.powi(797),
    ] {
        v.extend(around(x));
    }
    for k in 1..=8_i32 {
        v.extend(around(f64::from(k) * pio2));
    }
    v
}

/// The correctly rounded exp's thresholds, each straddled: the 1.0 cut 2^-55, the fast path's
/// −708 and 709.7, the overflow cut 710 and the underflow cut −746.
fn exp_edges() -> Vec<f64> {
    let mut v = Vec::new();
    for x in [2f64.powi(-55), 708.0, 709.7, 710.0, 746.0] {
        v.extend(around(x));
    }
    v
}

fn log_sweep(seed: u64, n: usize, lo_exp10: f64, hi_exp10: f64) -> Vec<f64> {
    let mut rng = Rng::new(seed);
    (0..n)
        .map(|_| rng.log_uniform(lo_exp10, hi_exp10))
        .collect()
}

fn pairs(seed: u64, n: usize, lo_exp10: f64, hi_exp10: f64) -> Vec<(f64, f64)> {
    let mut rng = Rng::new(seed);
    (0..n)
        .map(|_| {
            let x = rng.signed_log_uniform(lo_exp10, hi_exp10);
            (x, rng.signed_log_uniform(lo_exp10, hi_exp10))
        })
        .collect()
}

fn uniform_pairs(seed: u64, n: usize, lo: f64, hi: f64) -> Vec<(f64, f64)> {
    let mut rng = Rng::new(seed);
    (0..n)
        .map(|_| {
            let x = rng.uniform(lo, hi);
            (x, rng.uniform(lo, hi))
        })
        .collect()
}

fn powi2(x: f64) -> f64 {
    x.powi(2)
}

fn powi3(x: f64) -> f64 {
    x.powi(3)
}

#[allow(clippy::too_many_lines)] // one grid per function, in the module comment's order
pub fn emit() -> Module {
    let mut m = Module::new("math");

    // ── exact (class A) ──
    let mut xs = wide(0x5157, 300);
    xs.extend(log_sweep(0x5158, 300, -40.0, 20.0));
    xs.extend((1..=100).map(|k| f64::from(k * k)));
    xs.extend(sweep(0x5159, 200, 0.0, 1e4));
    unary(&mut m, "f64::sqrt", Class::A, f64::sqrt, &xs);
    unary(&mut m, "f64::abs", Class::A, f64::abs, &wide(0xab5, 100));

    // floor / ceil: impetus's spatial hash takes floor/ceil of coordinate / cell size. Ties
    // and the doubles either side of an integer, then sweeps.
    let mut halves: Vec<f64> = (-20..=20).map(|k| f64::from(k) + 0.5).collect();
    halves.extend((-20..=20).map(|k| f64::from(k) - f64::EPSILON * 8.0));
    halves.extend((-20..=20).map(f64::from));
    halves.extend([
        0.499_999_999_999_999_94,
        -0.499_999_999_999_999_94,
        4_503_599_627_370_495.5,
        -4_503_599_627_370_495.5,
        4_503_599_627_370_497.0,
    ]);
    halves.extend(sweep(0x7a11, 300, -1e4, 1e4));
    halves.extend(wide(0x7a12, 100));
    unary(&mut m, "f64::floor", Class::A, f64::floor, &halves);
    unary(&mut m, "f64::ceil", Class::A, f64::ceil, &halves);

    // min / max: every special pair (NaN on either side, ±0 both ways, ±∞), equal pairs, a
    // value against 0.0 both ways (impetus writes `x.max(0.0)`), and random pairs.
    let mut mm = grid(&specials());
    let mut rng = Rng::new(0x3a3a);
    for _ in 0..60 {
        let x = rng.signed_log_uniform(-30.0, 30.0);
        mm.push((x, x));
        mm.push((x, 0.0));
        mm.push((0.0, x));
        mm.push((x, f64::NAN));
    }
    mm.extend(pairs(0x3a3b, 300, -300.0, 300.0));
    binary(&mut m, "f64::max", Class::A, f64::max, &mm);
    binary(&mut m, "f64::min", Class::A, f64::min, &mm);

    // clamp: valid bounds only (min ≤ max, neither NaN; Rust panics otherwise), with the
    // probes at and around each bound, ±0, NaN and a random value.
    let mut rng = Rng::new(0xc1a2);
    let mut bounds = vec![
        (0.0, 0.0),
        (-0.0, 0.0),
        (-0.0, -0.0),
        (0.0, 1.0),
        (-1.0, 1.0),
        (0.0, f64::INFINITY),
        (f64::NEG_INFINITY, 0.0),
    ];
    for _ in 0..80 {
        let a = rng.signed_log_uniform(-10.0, 10.0);
        let c = rng.signed_log_uniform(-10.0, 10.0);
        bounds.push(if a <= c { (a, c) } else { (c, a) });
    }
    for &(lo, hi) in &bounds {
        let probes = [
            lo,
            hi,
            0.0,
            -0.0,
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            rng.signed_log_uniform(-10.0, 10.0),
        ];
        for &x in &probes {
            let r = black_box(x).clamp(black_box(lo), black_box(hi));
            m.row("f64::clamp", Class::A, &[f(x), f(lo), f(hi)], &[f(r)]);
        }
    }

    let mut preds = specials();
    preds.extend([
        f64::from_bits(0xfff8_0000_0000_0000), // -NaN
        f64::from_bits(0x7ff0_0000_0000_0001), // signalling-pattern NaN
        -f64::MAX,
        -f64::from_bits(1),
    ]);
    preds.extend(log_sweep(0xf1f1, 20, -300.0, 300.0));
    for &x in &preds {
        m.row(
            "f64::is_finite",
            Class::A,
            &[f(x)],
            &[i(i64::from(black_box(x).is_finite()))],
        );
        m.row(
            "f64::is_nan",
            Class::A,
            &[f(x)],
            &[i(i64::from(black_box(x).is_nan()))],
        );
    }

    // ── arithmetic (class B) ──
    // to_radians: impetus converts joint limits and angles given in degrees.
    let mut deg = specials();
    deg.extend((-720..=720).step_by(15).map(f64::from));
    deg.extend(sweep(0xde61, 300, -720.0, 720.0));
    deg.extend(wide(0xde62, 100));
    unary(&mut m, "f64::to_radians", Class::B, f64::to_radians, &deg);

    // powi with impetus's literal exponents: squared distances and speeds (2), ball volume (3).
    let mut pw = specials();
    pw.extend(sweep(0x9101, 300, -100.0, 100.0));
    pw.extend(log_sweep(0x9102, 200, -150.0, 150.0));
    pw.extend(
        log_sweep(0x9103, 100, -150.0, 150.0)
            .into_iter()
            .map(|x| -x),
    );
    unary(&mut m, "f64::powi2", Class::B, powi2, &pw);
    unary(&mut m, "f64::powi3", Class::B, powi3, &pw);

    // ── transcendental (class C) ──
    // sin_cos: body rotation angles. Two output tokens, sin then cos. Dense in one turn, then
    // angles that have accumulated over a long run, tiny angles, and huge ones.
    let mut ang = specials();
    ang.extend(sweep(0x5101, 1000, -2.0 * PI, 2.0 * PI));
    ang.extend(sweep(0x5102, 300, -1e3, 1e3));
    ang.extend(log_sweep(0x5103, 200, -12.0, 0.0));
    ang.extend(log_sweep(0x5104, 200, 0.0, 15.0));
    ang.extend(wide(0x5105, 100));
    ang.extend(sin_cos_edges());
    for &x in &ang {
        let (s, c) = black_box(x).sin_cos();
        m.row("f64::sin_cos", Class::C, &[f(x)], &[f(s), f(c)]);
    }
    unary(&mut m, "f64::sin", Class::C, f64::sin, &ang);
    unary(&mut m, "f64::cos", Class::C, f64::cos, &ang);

    // atan2(y, x): impetus's two calls are `q.z.atan2(q.w)` for a unit quaternion, so both
    // arguments in [−1, 1]; then every quadrant, the special grid (±0 and ±∞ in each slot)
    // and wide pairs.
    let mut at = grid(&specials());
    at.extend(uniform_pairs(0xa7a1, 600, -1.0, 1.0));
    at.extend(pairs(0xa7a2, 300, -10.0, 10.0));
    at.extend(pairs(0xa7a3, 200, -300.0, 300.0));
    binary(&mut m, "f64::atan2", Class::C, f64::atan2, &at);

    // hypot: 2D vector lengths — contact offsets, joint anchors, ray directions, impulses.
    // Pythagorean triples (exact answers), unit-scale and metre-scale sweeps, the special grid,
    // and pairs spanning the range (overflow and underflow avoidance).
    let mut hy = grid(&specials());
    for (a, b) in [
        (3.0, 4.0),
        (5.0, 12.0),
        (8.0, 15.0),
        (7.0, 24.0),
        (20.0, 21.0),
    ] {
        hy.push((a, b));
        hy.push((-a, b));
        hy.push((b, -a));
    }
    hy.extend(uniform_pairs(0x4791, 600, -1.0, 1.0));
    hy.extend(uniform_pairs(0x4792, 400, -1e3, 1e3));
    hy.extend(pairs(0x4793, 400, -300.0, 300.0));
    hy.extend(pairs(0x4794, 200, -160.0, -150.0));
    binary(&mut m, "f64::hypot", Class::C, f64::hypot, &hy);

    // exp: the spring transition takes exp(−α·dt) and exp(r·dt) with r < 0: arguments in
    // [−745, 0] above all, near 0 for small steps; the full finite range and its edges.
    let mut ex = specials();
    ex.extend(sweep(0xe001, 600, -50.0, 0.0));
    ex.extend(sweep(0xe002, 300, -745.2, 0.0));
    ex.extend(sweep(0xe003, 200, -1e-3, 0.0));
    ex.extend(sweep(0xe004, 200, -745.2, 709.8));
    ex.extend((-745..=709).step_by(7).map(f64::from));
    ex.extend([
        709.782_712_893_384,
        709.782_712_893_384_1,
        -708.396_418_532_264_1,
        -745.133_219_101_941_1,
        -745.133_219_101_941_2,
        1e-300,
        -1e-300,
    ]);
    ex.extend(exp_edges());
    unary(&mut m, "f64::exp", Class::C, f64::exp, &ex);

    // powf: a radial force field divides by (d² + ε²)^(falloff/2), so a base ≥ 10⁻⁴ and an
    // exponent near 0..2; then random positive pairs, the special grid, and negative bases
    // with integer exponents.
    let mut pf: Vec<(f64, f64)> = Vec::new();
    let mut rng = Rng::new(0x9001);
    for _ in 0..400 {
        let base = 1e-4 + rng.log_uniform(-6.0, 6.0);
        pf.push((base, rng.uniform(0.0, 2.0)));
    }
    for k in 0..=8 {
        let e = f64::from(k) * 0.25;
        for _ in 0..20 {
            pf.push((1e-4 + rng.log_uniform(-6.0, 4.0), e));
        }
    }
    for _ in 0..300 {
        let x = rng.log_uniform(-30.0, 30.0);
        pf.push((x, rng.uniform(-10.0, 10.0)));
    }
    pf.extend(grid(&specials()));
    for _ in 0..50 {
        let x = -rng.uniform(0.1, 10.0);
        #[allow(clippy::cast_precision_loss)]
        let y = rng.int(-5, 5) as f64;
        pf.push((x, y));
    }
    binary(&mut m, "f64::powf", Class::C, f64::powf, &pf);

    m
}
