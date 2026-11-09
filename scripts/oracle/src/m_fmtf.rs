//! `fmtf.tsv` — f64 to text, as Rust's `Display` and serde_json 1.0.154 write it (ADR 0010).
//!
//! Both print the shortest digits that read back to the same double, the closest to its exact
//! value; they differ in layout (`src/fmtf.cyr`). The grid covers every binade and its neighbours,
//! every power of ten from 1e-323 to 1e308 and its neighbours, the 2^53 integer edge, serde_json's
//! −5/+15 exponent thresholds, short decimals, 300 exact ties (where the two printers differ: core
//! rounds up, zmij to even), and seeded random bit patterns, each with both signs for part of the grid.

use crate::grids::{Rng, pow2, specials};
use crate::tsv::{Class, Module, f, s};

fn next_up(x: f64) -> f64 {
    if x.is_nan() || x == f64::INFINITY {
        return x;
    }
    if x == 0.0 {
        return f64::from_bits(1);
    }
    let b = x.to_bits();
    f64::from_bits(if x > 0.0 { b + 1 } else { b - 1 })
}

fn next_down(x: f64) -> f64 {
    -next_up(-x)
}

/// The significant digits of a printed number, layout removed: "2.9802322387695313e-8" and
/// "0.000000029802322387695313" are both "29802322387695313"; "100.0" and "1e2" are both "1".
fn digit_string(text: &str) -> String {
    let mant = text.split(['e', 'E']).next().unwrap_or_default();
    let digits: String = mant.chars().filter(char::is_ascii_digit).collect();
    digits
        .trim_start_matches('0')
        .trim_end_matches('0')
        .to_string()
}

pub fn emit() -> Module {
    let mut m = Module::new("fmtf");
    let mut xs = specials();
    for k in -1074..=1023 {
        let x = pow2(k);
        xs.push(x);
        if k % 3 == 0 {
            xs.push(next_up(x));
            xs.push(next_down(x));
        }
    }
    for e in -323..=308 {
        let x: f64 = format!("1e{e}").parse().unwrap_or(0.0);
        xs.extend([x, next_up(x), next_down(x)]);
    }
    for d in -4i64..=4 {
        #[allow(clippy::cast_precision_loss)]
        xs.push((9_007_199_254_740_992_i64 + d) as f64);
    }
    for t in [
        "1e15",
        "1e16",
        "9999999999999998",
        "1e-5",
        "1e-6",
        "0.00001",
        "0.000009999999999999999",
    ] {
        let x: f64 = t.parse().unwrap_or(0.0);
        xs.extend([x, next_up(x), next_down(x)]);
    }
    let mut rng = Rng::new(0xd15c);
    for _ in 0..3000 {
        let digits = rng.int(1, 7);
        let mant = rng.int(1, 10_i64.pow(u32::try_from(digits).unwrap_or(1)) - 1);
        let exp = rng.int(-30, 30);
        let x: f64 = format!("{mant}e{exp}").parse().unwrap_or(0.0);
        xs.push(x);
    }
    for _ in 0..6000 {
        let bits = rng.bits() & 0x7fff_ffff_ffff_ffff;
        let x = f64::from_bits(bits);
        if x.is_finite() {
            xs.push(x);
        }
    }
    // Exact ties: values halfway between their two closest shortest candidates, where Rust's core
    // printer rounds up and zmij to the even digit. Found by comparing the two printers' digits
    // over odd mantissas times powers of two (2^-25 is the first).
    let mut ties = 0;
    'search: for k in 1..=80 {
        for m in (1u64..4096).step_by(2) {
            #[allow(clippy::cast_precision_loss)]
            let x = m as f64 * pow2(-k);
            if digit_string(&format!("{x:e}"))
                != digit_string(&serde_json::to_string(&x).unwrap_or_default())
            {
                xs.push(x);
                xs.push(-x);
                ties += 1;
                if ties >= 300 {
                    break 'search;
                }
            }
        }
    }
    let negs: Vec<f64> = xs.iter().step_by(4).map(|x| -x).collect();
    xs.extend(negs);
    for &x in &xs {
        m.row("f64::fmt_display", Class::A, &[f(x)], &[s(format!("{x}"))]);
        let json = serde_json::to_string(&x).unwrap_or_default();
        m.row("serde_json::f64", Class::A, &[f(x)], &[s(json)]);
    }
    m
}
