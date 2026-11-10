# 0011 — Correctly rounded exp, sin and cos, gated exactly against a corrected oracle

**Status**: Accepted
**Date**: 2026-11-09

## Context

The port-parity rule (`docs/development/port-parity.md` §2) sets a class-C function's bound before its first
comparison: K_F = 2·S·c + 2, where S is the most its outputs move per one-ulp change in a transcendental result,
and c is the number of such calls on its path. If K_F > 64, the rule says to make the primitive exact instead.

M1b measured `Spring::step`'s S on every one of its 800 golden rows (`port-parity.md` §7, from a transcription the
generator checks bit for bit against the oracle). S is above 32 on more than half the rows and reaches 4.5·10^18.
Two things drive it:
- `position = target + y` cancels when the spring sits near 0 and its target is far;
- the over-damped branch divides `e1 − e2` by `r1 − r2` next to the critical point.

The port's primitives then were the exp polyfill and the builtin sin/cos, which are 1 ulp from glibc on 7.5 % and
about 2 % of arguments. On the sensitive rows that ulp became up to 2.4·10^7 ulp in the output. The 800 rows could
only be measured, not gated.

glibc 2.44, the oracle's libm, is not correctly rounded either. Against MPFR, it misrounds 0.105 % of random exp
arguments in [−700, 700], 0.14 % of sin arguments and 0.15–0.17 % of cos arguments. On the spring grid it misrounds
2 of the 377 sin calls and none of the exp or cos calls.

kimiya met the same rule with its Peng–Robinson functions. Its ADR 0005 made acos, cos, ln and x^2.5 correctly
rounded. Its ADR 0006 then made correct rounding the accuracy target, compared against a "corrected oracle" (the
user: "WE WANT TO BE BETTER THAN glibc"). The user chose the same remedy here ("Exact exp/sin/cos now", 2026-11-09).

## Decision

1. **exp, sin and cos are correctly rounded** (`src/crmath.cyr`): round-to-nearest-even of the exact value on every
   double argument, subnormal results and overflow included. They use + − × ÷, the f64 rounding builtins and
   integer operations only, so the bits are the same on every target. They follow Ziv's strategy, in three phases.
   A phase's result is returned only when a rounding test proves that every value within its error bound rounds
   to the same double.
   - **Phase 1**, table-driven, in double with a few exact products:
     - exp is 2^k·2^(j/64)·exp(r), with |r| ≤ 2^-7.5 and seven polynomial terms;
     - sin and cos reduce against a three-part π/2, then use the sin and cos of i/64 with |z| ≤ 2^-7.

     Its bound is 2^-62. The analysis gives 2^-64.5 to 2^-65, and the measured worst over 200,000 arguments
     each is 2^-67 (exp), 2^-66 (sin) and 2^-67 (cos). It decides about 99.7 % of arguments in its range
     (−708 ≤ x ≤ 709.7; |x| ≤ 2^20).
   - **Phase 2**, in double-double:
     - exp is k·ln 2 + r against a three-part ln 2, with 27 Taylor terms;
     - sin and cos reduce against a four-part π/2 and take 16 Taylor terms.

     Its bounds are 2^-88, and 2^-74 for a sin y.
   - **Phase 3**, the slow path, in 364-bit fixed point:
     - exp: an exact reduction to 0 ≤ r < ln 2, its Taylor series, then 2^k;
     - sin and cos: Payne–Hanek reduction against 1,568 bits of 2/π, then the series on [0, π/2).

     It is correct to about 2^-290 relative. The hardest double arguments of exp, sin and cos need about 2^-120
     (Lefèvre–Muller), and none is an exact midpoint (Lindemann–Weierstrass), so one rounding is always the
     correct one.

   The tables are generated (`scripts/oracle/crtables.py`, written by `validate.py --write`) from π and ln 2
   computed exactly in integer arithmetic. `validate.py` checks those against MPFR to 2,048 bits on every run.
   `_im_exp`, `_im_sin_cos`, `_im_sin` and `_im_cos` (`src/mathx.cyr`) call them, so the spring transition does too.
2. **The reference is the corrected oracle**, as kimiya's ADR 0006:
   - A class-C row's expected value is the oracle's own operations with every exp, sin and cos result correctly
     rounded (MPFR 4.2, through ctypes: `scripts/oracle/mpfr_ref.py`, copied from kimiya).
   - Where glibc rounded every call of a row correctly, that is the oracle's row and nothing changes.
   - Where it didn't, `validate.py --write` writes the corrected outputs, with the calls glibc misrounded, into
     `tests/golden/<module>.cr.tsv`. The harness then compares the port against the corrected outputs only, and
     counts the row as **accepted (ADR 11)**.
   - A port value that equals glibc's misrounded value there is a failure. kimiya's transitional rule, which
     measures such rows, is off.
3. **Class C on these primitives is compared exactly**: the bound is 0 for `f64::exp`, `f64::sin`, `f64::cos`,
   `f64::sin_cos` and `Spring::step`. Both sides evaluate the same operations in the same order on the same
   correctly rounded results, so any difference is a port defect. The sensitivity S stays a diagnostic (the step
   rows' notes) and no longer sets their bound.
4. **Scope.** atan2, hypot and powf keep their glibc-relative K = 2 until a function that needs them is
   ill-conditioned or they are made correctly rounded. The 3D path's hisab primitives stay class D (ADR 0007).
5. **Grids keep the hard rows**: the math grid gains every threshold of the new code, each straddled (2^-27, 0.78,
   2^20, the doubles nearest kπ/2 for k ≤ 8, 6381956970095103·2^797; and 2^-55, −708, 709.7, 710, −746 for exp).

## Consequences

- **Positive**
  - All 800 spring step rows are gated and bit-identical to the (corrected) oracle: 799 to the oracle itself, and 1
    where glibc's sin misrounds. That holds on x86_64 ELF, x86_64 PE and aarch64.
  - The math rows for exp, sin, cos and sin_cos are 0 ulp. 10 rows are corrected (9 math, 1 spring), and on every
    one the port equals MPFR.
  - The port is at least as accurate as the oracle everywhere and more accurate where glibc misrounds. The platform
    libm no longer matters for these three.
  - Verification beyond the corpus (scratch probes, not committed):
    - 180,062 seeded arguments over the whole range: huge sin/cos arguments, subnormal and overflow exp,
      the closest double to π/2's multiples;
    - 90,000 in the fast paths' ranges;
    - 1,200 arguments where glibc misrounds.

    All are equal to MPFR on the slow path alone and through the full functions, the 180,062 on x86_64 and
    aarch64 alike. On the ordinary set, phase 1 decides 89,754 of the 90,000 results, phase 2 the other 246, and
    none needs the slow path.
- **Negative**
  - impetus owns about 700 lines of correctly rounded math (plus 112 generated) and its error analysis, until ganita provides it (port
    plan §8).
  - Cost:
    - Phase 1 takes 97 ns (exp) and 187 ns (sin_cos), against the polyfills' 48–64 and 75–83 ns.
    - Phase 2 takes about 2.6 µs and 3.3 µs, and the slow path 17–25 µs; the probes measured phase 2 on about
      0.3 % of ordinary arguments.
    - The spring caches its transition, so a steady step is unchanged (28 ns). A spring's first step pays the
      primitives once: `spring_settle_frames` is 1.86 µs, against 1.80 µs on the polyfills and Rust's 1.47 µs.
    - The first version had only phases 2 and 3, and `spring_settle_frames` took 4.44 µs.
  - The tooling needs the system MPFR library (development only).
- **Neutral**
  - A Rust 1.x user comparing results will see the last bit differ where glibc misrounds (about 0.1–0.2 % of exp,
    sin and cos calls). The CHANGELOG says so.
  - The corrected oracle depends on glibc 2.44's rounding. `validate.py` recomputes it, and the generator's
    `--check` refuses a corpus from another glibc.

## Alternatives considered

- **A bound above 64**: the rule refuses it. K = 10^19 ulp says nothing.
- **Correctly rounded primitives, gated against glibc** (kimiya's ADR 0005 road, where S counts only glibc's
  misrounded calls): the step rows where glibc misrounds a sensitive call (1 on this grid, more on a consumer's
  inputs) could never pass. It also keeps glibc's errors as the target, which kimiya's ADR 0006 and the user
  rejected.
- **Double-double with one final rounding** (kimiya's `_km_cos`, `_km_ln`): right on all but about 2^-47 of inputs,
  with no proof. The rounding test plus the fixed-point fallback makes every result provably correct for the price
  of rare slow calls.
- **Transcribing glibc's exp, sin and cos**: not in the oracle's dependency graph, FMA-dispatched on x86_64, and it
  would copy glibc's errors.
- **A slow path only** (no double-double): about 20 µs per call, so every cache miss of a spring's transition and
  every future rotation would pay it.
- **Double-double as the first phase** (this ADR's first version): correct, but 2.6–3.3 µs a call, 30–40× the
  polyfills. The table-driven phase 1 brought it to about 2×.

## Amendment — M2d (2026-11-09): hypot joins exp, sin and cos

**Context.** The 2D raycasts normalise the ray direction and their normals with `f64::hypot`, and M3's joints call
it six times (lengths, impulse magnitudes, the rope and distance biases). The port's `hypot` was ganita's. It
differs from glibc on 414 of the 1,715 golden math rows (411 by 1 ulp, 3 by 2), so every row through it could only
have a measured bound. glibc's own `hypot` misrounds 0.31 % of 300,000 seeded arguments against MPFR (CPython's
`math.hypot` is not glibc's and was no reference), and 12 of the golden rows. The project's aim is accuracy across
the Cyrius libraries, not parity with glibc's limitations (user, 2026-11-09).

**Decision.** `src/crmath.cyr` gains `_im_cr_hypot`, correctly rounded on every input. `_im_hypot` calls it, and
`f64::hypot` rows are gated at 0 ulp against the corrected oracle, as exp, sin and cos are. It works in two phases:
- **Phase 1.** Both operands are scaled by a power of two so the larger is in [1, 2), so the scaling is exact.
  x² + y² is formed in double-double from exact products, its square root is corrected once by the residual, and
  the result is kept when the rounding test decides it. The relative error is below 2^-100, tested against 2^-96.
- **The slow path** handles undecided cases, and results in the first normal binade or below, whose spacing is the
  subnormal grid's. It forms x² + y² exactly in the 364-bit fixed point and compares it with the exact squares of
  the midpoints next to the candidate.

Unlike exp, sin and cos, `hypot` has **exact midpoints**: a Pythagorean triple with a 54-bit odd hypotenuse. The
exact comparison rounds them to even.

**Consequences.**
- **Verified against MPFR.** All 404,234 probe pairs match, on x86_64 and aarch64, which produce identical output.
  The pairs cover near and far exponents, the whole range with subnormal results and overflow, the IEEE specials
  (an infinity wins over a NaN), and 4,134 exact midpoints.
- **The golden rows.** All 1,715 golden rows are at 0 ulp: 1,703 equal glibc, and 12 equal `math.cr.tsv`'s
  corrections. ganita's `hypot` stays as a shadow line.
- **Cost.** 168 ns a call against ganita's 33 ns (`math/hypot_cr`), between exp (104 ns) and sin_cos (201 ns).
- **For a Rust 1.x user:** results differ in the last bit wherever glibc misrounds a `hypot`.
- **Upstream.** Like exp, sin and cos, `hypot` belongs in ganita once (port plan §8).

## Amendment — M3d (2026-11-09): pow joins exp, sin, cos and hypot

**Context.** The world's radial force fields divide by `(d² + ε²)^(falloff/2)`, the port's one `powf`. The port's
`pow` was ganita's, which misrounds 9.62 % of 100,000 ordinary arguments (x in [10⁻⁴, 10⁴], y in [−10, 10]; glibc
0.096 %), and returns +∞ for `pow(−1, DBL_MAX)` (ADR 0009). So the `f64::powf` rows had a 2-ulp bound, and the world's
rows could not have been held at 0. The aim is accuracy, not parity with glibc (user, 2026-11-09). The user chose a
correctly rounded pow before the world.

**Decision.** `src/crmath.cyr` gains `_im_cr_pow`, correctly rounded on every input, with C99 Annex F's specials.
`_im_powf` calls it, and the `f64::powf` rows and the world's rows are gated at 0 ulp against the corrected oracle.
- **Phases 1 and 2.**
  - log x is computed in double-double by the atanh series of s = (m − 1)/(1 + m), on 0.75 ≤ m < 1.5, so it stays
    relative near x = 1.
  - Then y·log x by TwoProduct, then exp of the high word times exp of the low word.
  - Phase 1 carries the log to 2^-73 (measured worst 2^-73.2), with exp's table phase, tested against
    2^-61 + |y·log x|·2^-71. It decides over 99 % of ordinary arguments.
  - Phase 2 carries the log to 2^-100 (measured worst 2^-103.4), with exp's phase 2, tested against 2^-80.
- **The slow path** first runs an exactness test in integer arithmetic. x^y is a dyadic rational exactly when, with
  x = a·2^u (a odd), y is a non-negative integer, or y = Y/2^s (Y > 0 odd) with a a perfect 2^s-th power and 2^s
  dividing u. Such a value is computed exactly and rounded once when its odd part is below 2^63. Otherwise log x,
  y·log x and exp go through the 364-bit fixed point (relative error below 2^-290). exp's slow path now takes a
  fixed-point argument, so pow shares it.

pow has **exact midpoints**, for example 25^11.5 = 5^23, a 54-bit tie. The exactness test rounds them to even, as it
does the subnormal ties glibc 2.44 misrounds (`port-parity.md` §8). Unlike exp, sin and cos, pow's worst cases are not
known in general:
- An inexact dyadic value of B > 63 bits is at least 2^-B from every midpoint, so the slow path decides it outright for
  B below about 290.
- Beyond that, and for every other input, it relies on no x^y lying within 2^-290 of a midpoint without being one. The
  published hard cases need about 2^-120.

**Consequences.**
- **Verified against MPFR.** 1,290,769 pairs on the final code match, in three seeded sets. They cover ordinary
  arguments, x near 1 with huge y, results at the overflow and subnormal edges, integer and dyadic exponents, exact
  cases and constructed midpoints, negative bases, tiny y, random bit patterns and the specials. An earlier design
  matched 1,176,923 more.
- **The golden rows.** All 1,030 `f64::powf` rows are at 0 ulp; glibc is correctly rounded on all of them, so no
  correction file is needed. The ADR 0009 row is now an ordinary match.
- **Cost.** 755 ns a call against ganita's 200 ns (`math/pow_cr`). The first design, phase 2 alone, cost 5.4 µs.
- **For a Rust 1.x user:** results differ in the last bit wherever glibc misrounds a `pow`.
- **Upstream.** Added to ganita's correctly rounded tier filing (2026-11-09), with 8 more repro rows.
