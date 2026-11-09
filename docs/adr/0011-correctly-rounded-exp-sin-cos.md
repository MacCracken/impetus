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
   integer operations only, so the bits are the same on every target. They follow Ziv's strategy:
   - **fast path**, in double-double:
     - exp is k·ln 2 + r against a three-part ln 2, with 27 Taylor terms, for −708 ≤ x ≤ 709.7;
     - sin and cos reduce |x| − k·π/2 against a four-part π/2 and take 16 Taylor terms, for |x| ≤ 2^20.

     The result is returned only when a rounding test proves that every value within the error bound (2^-88
     relative; 2^-74 for sin y) rounds to the same double.
   - **slow path**, in 364-bit fixed point:
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
    aarch64 alike. On the ordinary set, the fast path decides all 90,000.
- **Negative**
  - impetus owns about 700 lines of correctly rounded math (plus 112 generated) and its error analysis, until ganita provides it (port
    plan §8).
  - Cost: the fast paths take 2.6 µs (exp) and 3.3 µs (sin_cos) against the polyfills' 64 and 83 ns; the slow
    path takes 18–27 µs. The spring caches its transition, so a steady step is unchanged (29.5 ns), but a spring's
    first step pays the primitives: `spring_settle_frames` 1.80 → 4.44 µs (Rust 1.47 µs).
  - 2D rotation (M2/M3) calls sin_cos per body, so the fast paths need to get faster before then.
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
