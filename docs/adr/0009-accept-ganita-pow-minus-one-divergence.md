# 0009 — Accept ganita's pow(−1, DBL_MAX) divergence

**Status**: Superseded by ADR 0011's pow amendment (2026-11-09)
**Date**: 2026-11-09

> **Superseded.** The port's `pow` is now crmath's correctly rounded `_im_cr_pow`, not ganita's. It returns 1 for
> `pow(-1, DBL_MAX)`, as glibc does, so the row is an ordinary match and the harness no longer lists it. The ganita
> defect stays filed upstream.

## Context

The M0 `math` golden module records Rust std `f64::powf` (glibc 2.44) over special values. For
`powf(-1.0, 1.7976931348623157e308)` glibc returns **1.0** (DBL_MAX is an even integer); ganita
1.2.15's `ganita_f64_pow` returns **+∞**. It is ganita's |x| == 1 defect for huge even exponents,
filed upstream by tanmatra (`ganita/docs/development/issues/2026-11-08-pow-minus-one-huge-even-exponent.md`)
and accepted in the same way by tanmatra ADR 0007 and kimiya ADR 0004. It is the only class-C row
beyond K = 2 ulp on any target: x86_64 ELF, x86_64 PE and aarch64 (`port-parity.md` §2).

## Decision

Accept that one row. `tests/parity_2d.tcyr` `_par_accepted` lists `f64::powf` with input
`f:bff0000000000000 f:7fefffffffffffff` under ADR 9. It is reported on every run and doesn't fail
its module. No other row is accepted, and the K = 2 bound is not widened.

## Consequences

- **Positive** — the bound stays tight for every other `powf` row.
- **Negative** — none for impetus's results. Its one `powf` call is a radial force field's
  `(d² + ε²)^(falloff/2)`, whose base is at least ε² = 10⁻⁴.
- **Neutral** — remove the entry when ganita fixes the defect and the pin moves.

## Alternatives considered

- **Drop the row from the grid**: rejected — the row documents a real primitive divergence.
- **Transcribe glibc's pow**: unnecessary — no impetus input reaches the defect.
