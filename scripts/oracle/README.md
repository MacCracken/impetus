# scripts/oracle: what the port derives from the Rust oracle

`oracle` drives the frozen Rust impetus 1.4.0 in [`rust-old/`](../../rust-old/), with its hidden
`oracle` hooks (port plan §4 step 0), and writes two kinds of output:
- **The golden corpus, `tests/golden/<module>.tsv`** (port plan §4 step 5). It has one row per
  call, with every float as its IEEE-754 bit pattern. The Cyrius parity harnesses
  (`tests/parity_2d.tcyr`, and `tests/parity_3d.tcyr` from M4) replay every row against the
  port and check it within its tolerance class.
- **The generated Cyrius sources**, `src/constants.cyr` now and data modules as milestones need
  them (step 6). No value the port reads is typed by hand.

```sh
cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml                   # 2D + shared: write
cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml -- --check        # verify; exit 1 on any difference
cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml \
    --no-default-features --features b3 [-- --check]                                     # 3D
python3 -I scripts/oracle/validate.py .                                                  # independent re-check
```

The format, the comparison rules and the generator's structure are kimiya's
(`/home/macro/Repos/kimiya/scripts/oracle/README.md`), which are tanmatra's: every port's
harness reads the same files. This README records what is specific to impetus.

## Rules

- **`rust-old/` is never built in place or edited.** This crate depends on it by path.
  `Cargo.lock` was seeded from `rust-old/Cargo.lock`, so every shared dependency is the version
  the oracle was tested with (hisab 1.4.0, which links glam 0.29.3).
- **Two builds.** The oracle's `2d` and `3d` features are exclusive, so the crate is built with
  `b2` (default) or `b3`. Each build **owns** its files: `b3` every `b3_*.tsv`; `b2` everything
  else (the shared modules, `b2_*.tsv` and the generated `.cyr` files). `modules.txt` lists the
  union, so either build keeps it current; the stale and platform checks look only at owned files.
- **The output is deterministic.** Two runs give byte-identical files, and `--check` passes on
  any machine with the recorded glibc. Grids use only + − × ÷, `floor`, decimal parsing, bit
  construction and seeded xorshift; never a transcendental std method, never a timestamp or a
  hash-map order.
- **glibc, not the `libm` crate.** impetus calls the std f64 methods, so `sin_cos`, `atan2`,
  `hypot`, `exp` and `powf` are the host glibc's bits (`src/glibc.rs`). Every TSV header records
  the glibc and the CPU class (`glibc 2.44 x86_64 fma`), and `--check` refuses a corpus from
  another one.
- **The corpus is regenerated only from the oracle.** A wrong oracle value is fixed in Rust as
  1.4.x, then regenerated. It is never patched in a `.tsv`.
- **Modules arrive with their milestone.** Each one starts by adding its golden emitter
  (`src/m_<module>.rs`, the grid from the port plan §6) and, where it has data, its Cyrius
  emitter (`src/t_<module>.rs`), registered in `src/main.rs`.

| Module | Build | Milestone | Golden rows | Golden emitter | Generated Cyrius |
|---|---|---|---|---|---|
| `constants` | b2 | M0 | 3 | `src/m_constants.rs` | `src/constants.cyr` (`src/t_constants.rs`, with the completeness proof) |
| `math` | b2 | M0 | 18,919 | `src/m_math.rs` | — |

## Format 1

As kimiya's README, *Format 1* and *Comparison*: five header lines, then one row per call,

```text
fn <TAB> class <TAB> inputs <TAB> outputs <TAB> note
```

with typed tokens (`f:` bits, `i:`, `e:`, `s:`, `n:`, `E:`, `v:`) and classes A (exact),
B (≤ 2 ulp, expected 0), C (≤ K ulp, K per function from step 8) and D (relative 1e-12).
impetus adds two classes at M3 (port plan §5), as new row shapes rather than new tokens:
**S** single-step rows (a state dump in, a state dump out) and **R** reference scenarios, whose
format is specified when M3 adds the first emitter.

impetus specifics:
- `const` rows hold integers (`i:`): every public constant is a `u32` limit.
- `f64::sin_cos` has two outputs, sin then cos. `validate.py` checks they equal the separate
  `f64::sin` and `f64::cos` rows.
- `f64::powi2` and `f64::powi3` are `x.powi(2)` and `x.powi(3)` with **literal** exponents, as
  impetus writes them: LLVM expands them to products, which is what the port must reproduce.

## `validate.py`

Standard-library Python, run with `-I`. It never calls the oracle:
- every file parses (header, row count, fields, classes, tokens), and `modules.txt` lists
  exactly the files present;
- `constants.tsv` equals the `pub const` literals in `rust-old/src`;
- every class-A and class-B `math` row is recomputed in Python in Rust's operation order (7,197
  rows at M0): sqrt, abs, floor, ceil, min/max (NaN-ignoring; the second operand when equal,
  as rustc 1.99.1 lowers them), clamp, is_nan, is_finite, to_radians, powi2, powi3;
- class-C rows are checked for consistency: `sin_cos` equals `sin` and `cos`, and hypot of a
  Pythagorean triple is exact.

A tampered row makes it exit 1.
