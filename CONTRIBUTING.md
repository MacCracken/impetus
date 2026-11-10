# Contributing to impetus

Thank you for considering a contribution! This document covers the development
workflow, coding standards, and review process.

## Development Setup

impetus is a Cyrius library, ported from the Rust crate 1.4.0 (the plan of record is
[`docs/development/impetus-port-plan.md`](docs/development/impetus-port-plan.md)). The Rust crate in
`rust-old/` is the parity oracle and is never edited. You need the Cyrius toolchain at the version
pinned in `cyrius.cyml`, and Rust 1.99 or newer only to run the oracle and the generators in
`scripts/oracle/`.

```bash
# Clone the repository
git clone https://github.com/MacCracken/impetus.git
cd impetus

# The Cyrius toolchain version is pinned in cyrius.cyml ([package].cyrius)
cyrius deps          # vendor lib/ (stdlib leaves, and hisab for the tests)
cyrius audit         # fmt + lint + docs + tests + bench
```

## Pull Request Process

1. **Fork and branch** — create a feature branch from `main`.
2. **Keep commits focused** — one logical change per commit.
3. **Write tests** — a change to ported code keeps its golden rows and ported Rust tests passing;
   new physics (no oracle) needs reference tests; bug fixes need a regression test. A behavior
   change that diverges from `rust-old/` needs an ADR first.
4. **Run CI locally** before pushing: the cleanliness check in [`CLAUDE.md`](CLAUDE.md)
   (`cyrius test` must be warning-free, plus `test --bounds`, `deps --locked` and `audit`).
5. **Open a PR** against `main` with a clear description of the change.
6. **Address review feedback** — maintainers may request changes before merging.

## Code Style

- `cyrius fmt` and `cyrius lint` clean (enforced by `cyrius audit` in CI).
- Public functions have doc comments; `#must_use` on pure functions.
- Public enums have explicit, append-only values (the Cyrius form of `#[non_exhaustive]`).
- No aborts in `src/`: no `+?`, `-?`, `*?`, `assert_fatal`, `panic` or `result_unwrap`.
- Public globals carry `IM_`, private helpers `_im_`, public structs `Im<Name>`.

## Testing Requirements

- `tests/*.tcyr` — parity and reference tests, auto-discovered by `cyrius test`. A ported Rust test
  keeps its Rust name as its `test_group`, so the ledger can find it.
- `tests/*.bcyr` — benchmarks; every Rust benchmark has a twin, recorded in
  `docs/benchmarks-rust-v-cyrius.md`.
- `tests/*.fcyr` — the fuzz targets, on both profiles.
- The docs' `cyrius` code blocks are programs, run by `scripts/doc_examples.py`.
- The Rust oracle runs read-only from `rust-old/`:

```bash
cyrius test
cyrius bench
cyrius fuzz
python3 -I scripts/surface_parity.py .               # every Rust item has its Cyrius name
python3 -I scripts/rust_test_ledger.py . --check     # every Rust test is accounted for
python3 -I scripts/bench_record.py . --check         # every Rust bench has its twin
python3 -I scripts/doc_examples.py .                 # the README's and guides' code blocks run
cargo test --manifest-path rust-old/Cargo.toml --features serialize,oracle   # Rust oracle, 2D
cargo test --manifest-path rust-old/Cargo.toml --all-features                # Rust oracle, 3D
```

## Commit Messages

Use clear, imperative-mood commit messages:

```
add spring damping parameter validation

fix trimesh AABB computation for rotated bodies
```

## License

By contributing, you agree that your contributions will be licensed under the
same license as the project (see [LICENSE](LICENSE)).
