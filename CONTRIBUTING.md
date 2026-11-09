# Contributing to impetus

Thank you for considering a contribution! This document covers the development
workflow, coding standards, and review process.

## Development Setup

impetus is being ported from Rust to Cyrius (see
[`docs/development/impetus-port-plan.md`](docs/development/impetus-port-plan.md)). The Rust 1.4.0
crate in `rust-old/` is the parity oracle and is not edited.

```bash
# Clone the repository
git clone https://github.com/MacCracken/impetus.git
cd impetus

# The Cyrius toolchain version is pinned in cyrius.cyml ([package].cyrius)
cyrius deps          # vendor lib/ (stdlib leaves and hisab)
cyrius audit         # fmt + lint + docs + tests + bench
```

## Pull Request Process

1. **Fork and branch** — create a feature branch from `main`.
2. **Keep commits focused** — one logical change per commit.
3. **Write tests** — ported code needs its golden parity rows and its ported Rust tests; bug fixes
   need a regression test.
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

- `tests/*.tcyr` — parity and reference tests, auto-discovered by `cyrius test`.
- `tests/*.bcyr` — benchmarks, each mirroring a Rust benchmark and recorded in
  `docs/benchmarks-rust-v-cyrius.md`.
- The Rust oracle runs read-only from `rust-old/`:

```bash
cyrius test
cyrius bench
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
