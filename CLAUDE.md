# Impetus — Claude Code Instructions

> **Core rule**: this file is **preferences, process, and procedures** —
> durable rules that change rarely. Volatile state (current version,
> module line counts, port progress, test counts, consumers) lives in
> [`docs/development/state.md`](docs/development/state.md).
> Do not inline state here.

## Project Identity

**Impetus** (Latin: driving force) — Physics engine — 2D/3D rigid bodies, collision, constraints, spatial hash, particles

- **Type**: Flat library (Cyrius), ported from the Rust crate 1.4.0; the port shipped as 2.0.0
- **License**: GPL-3.0-only
- **Language**: Cyrius. The toolchain is pinned in `cyrius.cyml` `[package].cyrius`
- **Version**: `VERSION` at the project root is the source of truth. Don't inline the number here
- **Rust oracle**: impetus 1.4.0 plus the unreleased `oracle` hooks, frozen at `rust-old/`. It stays
  the parity reference until the deferred bridges are ported or dropped (port plan §3.2)
- **Plan of record**: [`docs/development/impetus-port-plan.md`](docs/development/impetus-port-plan.md),
  with [ADR 0007](docs/adr/0007-cyrius-port-hisab-first.md) (hisab-first) and
  [ADR 0008](docs/adr/0008-port-api-divergences.md) (2.0 API changes)
- **Standards**: [First-Party Standards](https://github.com/MacCracken/agnosticos/blob/main/docs/development/applications/first-party-standards.md) · [First-Party Documentation](https://github.com/MacCracken/agnosticos/blob/main/docs/development/applications/first-party-documentation.md)

## Goal

impetus owns rigid-body physics for AGNOS: 2D and 3D bodies, collision detection, contact and
joint constraints, spatial queries, physics particles and standalone springs. Deterministic
stepping (fixed timestep, ordered iteration) and serializable state are part of the contract.

## Consumers

kiran (game physics), joshua (simulation) and aethersafha (window physics) use or have used the
Rust crate; none consumes the Cyrius port yet. The current list is in `state.md`.

## Quick Start

```sh
cyrius deps                                   # resolve dependencies (vendors lib/, incl. hisab)
cyrius build src/main.cyr build/impetus       # compile
cyrius test                                   # run tests/*.tcyr (parity + reference tests)
cyrius bench                                  # run tests/*.bcyr
cargo test --manifest-path rust-old/Cargo.toml --features serialize,oracle   # the Rust oracle (2D), read-only
cargo test --manifest-path rust-old/Cargo.toml --all-features                # the Rust oracle (3D), read-only
cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml -- --check   # golden corpus (2D + shared) up to date
cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml --no-default-features --features b3 -- --check   # (3D)
python3 -I scripts/oracle/validate.py .                                         # independent check of the corpus
python3 -I scripts/oracle/validate.py . --write                                 # regenerate src/crmath_data.cyr and tests/golden/*.cr.tsv (needs MPFR)
```

## Development Process

### P(-1): Scaffold Hardening (before any new features)

0. Read the roadmap, the port plan, the CHANGELOG and the open issues
1. Test + benchmark sweep of the existing code
2. Cleanliness check (below)
3. Get baseline benchmarks
4. Internal deep review — gaps, optimizations, security, logging/errors, docs
5. External research — domain completeness, best practices, world-class accuracy
6. Cleanliness check — must be clean after the review
7. Additional tests/benchmarks from the findings
8. Post-review benchmarks — prove the wins
9. Repeat if heavy

### Cleanliness check

```sh
cyrius fmt --check <files>
cyrius lint <files>
cyrius vet src/main.cyr
cyrius build src/main.cyr build/impetus
cyrius test                 # must be warning-free, not just passing
cyrius test --bounds
cyrius bench
cyrius distlib --all && cyrius distlib --check   # once [lib] exists (M1)
cyrius deps --locked
cyrius audit
```

`cyrius test` must be **warning-free**. The f64 type diagnostics come only from the
compiles that `test` and `build` drive, never from `cyrius lint`. Each one is a bug:
- *integer arithmetic with an f64 right operand*
- *f64 arithmetic with a non-f64 right operand*
- *an integer stored into an f64 slot*
- *comparison mixes f64 and integer*

### Work Loop (continuous)

1. Work phase — one module at a time, in the order the port plan gives
2. Cleanliness check
3. Test + benchmark additions: golden parity rows, single-step rows, reference tests, benches
4. Run benchmarks, recorded against the Rust numbers (`docs/benchmarks-rust-v-cyrius.md`)
5. Internal review — performance, memory (flat heap per step), correctness, and a
   surface-parity diff against `rust-old/src/`
6. Cleanliness check — must be clean after the review
7. Deeper tests/benchmarks from the review
8. Benchmarks again — prove the wins
9. If the review was heavy, return to step 5
10. Documentation — CHANGELOG, `state.md`, roadmap, the port plan's tracking table
11. Version check — `VERSION` and `cyrius.cyml` (`${file:VERSION}`) in sync
12. Return to step 1

### Task Sizing

- **Low/Medium effort**: Batch freely — multiple items per work loop cycle
- **Large effort**: Small bites only — break into sub-tasks, verify each before moving to the next. Never batch large items together
- **If unsure**: Treat it as large. Smaller bites are always safer than overcommitting

### Refactoring

- Refactor when the code tells you to — duplication, unclear boundaries, performance bottlenecks
- Never refactor speculatively. Wait for the third instance before extracting an abstraction
- Refactoring is part of the work loop, not a separate phase
- Every refactor must pass the same cleanliness + benchmark gates as new code

## Key Principles

- **Cross-check against `rust-old/`.** The port's correctness bar is "matches what Rust did",
  within the tolerance class the port plan (§5) assigns. Diverge only with an ADR in `docs/adr/`.
- **hisab-first** (ADR 0007). Use Cyrius hisab for everything it provides. Where it lacks
  something, file the gap upstream with the numbers that show it (port plan §8) instead of
  writing a private copy.
- **Correctness over cleverness.** If the Cyrius behavior diverges silently from Rust, the bugs win.
- Test after every change, not after the feature is "done". ONE change at a time.
- **Never skip benchmarks.** Numbers don't lie. Speed is measured, not gated: a slow row is a
  finding to file, not a reason to change the port.
- **Deterministic.** Never iterate a `hashmap` (its seed is random per process); use sorted
  arrays keyed by handle or pair words. Every sort key is total, or the sort is stable.
- **Flat heap per step.** Never `alloc_reset` (it wipes the consumer's heap). World storage is
  world-owned; per-step scratch lives in a step arena (port plan §7.5).
- **Enums.** Public enums have explicit, append-only numeric values that are never
  renumbered (the Cyrius form of `#[non_exhaustive]`). Property functions `match` with an `_ =>` arm.
- **`#must_use` on all pure functions**, plus `#pure` where it applies; `#inline` on hot paths.
- **No aborts in library code.** `src/` never uses `+?`, `-?` or `*?` (they exit the
  process), `assert_fatal`, `panic` or `result_unwrap`. Errors return a value-form `Result`
  with an `IM_ERR_*` code.
- **f64 hygiene.**
  - Bind every call result to a `: f64` local before using it in an operator.
  - Never put hex in a `const`: it is an integer.
  - Never write `(0 - x)` on an f64: use `-x`.
  - No exponent notation in literals: the lexer has none. Generated files write decimals positionally.
  - Use the canonical `ganita_f64_*` names, not the deprecated short aliases.
  - For cross-target determinism use the `_polyfill` transcendentals (port plan §7.1).
- **Constants are generated, never typed by hand.** `scripts/oracle/` writes them from the
  oracle. Regenerate them; never edit them.
- **Every public global carries the `IM_` prefix** (constants, enum members, tables); private
  helpers use `_im_`; public structs are `Im<Name>`. A bundle shares one namespace with its consumers.
- **Function names follow the Rust API**, except where ADR 0008 changes them.
- **tracing on all operations** → sakshi spans, behind a level.
- Build with `cyrius build`, not raw `cat file | cc5` — the manifest auto-resolves deps.
- Source files only need project includes — stdlib auto-resolves from `cyrius.cyml`.
- `var buf[N]` = N **bytes**, not N entries.

## DO NOT

- **Do not commit or push** — the user handles all git operations (commit, push, tag)
- **NEVER use `gh` CLI** — use `curl` to the GitHub API only
- **Do not modify `rust-old/`** — it is the parity oracle. Tools that drive it (golden and table
  generators) live in `scripts/oracle/` and depend on it by path
- Do not modify `lib/` files (vendored stdlib / dep symlinks)
- Do not hardcode toolchain versions in CI YAML — `cyrius = "X.Y.Z"` in `cyrius.cyml` is the source of truth
- Do not widen a parity tolerance class to make a row pass. Class-C bounds are set by the
  rule in `docs/development/port-parity.md` before a function's first comparison
- Do not add unnecessary dependencies
- Do not `panic` or abort in library code
- Do not break backward compatibility without a major version bump
- Do not skip benchmarks before claiming performance improvements
- Do not skip tests before claiming changes work

## Documentation Structure

```
Root files (required):
  README.md, CHANGELOG.md, CLAUDE.md, CONTRIBUTING.md, SECURITY.md, CODE_OF_CONDUCT.md, LICENSE

docs/:
  development/impetus-port-plan.md — the port plan of record
  development/state.md             — live state
  development/roadmap.md           — completed items, backlog, 2.0 breaking cleanup
  development/port-parity.md       — measured parity baselines (M0 step 8)
  benchmarks-rust-v-cyrius.md      — Rust vs Cyrius benchmark record
  adr/NNNN-title.md                — architecture decision records (template.md)
  architecture/                    — non-obvious constraints
  guides/                          — task-oriented how-tos
rust-old/docs/                     — the Rust 1.4.0 architecture overview and testing guide, kept for reference
```

ADR format: `# NNNN — Title`, then Status, Context, Decision, Consequences (see `docs/adr/template.md`).

## CHANGELOG Format

Follow [Keep a Changelog](https://keepachangelog.com/): `## [Unreleased]`, then per version
`### Added / Changed / Fixed / Removed / Security / Performance`.

- Every change that alters behavior gets a CHANGELOG entry
- Performance claims MUST include benchmark numbers (Rust vs Cyrius where relevant)
- Breaking changes get a **Breaking** section with a migration guide
- Group by module when multiple changes land in one release
- Link the ADR when a change was driven by an architectural decision
