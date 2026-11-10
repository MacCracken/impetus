# impetus Rust → Cyrius port plan

> **Plan of record** for porting impetus to Cyrius. Rust **1.4.0** is the final Rust feature
> release and the **reference implementation** (the oracle); the Cyrius port ships as **2.0.0**.
> The method is tanmatra's ([its ADR 0006](../../../tanmatra/docs/adr/0006-cyrius-port-with-rust-oracle.md),
> [its port plan](../../../tanmatra/docs/development/tanmatra-port-plan.md)), as adapted by kimiya
> ([its port plan](../../../kimiya/docs/development/kimiya-port-plan.md)). Where this plan says
> "as tanmatra" or "as kimiya", follow that document. §2 lists what is different here and why.
>
> ▶ **Start on Cyrius 6.7.18 with hisab 3.4.1**, after 1.4.0 is committed and tagged (M0 step 0).

| Thing | Value | Source |
|---|---|---|
| Rust source (oracle) | impetus **1.4.0** (tagged and released) plus the unreleased port-tooling change of 2026-11-09 (the hidden `oracle` feature, `2d` without hisab; `VERSION` stays 1.4.0, behaviour unchanged): 10,399 code lines in `src/` (no tests, no comments), 36 files, two exclusive backends (`2d` default, `3d`) | `Cargo.toml`, `VERSION` |
| Rust health, 2026-11-09 on rustc 1.99.1 | ✅ fmt, clippy `-D warnings` on 5 feature sets, rustdoc `-D warnings` on both backends, `cargo audit`, `cargo deny`. Tests: 298 unit (2D) / 274 (3D), 30 regression (2 `#[ignore]`d for the 2.0 snapshot format), 9 integration, 2 doc | local run |
| Correctness baseline | The 1.4 audit-and-repair sweep: every fix is pinned by a test in `tests/regressions.rs` | `CHANGELOG.md` 1.4.0 |
| Cyrius toolchain | **6.7.18** (installed) | `~/.cyrius/current` |
| hisab | **3.4.1**, Cyrius, tag `3.4.1`, bundle `dist/hisab.cyr` | `/home/macro/Repos/hisab` |
| ganita | **1.2.15**, bundled in the 6.7.18 stdlib | `~/.cyrius/versions/6.7.18/lib/ganita.cyr` |
| Oracle's math | Rust **std** → host **glibc 2.44** for `sin_cos` (15 calls), `sin`/`cos`, `atan2` (2), `hypot` (15), `exp` (4), `powf` (1). glam 0.29.3 (re-exported by Rust hisab 1.4.0) for `DVec3` / `DQuat` / `DMat3`, 3D only | `grep` of `src/`, `Cargo.lock` |
| Cyrius consumers | **None yet.** kiran and joshua have no local repo; aethersafha is Cyrius (0.18.2) but declares no impetus dependency; the bridge partners dravya, sharira and ushma have no local repo | `~/Repos` |
| Target | impetus **2.0.0**, Cyrius | this document |

## 0. Decisions already made

- **hisab-first** (2026-11-09). The port uses as much of Cyrius hisab as it can: vector,
  quaternion and matrix types, ray tests, GJK/EPA/MPR, BVH, island detection, inertia tensors.
  It does not keep private copies of glam or of impetus algorithms that hisab already provides.
  Where hisab lacks something, the port **files the gap upstream** (§8) rather than transcribing
  around it. The ports are how the ecosystem finds where hisab, ganita, the stdlib and the
  compiler need work.
- **Performance is measured, not gated.** Every Rust benchmark gets a Cyrius twin and both
  columns are recorded per milestone (`docs/benchmarks-rust-v-cyrius.md`, as tanmatra). The
  ratios are first numbers on an 8-month-old language with no speed work yet, and hisab's own
  history shows its speed has been flat since September (median 0 % over 80 benchmarks, 6.6.4 →
  6.7.18). A slow row is a finding to file, with its numbers, not a reason to change the port.
- **The oracle is frozen.** Rust 1.4.0 moves to `rust-old/` and is never edited. A generator
  crate in `scripts/oracle/` drives it by path (as tanmatra). A wrong oracle value is fixed in
  Rust as 1.4.x and regenerated.
- **No Rust 2.0** (as kimiya). The roadmap's "2.0 — Breaking Cleanup" items that are API shape
  changes land in the Cyrius API only (§3.3), accepted by ADR. New physics stays out of the port.

## 1. Porting models and references

- **tanmatra 2.0.1** (`/home/macro/Repos/tanmatra`), the template. It is the source for:
  - `scripts/oracle/` (generator crate, TSV format 1, `validate.py`, `probe_tables.py`);
  - `tests/parity.tcyr`, `tests/parity_core.cyr` and `tests/ulp.cyr`;
  - `docs/development/port-parity.md` and the port-parity rule for class-C bounds;
  - the CLAUDE.md translation of the Rust principles (its plan §2 step 2);
  - `scripts/rust_test_ledger.py` and `scripts/surface_parity.py`.
- **kimiya** (in progress), the model for:
  - a **hisab dependency** (`[deps.hisab]`, ADR 0003);
  - a **glibc oracle** (std, not `libm`; ADR 0005 for correctly rounded primitives);
  - **API divergences** accepted by ADR (its ADR 0002 and the oracle ↔ `Err` mapping).
- **hisab 3.4.1**, the dependency. Read `CLAUDE.md`, `docs/guides/migration-3.4.md` (the Result
  convention) and the doc comment of every function impetus calls. Several 3.x functions are
  stricter than glam (§7.2).
- **Language and toolchain** (`/home/macro/Repos/cyrius`): `docs/guides/cyrius-guide.md`,
  `docs/stdlib-reference.md`, `docs/development/ecosystem-migration.md` (6.7.x entries),
  `cyrius help manifest`, `BENCHMARKS.md`.

---

## 2. What is different from tanmatra and kimiya

tanmatra and kimiya are libraries of pure functions: values in, values out, one golden row per
call. impetus is a **stateful simulation**, so five things change.

1. **A step is not a function call.** A world step runs broadphase, narrowphase, warm start,
   iterations, position correction, integration and sleep over shared state. A 1-ulp difference in
   a stacked pile grows every frame, so long trajectories can't be compared bit for bit. Parity
   therefore has three layers (§5):
   - **function rows** for everything that is a function (springs, mass properties, contact
     generation, ray tests, broadphase pair sets, materials, config validation);
   - **single-step rows**: load the oracle's full state at frame k, step once, and compare the
     full state at k + 1;
   - **scenario reference tests**: physical outcomes such as resting height, bounce speed,
     swing, settling and sleep. `tests/regressions.rs` is already this, one test per 1.4 fix.
2. **hisab replaces impetus code.** hisab's results differ from glam's and from impetus's own
   algorithms, on purpose:
   - `hquat_rotate_vec3` rotates by q/|q|;
   - `m3_inverse_checked` decides singularity exactly;
   - hisab's GJK is a more robust algorithm than impetus's.

   Those paths are compared within a class-D tolerance and by the reference tests, never by
   bits. ADR 0007 accepts each divergence by name (§5).
3. **Memory per frame is a design problem.** The stdlib `alloc` is a bump allocator with no free.
   hisab allocates every vector, quaternion and matrix it returns. `alloc_reset` would wipe the
   consumer's heap as well as impetus's. A world that steps 60 times a second for hours must
   reach a flat heap after warm-up. This is M0's first probe (§4 step 9).
4. **Two backends behind one API.** Rust's `2d` and `3d` features are exclusive and give the
   same public names. In Cyrius:
   - `[lib]` (the 2D default) and `[lib.3d]` are bundle profiles with disjoint backend modules;
   - each defines the same `ImPhysicsWorld` API, and a consumer includes exactly one bundle;
   - `[features]` only selects optional dependencies and defines nothing, and `#define` is
     capped at 16 names, so profiles are the clean split;
   - tests include their backend's module list explicitly, as tanmatra's split parity harness
     does.
5. **Snapshots can't be carried.** bitcode has no Cyrius implementation, and the 1.4 snapshot is
   incomplete anyway (roadmap 2.0). The port designs the complete format from the start (M6). It
   has no oracle, so it is checked by round trip and by step-equivalence: a restored world steps
   to the same bits as the original.

---

## 3. Scope

### 3.1 In scope for 2.0.0

| Milestone | Rust module | Code lines | Cyrius module(s) | hisab use |
|---|---|---|---|---|
| M1 | `error.rs` | 19 | `src/error.cyr` | — |
| M1 | `config.rs` | 124 | `src/config.cyr` | — |
| M1 | `material.rs` | 108 | `src/material.cyr` | — |
| M1 | `units.rs` | 106 | `src/units.cyr` | — |
| M1 | `force.rs`, `body.rs`, `collider.rs`, `joint.rs`, `event.rs`, `query.rs` | 259 | `src/types.cyr` (descriptors, handles, events) | — |
| M1 | `arena.rs` | 198 | `src/arena.cyr` | — |
| M1 | `contact_tracker.rs` | 107 | `src/contact_tracker.cyr` | — |
| M1 | `spring.rs` | 268 | `src/spring.cyr` | — |
| M2 | `backend_2d/types.rs` | 550 | `src/b2_types.cyr` | `HVec2` at the API edge |
| M2 | `backend_2d/narrowphase.rs` | 830 | `src/b2_narrowphase.cyr` | `convex_hull_2d` (hull input) |
| M2 | `backend_2d/raycast.rs` | 448 | `src/b2_raycast.cyr` | none: queries stay off hisab with the step until S1/H9 (ADR 0007 amendment); impetus-owned on crmath's `sin_cos` and `hypot` |
| M2 | `aabb_tree.rs`, `spatial_hash.rs` | 786 | `src/aabb_tree.cyr`, `src/spatial_hash.cyr` | none: hisab is off the step path (ADR 0007 amendment); impetus-owned, as the Rust is |
| M3 | `backend_2d/solver.rs`, `joints.rs`, `state.rs` | 2,172 | `src/b2_solver.cyr`, `src/b2_joints.cyr`, `src/b2_state.cyr` | `detect_islands` |
| M3 | `world.rs` (2D paths, particles) | 1,078 | `src/world.cyr`, `src/particle.cyr` | — |
| M4 | glam 0.29.3 `DVec3`, `DQuat`, `DMat3` (the operations impetus uses) | ~300 | `src/math3.cyr` | none until S1/H9: impetus-owned transcription of glam ([ADR 0012](../adr/0012-impetus-owned-3d-math-stopgap.md)) |
| M4 | `backend_3d/types.rs` | 446 | `src/b3_types.cyr` | none until S1/H9 (ADR 0012); Rust's own inertia formulas |
| M4 | `backend_3d/narrowphase.rs` | 1,109 | `src/b3_narrowphase.cyr` | none until S1/H9: impetus's own GJK/EPA, ported (ADR 0012) |
| M4 | `backend_3d/raycast.rs` | 200 | `src/b3_raycast.cyr` | none: queries stay off hisab until S1/H9, as in 2D (ADR 0012) |
| M5 | `backend_3d/solver.rs`, `joints.rs`, `state.rs` | 1,473 | `src/b3_solver.cyr`, `src/b3_joints.cyr`, `src/b3_state.cyr` | `math3` until S1/H9 (ADR 0012); `detect_islands` off the step path only |
| M6 | `serialize.rs` and `world.rs` snapshot code | 48 + ~150 | `src/snapshot.cyr` (shared pieces), `src/b2_snapshot.cyr` / `src/b3_snapshot.cyr` (the profile's `ImWorldSnapshot`), `src/world_snapshot.cyr` (`snapshot`, `restore`); [ADR 0013](../adr/0013-complete-snapshots.md) | — |

`particle.rs` (153 lines) goes with `world.rs` in M3. The surface parity check (§4 gates) covers
every `pub fn` and every `pub` method on the re-exported types.

### 3.2 Deferred or not carried

| Rust surface | Decision | Why |
|---|---|---|
| `bridge.rs` (17 fns, 27 tests): dravya, sharira, ushma | **deferred** until a Cyrius consumer exists | the same rule as tanmatra's bridges. Each is a thin primitive-value wrapper, so it ports later as a small parity-tested file |
| `SolverKind::Xpbd` (2D-only, experimental) | **not carried**. `config_validate` returns `Err(IM_ERR_UNSUPPORTED)` | it has known gaps (roadmap: XPBD parity) and no consumer. Demand-gated, as a Cyrius 2.x item |
| serde derives on every public type | **ported as serde-compatible JSON** ([ADR 0010](../adr/0010-serde-compatible-json.md)) | the same documents serde_json writes and reads, by codecs on bayan's value tree (`src/json.cyr`) and a shortest-digit float printer (`src/fmtf.cyr`); M1b covers the M1a types, each later milestone its own |
| `Send + Sync` assertions, `#[inline]` | **not carried** | Rust-only mechanics |
| Heightfield and TriMesh colliders | **carried as in Rust**: accepted, generating no contacts | parity; contacts for them are a roadmap item |

### 3.3 API changes in the Cyrius version (accepted divergences, [ADR 0008](../adr/0008-port-api-divergences.md))

These come from the roadmap's "2.0 — Breaking Cleanup". Each row says how the harness still
compares it, so no function leaves parity entirely.

| # | Rust 1.4.0 | Cyrius 2.0.0 | Golden comparison |
|---|---|---|---|
| 1 | `add_collider` / `add_joint` store an orphan for a missing body | return `Result`: `Err(IM_ERR_BODY_NOT_FOUND)` | oracle orphan ↔ `Err`; the rest class A on handles |
| 2 | deprecated `restore` beside `try_restore` | one fallible `restore` | the snapshot is Cyrius-only (M6) |
| 3 | incomplete bitcode snapshot | complete, versioned format | round trip and step-equivalence (§2 item 5) |
| 4 | 3D `BodyState` keeps only rotation about Z | full quaternion and angular-velocity vector | the oracle hook (§4 step 0) exposes the full state |
| 5 | 3D Revolute / Prismatic / motors accepted, with no effect | `add_joint` returns `Err(IM_ERR_UNSUPPORTED)` in the 3D profile | oracle accept-and-ignore ↔ `Err` |
| 6 | `SolverKind::Xpbd` | refused by `config_validate` (§3.2) | oracle Xpbd rows ↔ `Err` |
| 7 | results bit-identical only on the same platform | **bit-identical across targets** for impetus-owned arithmetic (§7.1) | measured on x86_64 ELF, x86_64 PE and aarch64, as tanmatra |

Not in 2.0, because they are new physics with no oracle: 3D islands and island sleep, consistent
3D joint frames, XPBD parity, 3D revolute/prismatic/motors. They start after 2.0, with reference
tests only.

---

## 4. M0 — Initialize with `cyrius port`

0. ✅ **Freeze a green oracle (Rust, before the move). Done 2026-11-09.** 1.4.0 was already
   tagged and released, so the two oracle changes went in afterwards as **unreleased port
   tooling**: `VERSION` stays 1.4.0, no tag, no release, and simulation behaviour is unchanged.
   - **`oracle` feature** (non-default, `#[doc(hidden)]`, `impetus::oracle`). It is read-only;
     `tests/oracle.rs` proves that dumping every frame leaves a 60-step run bit-identical. It
     provides:
     - `PhysicsWorld::oracle_dump() → WorldDump`:
       - config, particles, emitters, force fields and the last step's events;
       - per backend, the arena slot generations and free lists (they predict the next handle);
       - every body field, including the full 3D quaternion, angular-velocity vector and the
         three inertia tensors;
       - colliders, and joints with their reference angle/rotation;
       - body → collider attachment order;
       - in 2D, the manifolds with their warm-start impulses;
       - the contact tracker.
     - `contacts_2d` / `shapes_touch_2d` (2D) and `contact_3d` (3D): the narrowphase for one
       shape pair.
     - `aabb_tree_pairs_{2d,3d}`, `spatial_hash_pairs_{2d,3d}` and `spatial_hash_auto_cell_size`:
       broadphase candidate pairs. Both structures report a superset of the overlapping pairs:
       the tree from its fattened leaves, the hash from shared cells.

     Mass properties come through the dump (body mass and inertia after `add_collider`); ray
     tests through the public `raycast`. Neither backend keeps joint accumulators or the island
     union-find between steps, and 3D keeps no manifolds, so there is nothing more to dump.
   - **`2d = []`.** The 2D backend never used hisab; the default build no longer compiles hisab
     and glam.
   - Gates on the change: fmt; clippy `-D warnings` on six feature sets (including `2d,oracle`
     and `3d,oracle`); rustdoc `-D warnings`; audit; deny. Tests: 2D 298 unit + 7 oracle + 28
     regression; 3D 274 + 7 + 14. CI and the Makefile run the 2D build with `oracle`.
   - **Maintainer:** commit it. Nothing in `src/` changes after this point. A wrong oracle
     value is fixed as 1.4.x and the corpus regenerated.
   - glibc is **2.44**. The oracle's transcendental bits depend on it, and every TSV header
     records it.
1. ✅ **Scaffold. Done 2026-11-09.** `cyrius port --dry-run .`, then `cyrius port .`.
   - Expect these to move into `rust-old/`: `Cargo.toml`, `Cargo.lock`, `src/`, `tests/`,
     `benches/`, `fuzz/`, `Makefile`, `deny.toml`, `codecov.yml`, `target/`.
   - Untrack `fuzz/target/` first. 208 build files under it are committed despite `.gitignore`.
2. ✅ **Repair what the scaffold overwrites** (done 2026-11-09). The scaffold reports appending `/rust-old/` to `.gitignore` but appends `/rust-old/target/`, so `rust-old/` stays tracked; `rust-old/README.md` is restored for the crate's `include_str!` doctest. The steps were (as tanmatra's plan §2 step 2, filed as
   `cyrius/docs/development/issues/2026-11-08-cyrius-port-clobbers-project-docs-and-omits-bench.md`):
   - **CLAUDE.md**: merge, keeping the identity, process and DO NOT list. Translate the principles:
     - `#[non_exhaustive]` → append-only enum values;
     - `#[must_use]` → `#must_use` / `#pure`;
     - no `unwrap`/`panic!` → no `+?`, `-?`, `*?`, `assert_fatal`, `result_unwrap` or `panic` in
       `src/`;
     - "Vec arena over HashMap" and "BTreeMap for determinism" → "sorted arrays and typed stride
       arenas; never `hashmap`" (its seed is random per process, §7.3);
     - "tracing on all operations" → sakshi spans, behind a level.
   - **Roadmap** restored from git; **`state.md`** rewritten for 1.4.0 → 2.0.0-dev.
   - **Workflows**: keep the scaffold's Cyrius CI and add `test --bounds`, `deps --locked`,
     `audit`, plus a **3D profile job**. Move the Rust workflows to
     `rust-old/.github/workflows/`. Append `/rust-old/target/` and `/build/` to `.gitignore`.
   - **Move Rust-only material** to `rust-old/`: `scripts/bench-history.sh`,
     `scripts/version-bump.sh`, `bench-history.csv`, `docs/guides/testing.md`,
     `docs/architecture/overview.md`.
   - **ADRs**: renumber `docs/adr/001`–`006` to the scaffold's four-digit form and add:
     - [0007](../adr/0007-cyrius-port-hisab-first.md), the port (drafted as 007 before the renumbering);
     - 0008, the §3.3 API divergences.
   - Check that `cyrius.cyml` reads `version = "${file:VERSION}"` and that `bench` is in
     `[test] stdlib`. Document the scaffold's stub functions so `cyrius audit` passes.
3. ✅ **`cyrius.cyml`. Done 2026-11-09.** No `[lib]` until M1. sakshi comes through hisab's own dependency: listing it in `[deps] stdlib` as well makes `cyrius deps` refuse one copy. Confirm the keys with `cyrius help manifest`.
   ```toml
   [package]
   name = "impetus"
   version = "${file:VERSION}"
   license = "GPL-3.0-only"
   language = "cyrius"
   cyrius = "6.7.18"

   [deps]
   stdlib = ["syscalls", "string", "alloc", "str", "fmt", "io", "vec", "math", "ganita",
             "tagged", "result", "freelist", "fnptr"]   # sakshi arrives via hisab

   [deps.hisab]
   git = "https://github.com/MacCracken/hisab.git"
   tag = "3.4.1"
   modules = ["dist/hisab.cyr"]

   [test]
   stdlib = ["assert", "bench"]

   [lib]                 # 2D, the default, as Rust's default feature
   modules = ["src/error.cyr", ...]

   [lib.3d]
   modules = ["src/error.cyr", ...]
   ```
   - **Shared order:** error → config → material → units → types → arena → contact_tracker →
     spring → broadphase. The world, JSON and the snapshot come last (M6: `snapshot.cyr`, the profile's snapshot
     file, then `world_snapshot.cyr`, each after JSON).
   - **2D:** b2_types → b2_narrowphase → b2_raycast → b2_joints → b2_solver → b2_state →
     particle → world.
   - **3D:** b3_types → … → world.
   - `world.cyr` is shared. Where it calls a backend, it calls the same `_im_backend_*` names,
     which each profile defines.
   - hisab's own leaves (`dist/hisab.deps`: math, alloc, vec, ganita, result, str, fmt) are
     listed in `[deps] stdlib`.
4. ✅ **Toolchain and math check. Done 2026-11-09.** Probed on 6.7.18: `f64_min` / `f64_max` return the second operand when the first is NaN and NaN when the second is (Rust ignores a NaN either way); `f64_sin` / `f64_cos` are the polyfills on x86_64 (same bits); `f64_floor` / `f64_ceil` / `f64_sqrt` / `ganita_f64_hypot` / `_f64_exp_polyfill` / `_f64_atan_polyfill` present; NaN is `x != x` and finiteness `x - x == 0`. Confirm the counterpart of every Rust f64 method impetus calls
   (§7.1). The stdlib lacks public `is_nan`, `is_finite`, `copysign`, `sin_cos`, `EPSILON`,
   `INFINITY` and `NAN`, and Rust-semantics `min` / `max`. Write local `_im_*` helpers for them
   (as kimiya's `mathx.cyr`) and file the gaps (§8).
5. ✅ **Golden-value generator. Done 2026-11-09** (`scripts/oracle/`, from kimiya's crate: `constants` 3 rows, `math` 18,919 rows, 2.2 MB; both builds `--check` clean; `validate.py` recomputes the 7,197 class-A/B math rows in Python and checks the constants against the Rust literals, and a tampered row fails it). hisab 1.4.0 links glam **0.29.3**, which the header records. Copy tanmatra's crate and adapt it.
   - **Two oracle builds.** The backends are exclusive, so the generator is one crate run
     twice: `--features 2d,oracle` and `--features 3d,oracle`. The 3D build writes
     `tests/golden/b3_*.tsv`.
   - TSV format 1, f64 as bit patterns, deterministic, with a `--check` mode, as tanmatra.
   - **glibc in the header**, as kimiya; `--check` refuses a corpus from another glibc.
   - **Step format** (new): each row is a scenario name and a frame index. Its inputs are the
     full state dump at frame k plus the frame's applied forces; its outputs are the dump at
     k + 1 and the frame's events. State dumps flatten in a fixed order (bodies by handle, then
     manifolds by pair key, then joints).
6. ✅ **Table and constant generator. Done 2026-11-09** for the public constants: `src/constants.cyr` (`IM_MAX_SUB_STEPS`, `IM_MAX_ITERATIONS`; `IM_MAX_SNAPSHOT_INDEX` waits for M6), with a proof that the list is every `pub const` in `rust-old/src`. The private constants are extracted and proven at their milestones: `src/constants_private.cyr` (`t_private.rs`) since M2b, for `aabb_tree`, `spatial_hash` and `backend_2d/types`. impetus has few tables: the material presets, config
   defaults and limits (`MAX_SUB_STEPS`, `MAX_ITERATIONS`, `MAX_SNAPSHOT_INDEX`), and solver
   constants (`RESTITUTION_VELOCITY_THRESHOLD`, `MAX_CONDITION_NUMBER`, `JOINT_BAUMGARTE`). They
   are extracted and proven as tanmatra does for private constants.
7. ✅ **Parity harness. Done 2026-11-09:** `tests/parity_2d.tcyr` (skips `b3_*`), `tests/parity_core.cyr` (kimiya's, with an `_par_error_name` hook), `tests/ulp.cyr`. First run on x86_64 ELF: 3,154 ok, 0 failed, 11,722 measured, 4,046 pending (M1's Rust-semantics helpers). Against glibc: sin, cos, atan2, exp and powf ≤ 1 ulp, hypot ≤ 2; one outlier, ganita's `pow(-1, DBL_MAX)` (as tanmatra ADR 0007 / kimiya ADR 0004; impetus's bases are ≥ 10⁻⁴). Copy `tests/parity.tcyr`, `tests/parity_core.cyr` and `tests/ulp.cyr`, and
   split them per profile: `tests/parity_2d.tcyr` and `tests/parity_3d.tcyr`. Add:
   - a **state loader** that builds a world from a dump row;
   - a **step comparator** with per-field classes;
   - a **divergence table** naming every ADR 0007 path, reported by name, as kimiya's ADR 0002
     mapping.
8. ✅ **Baseline measurements. Done 2026-11-09** ([`port-parity.md`](port-parity.md) §2): the corpus on x86_64 ELF, x86_64 PE and aarch64, with a per-function digest of the port's output bits. sin, cos, hypot, pow and every class-A/B row give **identical bits on all three targets**. The native `f64_exp` and ganita's atan2 (on the native `f64_atan`) differ on aarch64, so the port uses `_f64_exp_polyfill` and ganita's atan2 on `_f64_atan_polyfill`, identical everywhere. K = 2 ulp for the primitives (max observed 1; hypot 2 on 3 rows), now gated. [ADR 0009](../adr/0009-accept-ganita-pow-minus-one-divergence.md) accepts `pow(-1, DBL_MAX)`. Result on every target: **14,875 ok, 0 failed, 0 measured, 4,046 pending, 1 accepted**. Baseline measurements → `docs/development/port-parity.md`. The math sweep runs every
   Cyrius counterpart against glibc on x86_64 ELF, x86_64 PE (wine) and aarch64 (qemu). Set the
   class-C bounds by tanmatra's rule (K_F = 2·S·calls + 2; above 64, make the primitive exact
   or write an ADR).
9. ✅ **Probes. Done 2026-11-09** ([`port-parity.md`](port-parity.md) §4, programs in `scripts/probes/`):
   (1) every hisab call that returns a value leaks from the global bump heap (24 B per vector op, 105,920 B per
   `gjk_epa_3d`), so hisab stays off the step path until S1/H9 ([ADR 0007 amendment](../adr/0007-cyrius-port-hisab-first.md));
   freelist and a per-step arena are flat. (2) Hot loops load components into `f64` locals (56 ns per contact kernel;
   struct values 154 ns, a compiler finding). (3) Typed stride arrays work; `--bounds` can't check them. (4) Struct
   values, nested fields and overloads work and are bit-exact. (5) Both solvers' shock-propagation sorts are stable
   on a non-total key: the port sorts on (level, input index). (6) `[lib]` is never prepended to tests: per-profile
   `tests/lib_2d.cyr` / `tests/lib_3d.cyr`. The probes, each answered with a small program and numbers before M1 writes any code:
   1. **Memory model.** Can world-lifetime storage (freelist or a world-owned arena) and a
      per-step scratch arena coexist with hisab's allocations? Can hisab be pointed at the
      scratch arena (`alloc_via`), or must it keep using the global bump heap? Gate: a soak
      test of 10⁵ steps of a 100-body pile reaches a flat heap high-water mark after warm-up.
   2. **Vector representation in hot loops.** Time a contact-solve inner loop four ways:
      - hisab `HVec3` handles;
      - hisab with in-place setters;
      - struct values (`struct ImV3 { x: f64; y: f64; z: f64; }` with `ImV3_add` operator
        overloads);
      - plain scalars.

      The numbers go into hisab request H1 (§8). Until H1 lands, use the fastest form inside
      the solver, narrowphase and integration loops, and hisab everywhere else.
   3. **Typed stride arrays.** An arena of `ImBody` structs through a typed `*ImBody` base, with
      `base + k` stepping by `sizeof`, under `--bounds`.
   4. **Struct values.** Re-probe at the pin (they changed heavily in 6.7.10 – 6.7.17):
      - returns of 16 bytes (vec2, two registers) and 24/32 bytes (vec3/quat, hidden pointer);
      - `Result` payloads (the open small-struct bug, so use a handle or out-pointer);
      - operator overloading on struct values.
   5. **Deterministic sort.** `vec_sort_by` is introsort, not stable. Every sort key impetus uses
      must be total (pair keys, handle indices), or a local stable merge sort must replace it.
      List each Rust sort site and its key.
   6. **Per-profile tests.** How `cyrius test` prepends `[lib]` modules. The 3D test files must
      compile against the 3D list only.

**Gate:** `cyrius build` and `cyrius bench` pass; `cyrius test` runs both harnesses (0 modules)
with no warnings; `cyrius lint` and `fmt --check` are clean. Committed: the generators,
`tests/ulp.cyr`, ADRs 0007 and 0008, the probe results in `port-parity.md` and the upstream issues
(§8).

---

## 5. Parity policy

tanmatra's classes apply (its plan §4), with two additions for the simulation:

| Class | Tolerance | impetus examples |
|---|---|---|
| **A — exact** | bit-identical / equal integers | handles and generations, config validation verdicts, material presets, combine rules, collision layers, broadphase pair **sets**, contact counts and feature ids, event sequences, constants |
| **B — arithmetic** | ≤ 2 ulp, expected 0 | spring step with its cached matrix (× + only), 2D contact generation (SAT, clipping), 2D mass properties, solver arithmetic on 2D paths, AABBs |
| **C — transcendental** | ulp ≤ K by the port-parity rule; **0** on the correctly rounded primitives, against the corrected oracle (ADR 0011) | spring transition (exp, sin, cos), 2D rotation (`sin_cos`), `atan2`, `hypot` sites, joint angles |
| **D — iterative / hisab-backed** | relative ≤ 1e-12, or a per-function bound set before its first comparison | every ADR 0007 path: 3D quaternion rotation, mat3 inverse, GJK/EPA normal and depth, BVH-derived pair order (the pair *set* stays class A), `detect_islands` membership (class A on sets) |
| **S — single step** | per field: the class of the path that produced it | one `step()` from an oracle state dump. Positions, velocities and impulses take the path's class; contact sets and events are class A, except a named near-degenerate contact flip |
| **R — reference** | physical tolerance stated in the test | the 30 regression scenarios, the Rust unit tests that assert physics (resting height, bounce speed, swing, settling, sleep), and energy and momentum invariants over long runs |

Rules:
- **Never widen a class to make a row pass** (as tanmatra). A failure is one of three things:
  - a port defect;
  - a Cyrius/hisab/ganita defect, which is filed upstream;
  - an oracle defect, which is fixed in Rust 1.4.x and regenerated.

  The only exception is a divergence accepted by an ADR, which the harness still reports by name.
- **Bit-level trajectories only where the scene is not chaotic**: free fall, a single pendulum, a
  spring, one bouncing ball. They are compared over the whole horizon. Piles, stacks and chains
  get class S rows at frames spread through the run, plus class R.
- **Branch points on both sides**:
  - the restitution threshold (1 m/s);
  - the block solver's condition number (1000);
  - the static/kinetic friction slip threshold;
  - sleep onset and wake-on-contact;
  - the speculative margin, and exactly touching shapes: a box placed at depth 0 registers a
    contact on the first step in 2D but not in 3D (seen in `tests/oracle.rs`);
  - the spatial-hash oversize cap (`MAX_CELLS_PER_ENTRY`);
  - the spring's damping regimes (under, critical, over).

---

## 6. Milestones

Every milestone ends with tanmatra's gates (its plan §3), run for **both profiles** from M4 on:
- `cyrius build`; `cyrius test` and `test --bounds` **with no warnings** (the f64 type diagnostics
  appear only there);
- `cyrius bench`, recorded against the Rust numbers;
- `vet`, `lint` and `fmt --check` on the milestone's files;
- `distlib --all`, then `distlib --check`, then `deps --locked`;
- **surface parity**: every in-scope Rust `pub fn` has a Cyrius counterpart or a written reason;
- the soak test (§4 step 9.1) once a world steps;
- CHANGELOG and `state.md`.

| Milestone | Modules | Key work | Golden grid (minimum) |
|---|---|---|---|
| **M1 Foundation** | error, config, material, units, types, arena, contact_tracker, spring | `ImpetusError` → `IM_ERR_*` append-only code enum, no detail string (as tanmatra). `WorldConfig::validate` with its limits. `CombineRule` priority. Descriptors as structs; `ColliderShape`, `JointType`, `ForceField`, `CollisionEvent` as a kind enum plus a typed payload struct (a boxed payload enum allocates, and `: stack` holds one word). Generational arena over a typed stride buffer with the lazy free-list rebuild. Sorted-array contact tracker. Exact spring with the cached transition | every config field at and past its limit; every combine-rule pair; arena insert/remove/reuse sequences (seeded); tracker sub-step sequences incl. start-and-stop within a frame; springs over under/critical/over-damped × dt ∈ {1e-6, 1/60, 1, 100} |
| **M2 2D collision** | b2_types, b2_narrowphase, b2_raycast, broadphase | Mass properties (polygon, capsule, box, circle; offset colliders). Narrowphase: reference/incident-face clipping, rounded shapes, capsule end contacts, either winding, normals A→B. Raycast: hisab `geo_ray_*` with z = 0 where probe 2 allows, impetus code for segments and polygons. Broadphase: the balanced AABB tree (or a per-frame `bvh_build`, if the probe wins), the spatial hash with an oversize list | every shape pair × both orders × seeded poses incl. touching, separated by the speculative margin, deep, rotated by 0/π/4/π/2; rays from every side and inside; pair sets vs brute force for 1,000 boxes (sorted row, random) |
| **M3 2D dynamics** | b2_solver, b2_joints, b2_state, particle, world | Sequential impulses: warm start 0.95, the block solver with the condition check, restitution from the pre-solve speed, split impulse, soft ERP. Velocity-level joints (`Row`, `PointBlock`, soft springs, limits, motors, breaking). Islands via `detect_islands` with contacts filtered to dynamic–dynamic (static ground doesn't merge), atomic sleep, wake on contact begin. Sub-steps with forces held for the frame. Particles: R3 spread, capped spawns, fields, drag, collisions. Events | the step corpus: box stack 10, pyramid, revolute chain 20, pinned box, bouncing balls (e = 0, 0.5, 1), friction ramp, sleeping stack plus support removal, sub-steps 1/4/8, an emitter scene; class S at frames {1, 2, 10, 60, 300}; class R for all 28 2D regression tests |
| **M4 3D collision** | b3_types, b3_narrowphase, b3_raycast | Per [ADR 0012](../adr/0012-impetus-owned-3d-math-stopgap.md): glam's `DVec3`/`DQuat`/`DMat3` transcribed by value (`src/math3.cyr`). World-space inverse inertia R·I⁻¹·Rᵀ. Gyroscopic step (implicit, energy-projected). Mass properties with the parallel-axis shift, on Rust's own formulas. impetus's own GJK/EPA for hull and general pairs, its scratch world-owned. 15-axis OBB SAT with face clipping, capsule and sphere closed forms. Raycast impetus-owned | every 3D shape pair × both orders × seeded poses; inertia for each shape, offset and rotated; rays incl. zero direction; classes A–C at 0 ulp against the corrected oracle (class D only after the switch to hisab) |
| **M5 3D dynamics** | b3_solver, b3_joints, b3_state, world (3D) | Two-tangent friction, restitution bias, Baumgarte /dt, Fixed joint as a 6×6 block (a hisab or ganita dense solve, if one fits), Distance, Rope, Spring, Wheel, Mouse joints; Revolute/Prismatic/motors refused (§3.3 #5). Per-body sleep, sleepers zeroed | the M3 corpus in 3D plus a spinning-top gyroscopic scene; class R for the 3D regression tests |
| **M6 Snapshots** | snapshot | A complete, versioned binary format written with `store64`: header and version, config, arena slots with generations and free-list order, bodies, colliders, joints with accumulators, manifolds with warm-start impulses, sleep state, force fields, particles and emitters. Validation as `validate_snapshot` (index caps, finite values) | round trip for every scenario at several frames; restore-then-step equals step bit for bit (the 2 `#[ignore]`d Rust tests become live Cyrius tests); corrupted and truncated inputs refused (port of `fuzz_serialize`) |
| **M7 Release 2.0.0** | all | Rust-test ledger (mela pattern): each of the 2D + 3D unit tests, 30 regression, 9 integration and 2 doc tests is ported, folded into golden rows, or marked n/a / deferred. Full surface-parity diff. `docs/benchmarks-rust-v-cyrius.md` with all 38 benchmarks in both columns. `cyrius distlib --all`. `cyrius audit`. VERSION 2.0.0 | the full corpus green for both profiles on x86_64 ELF, x86_64 PE and aarch64 |

Order: M0 → M1 → M2 → M3 → M4 → M5 → M6 → M7. The 2D backend is the default, uses no hisab
types in Rust, and gives a complete world first. 3D leans hardest on hisab, so it comes after the
memory and vector probes have settled and the first hisab requests have had time to land.

---

## 7. Rust → Cyrius mapping (impetus specifics)

### 7.1 Math functions

Every std f64 method in `src/` outside tests, 1.4.0:

| Rust (calls) | Cyrius | Parity |
|---|---|---|
| `sqrt` (41), `abs` (284), `floor` (2), `ceil` (2) | `f64_sqrt`, `f64_abs`, `f64_floor`, `f64_ceil` (builtins) | exact on every target |
| `sin_cos` (15), `sin` (1), `cos` (1) | `_im_sin_cos`, `_im_sin`, `_im_cos`: correctly rounded (`src/crmath.cyr`, [ADR 0011](../adr/0011-correctly-rounded-exp-sin-cos.md)) | identical everywhere; class C compared exactly against the corrected oracle |
| `atan2` (2) | an atan2 on `_f64_atan_polyfill`, as the language survey recommends | `ganita_f64_atan2` calls the native x87 `f64_atan` on x86_64, so its bits differ between targets |
| `exp` (4, spring) | `_im_exp`: correctly rounded (`src/crmath.cyr`, ADR 0011) | identical everywhere; class C compared exactly against the corrected oracle (`f64_exp` is x87 on x86_64) |
| `hypot` (15) | `ganita_f64_hypot` (pure Cyrius) | ≤ 2 ulp from libm (tanmatra measured); class C, or exact by the rule if a site is ill-conditioned |
| `powf` (1) | `ganita_f64_pow` | class C; mind ganita's `pow(-1, huge)` defect (tanmatra ADR 0007) |
| `powi` (19) | explicit products in LLVM's square-and-multiply order (as tanmatra) | class B |
| `min` (32), `max` (74), `clamp` (28) | local `_im_min` / `_im_max` / `_im_clamp` with Rust's NaN-ignoring semantics (`f64_min(NaN, 1)` is 1 but `f64_min(1, NaN)` is NaN) | exact |
| `is_nan` (14), `is_finite` (42) | `x != x`; `x - x == 0` (as kimiya's `_km_is_finite`) | exact |
| `to_radians` (5) | `x * (PI / 180.0)` in Rust's order | class B |

**Typing traps** (as tanmatra §5, all probed on 6.7.18):
- An operator is typed by its left operand. Bind every builtin and ganita result to a `: f64`
  local first; `(0 - x)` is an integer subtraction, so write `-x`.
- There is no exponent notation, so write `0.000000001`.
- A hex pattern in a `const` is an integer.
- `F64_*` stdlib constants are untyped `var`s; define impetus's own `const`s.
- `#derive(accessors)` getters return untyped bits.

### 7.2 glam and impetus algorithms → hisab

The step-path rows below are on hold until S1 or H9 lands: M4 and M5 run on impetus-owned glam formulas and
GJK/EPA instead ([ADR 0012](../adr/0012-impetus-owned-3d-math-stopgap.md)). The table stays the plan for the switch.

| Rust | hisab 3.4.1 | Plan |
|---|---|---|
| `DVec3` new/ZERO/splat/X/Y/Z, `+ − neg`, `* f64`, `dot`, `cross`, `length(_squared)`, `min`/`max`/`abs` | `hvec3_*` | use; hot loops per probe 2 until H1 |
| `/ f64`, `+= −= *=`, `==`, vector `is_finite`, `normalize_or`, `try_normalize`, `any_orthogonal_vector`, `any_orthonormal_pair`, `from_array`/`to_array` | none | H2; local helpers until then |
| `DQuat` IDENTITY, `from_xyzw`, `from_rotation_z`, `q * q`, `.inverse()`, `.normalize()` | `hquat_identity`, `hquat_new`, `hquat_from_rotation_z`, `hquat_mul`, `hquat_conjugate` (bit-exact with glam's inverse), `hquat_normalize_checked` | use |
| `q * v` (glam assumes a unit q) | `hquat_rotate_vec3` (divides by \|q\|² unless it is exactly 1) | use; class D, ADR 0007. A unit-assuming variant is H2 |
| `-q` | none (`hquat_new` on negated parts) | H2 |
| `DMat3` ZERO, `from_cols`, `from_quat`, `M·v`, `M·M`, `+`, `* f64`, transpose, determinant | `m3_new`, `m3_from_cols`, `m3_from_quat`, `m3_mul_vec3`, `m3_mul`, `m3_add`, `m3_scale`, `m3_transpose`, `m3_determinant` | use |
| `from_diagonal`, `M − M`, `−M` | none | H2 |
| `.inverse()` | `m3_inverse_checked` (exact singularity) | use; class D |
| impetus GJK / EPA (3D) | `gjk_intersect_3d`, `gjk_epa_3d` (normal and depth), `mpr_*` | use; contact points need H3 |
| impetus 15-axis OBB SAT + face clipping, capsule and sphere closed forms | none (hisab makes no manifolds) | impetus-owned |
| impetus 2D SAT + clipping | none (`convex_hull_2d` only) | impetus-owned |
| impetus AABB tree (balanced, incremental) | `bvh_build`, `bvh_query_aabb` (static; no pair query) | probe a per-frame rebuild; H7 for pair query and incremental updates |
| impetus spatial hash (AABB rasterisation, pairs, oversize list) | `spatial_hash_*` (points with radius queries) | impetus-owned unless H7 |
| impetus island union-find | `detect_islands` (sorted, deterministic; unions every contact given) | use, with contacts filtered to dynamic–dynamic |
| impetus box/sphere inertia | `inertia_box`, `inertia_sphere` | use; capsule, cylinder and polygon inertia are H5 |
| 3D ray vs ball / box / capsule | `geo_ray_sphere`, `geo_ray_obb_face`, `geo_ray_capsule_branch` | use |
| sequential impulses, joints, sleep | `sequential_impulse` is a reduced model; hisab's D039 assigns the full solver to impetus | impetus-owned |

### 7.3 Types and idioms

| Rust | Cyrius |
|---|---|
| `Arena<T>` (generational) | a typed stride buffer, `*ImBody` + k stepping by `sizeof` (probe 3), with a generation column and the free list. Its storage comes from freelist or the world arena, never `vec` (8-byte words only, and growth never frees) |
| `BTreeMap` (11) / `BTreeSet` (2) | sorted arrays keyed by `u64` pair or handle keys, with binary search. Manifolds are rebuilt every frame (ADR 0006), so a sorted array rebuilt per frame fits. **Never `hashmap`**: its seed is random per process, so iteration order changes between runs |
| `HashMap` (1) | the same sorted array |
| `AabbTree<K>`, `SpatialHashGrid<K>` | keys are already handle words (`i64`); no generics needed |
| `ColliderShape`, `JointType`, `ForceField`, `CollisionEvent` (payload enums) | a kind enum (`IM_SHAPE_BALL`, …, append-only) plus one payload struct with the union of fields. Hull points sit behind a pointer and a count: struct fields can't be fixed arrays |
| `Vec<ContactPoint>` in `ContactData` | caller buffer plus count, or a frame-scratch array the world owns |
| `[f64; 2]` / `[f64; 3]` public arguments | struct values `ImVec2` / `ImVec3` at the API (two registers for 16 bytes); `HVec2` / `HVec3` on the hisab side |
| `Result<T, ImpetusError>` | value-form `Result` + `IM_ERR_*` code, with `?` (as tanmatra). No `Result` through `callptr` |
| `Option<BodyState>` and similar | out-pointer + 1/0, or the value-form `Option` for one word |
| closures in `sort_by` / iterator chains | loops; a comparator through `fncall2` with `f64_lt` for f64 keys |
| stable `sort_by_key` on a non-total key (the solvers' shock-propagation order) | sort on (key, index in the input order): total, and the stable sort's exact order (`vec_sort_by` is introsort) |
| `#[non_exhaustive]` / `#[must_use]` | append-only enum values / `#must_use` |

### 7.4 Names

- **Public structs** `Im<Name>` with `impl` blocks, so methods keep their Rust names:
  `w.add_body(desc)`, `w.step()`, `s.step(dt)`. The constructors are `ImPhysicsWorld_new`,
  `ImSpring_critically_damped` and so on.
- **Globals and enum members** carry `IM_`; private helpers carry `_im_`; the backend seam
  that `world.cyr` calls is `_im_backend_*`.
- Scan for collisions with the 6.7.18 stdlib and hisab before M1 (`vec_*`, `map_*`,
  `Ok`/`Err`, `f64_min`/`max`/`clamp`, ganita `mat_*`, hisab's `hvec*`, `geo_*`, `gjk_*`,
  `contact_*`, `detect_islands`). "Last definition wins" across files is only a warning.

### 7.5 Memory

Settled by probe 1 (`port-parity.md` §4.1; freelist and the step arena measured flat):
- **World lifetime**: arenas, broadphase nodes, manifolds and particles live in storage the world
  owns (freelist or a growable world arena), freed by `ImPhysicsWorld_free`.
- **Step lifetime**: pair lists, contact scratch, island lists and joint prep live in a step
  arena reset at the start of every step.
- **hisab temporaries**: in the step arena if hisab can be pointed there (H9), otherwise kept
  out of the per-step loops.
- **Never `alloc_reset`.** It would wipe the consumer's heap.

---

## 8. Gaps to file upstream (M0, then as found)

Each one gets an issue in the owning repo with the probe or benchmark that shows it, as
tanmatra filed its findings.

**Cyrius stdlib (blocking)**
- **S1. A scoped allocator for `alloc`** — **filed 2026-11-09**:
  `cyrius/docs/development/issues/2026-11-09-no-scoped-allocator-for-alloc.md`, with a self-proving repro. `alloc` is a bump allocator with no free and no hook, so a
  library can't contain what the libraries it calls allocate. An allocation scope (`alloc_push(arena)` /
  `alloc_pop()`, or an allocator the global `alloc` dispatches through) lets impetus run each step's hisab
  calls in its step arena and reset it, with no change to hisab's 574 allocation sites. Measured cost of
  its absence: `port-parity.md` §4.1. **It blocks hisab on the step path** (ADR 0007 amendment), and it
  helps every library built on hisab or the stdlib containers.

**hisab**
- **H9 (blocking unless S1 lands). Allocator-scoped calls** — **filed 2026-11-09**:
  `hisab/docs/development/issues/2026-11-09-allocator-scoped-calls.md`, with a self-proving repro (exit 1 while
  open). hisab's alternative to S1: allocator-taking variants of the functions a step calls (vector, quaternion and
  matrix ops; `gjk_epa_3d`; `bvh_build`; `detect_islands`). Measured per-call leaks are in `port-parity.md` §4.1.
  Until it or S1 lands, M4 runs on impetus-owned 3D math and GJK/EPA ([ADR 0012](../adr/0012-impetus-owned-3d-math-stopgap.md)).
- **H1. Allocation-free vector math** in hot loops: in-place `_into(out, a, b)` variants, or
  value-type vectors. Today every returned vector, quaternion or matrix allocates. hisab's
  benchmark doc lists stack structs and arenas as "not planned unless a consumer asks"; this is
  the consumer asking, with probe 2's numbers.
- **H2. glam conveniences:**
  - vectors: divide by a scalar, equality, a public vector `is_finite`, `normalize_or`,
    `any_orthogonal`, `any_orthonormal_pair`;
  - matrices: `m3_from_diagonal`, `m3_sub`, `m3_neg`;
  - quaternions: `hquat_neg`, a unit-assuming `hquat_rotate_vec3_unit`;
  - 2D: `hvec2_cross` (the perp-dot).
- **H3. Contact points from GJK/EPA** (witness points), and a GJK distance query for
  separated pairs (speculative contacts).
- **H4. Segment–segment closest points** (capsule pairs).
- **H5. Inertia** for capsule, cylinder and convex polygon/hull, with the parallel-axis shift.
- **H6. 3D convex hull** (roadmap: ConvexHull raycast needs hull faces).
- **H7. Broadphase support**: a BVH self-pair query, incremental insert/remove/refit, and an
  AABB-cell spatial hash with pair enumeration.
- **H8. 2D ray tests**, or document that z = 0 embedding is supported.
- (H9 is listed first, above.)

**Cyrius stdlib / ganita / compiler**
- **S2. Shortest round-trip f64 printing** (Rust's `Display`, serde_json's zmij): **solved locally in
  M1b** (`src/fmtf.cyr`, ADR 0010), with Burger & Dybvig's free-format digits on a 40-limb bignum,
  pinned by 37,576 golden rows. The stdlib still lacks it; `fmtf.cyr` can seed an upstream printer
  (not filed yet).
- **S3. bayan: JSON integers above `i64::MAX`** — **filed 2026-11-09**:
  `bayan/docs/development/issues/2026-11-09-json-integer-above-i64-max-refuses-the-document.md`, with a
  self-proving repro and serde_json's output beside bayan's. The tree parser refuses any integer that
  doesn't fit an i64 ("integer is out of range"). serde_json reads u64 up to 2^64 − 1, and an integer
  beyond u64 as an f64. impetus's handles are u64: one at generation ≥ 2^31 is written by serde_json
  and can't be read back by the port. 3 golden rows pending (ADR 0010 findings). Ask: keep an
  out-of-i64 integer as a u64 (a flag or tag) or as its f64, as serde_json does, instead of failing.
- **S4. bayan: a duplicate-key policy** — **filed 2026-11-09** as a request (the strict default is deliberate and
  stays): `bayan/docs/development/issues/2026-11-09-json-duplicate-key-has-no-opt-in-for-ignored-members.md`, with a
  repro. Every bayan parser refuses a document with any
  duplicate key (1.11.1, deliberate). serde refuses only a duplicate *known* field and ignores repeats of
  unknown fields and anything inside an ignored value. 3 golden rows pending. Ask: an option for
  last-wins (or keep-all) objects, so a caller with its own field checks can match serde.
- **`str_builder` aborts on OOM** (not filed yet): the back-compat calls (`putc`, `add_cstr`,
  `add_int`) exit the process when a grow fails, so `src/fmtf.cyr` and `src/json.cyr`'s writers do too.
  The `_a` variants return -1; moving the writers onto them, with an error return, is impetus-side work.
- **Correctly rounded transcendentals in ganita** — **filed 2026-11-09** in ganita's queue (its source repo
  owns fixes, per cyrius's fold rule): `ganita/docs/development/issues/2026-11-09-correctly-rounded-exp-sin-cos-hypot.md`,
  with a self-proving repro (32 arguments, all wrong today, 0 through impetus's crmath). Measured against
  MPFR on 100,000 seeded arguments each: `ganita_f64_hypot` misrounds 33.65 % (max 2 ulp), the stdlib's exp,
  sin and cos polyfills 9.73 %, 3.13 % and 2.96 % (1 ulp); glibc 0.64 %, 0.067 %, 0.14 % and 0.13 %. impetus now owns correctly rounded
  exp, sin, cos and hypot (`src/crmath.cyr`, ADR 0011 and its hypot amendment), as kimiya owns its acos,
  cos, ln and pow. They belong in ganita, once, with a phased (Ziv) design like impetus's. impetus's cost
  97 ns (exp), 187 ns (sin_cos) and 168 ns (hypot), about 2× the polyfills and 5× ganita's hypot
  (`benchmarks-rust-v-cyrius.md`). ganita's hypot is 1–2 ulp off the correctly rounded value on 24 % of
  the golden rows, glibc's on 0.31 % of seeded arguments.
- **Derive features for serde-shaped JSON** (not filed yet): variant renaming (`"Average"`, not the member
  name), `Option` fields as `null`, externally tagged payload enums (`{"Ball":{"radius":0.5}}`). With them,
  `src/json.cyr`'s hand-written codecs could move to `#derive`.
- **Stdlib math:**
  - public `f64_is_nan` / `f64_is_finite`, `f64_copysign`, `f64_sin_cos`;
  - `F64_EPSILON` / `F64_INFINITY` / `F64_NAN` as typed constants;
  - Rust-semantics `f64_min_num` / `f64_max_num`;
  - a deterministic atan2 (on the polyfill), and an exp polyfill exposed as the
    cross-target choice.
- **Collections:**
  - a stable sort (both solvers' shock-propagation sorts rely on stability, `port-parity.md` §4.3;
    the port works around it with a total key);
  - `vec_reserve` / `vec_free`;
  - a documented way to seed `hashmap` deterministically;
  - an ordered map, if more than impetus needs one.
- **S5. `memcpy` copies a byte at a time** — **filed 2026-11-09** as impetus's addition to ranga's open issue
  `cyrius/docs/development/issues/2026-11-02-memcpy-memset-byte-loops-dominate-frame-copies.md`
  (a second file would have duplicated it), with a repro of its own, `repros/2026-11-09-memcpy-small-struct-copies.bcyr`.
  `lib/string.cyr`'s `memcpy` is a `load8` / `store8` loop; the `rep movsb` it replaced broke aarch64's instruction
  alignment. ranga measured 8 MB frames (6.5×). impetus adds the struct-sized regime: 8-byte chunks with a byte
  tail, in plain Cyrius, are 4–7× faster (x86_64 ELF, two runs). 24 B goes from 42–44 to 11 ns, a 184-byte
  `ColliderShape` from 287–296 to 43–45 ns, a 224-byte rigid body from 341–361 to 51–54 ns, and 4 KiB from
  6.08–6.45 to 0.86–0.91 µs. The port calls `memcpy` for every arena insert and get, for the shape copy each
  collider takes (about 290 ns of the shape copy's 349, in a ball's 545 ns), for the spatial hash's 24-byte record
  moves, and in every JSON reader. The issue's proposed fix (8-byte chunks, landing with or after `memmove`)
  covers both regimes.
- **Compiler:**
  - f64 values live in integer registers and every op moves through xmm0/xmm1, with no xmm
    register allocation;
  - a struct-value vector kernel costs 2.5× the same kernel on `f64` locals (154 vs 56–63 ns,
    `port-parity.md` §4.2): the 24-byte return through the hidden pointer and the field copies;
  - `#inline` takes at most two parameters;
  - multi-return values are untyped words: an operator on one (`-x`, `x * y`) is integer arithmetic on
    the f64's bits, with no f64 diagnostic (M2c caught one through its golden rows). Bound to an `f64`
    local first, they are fine; four-value returns don't parse (`var a, b, c, d = f()`);
  - `memcpy` / `memset` are byte loops (open issue `2026-11-02`);
  - struct fields can't be fixed arrays;
  - `#define` is capped at 16 names;
  - small-struct `Result` payloads misread (open issue `2026-11-08`);
  - `f64v_*` writes past `dst` when n isn't a lane multiple (open issue `2026-11-06`);
  - no exponent literals.

---

## 9. After 2.0.0

1. **Bridges** (`src/bridge.cyr`), ported per consumer as dravya, sharira and ushma land in
   Cyrius. Parity comes from the Rust bridge tests in `rust-old/`.
2. **New physics from the roadmap**, now in Cyrius:
   - 3D islands and island sleep (on `detect_islands`);
   - 3D revolute, prismatic and motors;
   - consistent 3D joint frames;
   - XPBD, if a consumer asks;
   - CCD on hisab `time_of_impact` / `swept_aabb`;
   - compound shapes;
   - shape casting;
   - heightfield contacts.
3. **Speed work with the ecosystem.** Once kiran and soorat are in Cyrius, the benchmark
   columns and the §8 issues are the input to compiler, ganita and hisab tuning.
4. **Removing `rust-old/`** after the bridges are ported or dropped, the corpus and generators
   are archived, and the test ledger is closed (as tanmatra §6).

---

## 10. Tracking

| Milestone | Status | Golden rows | Reference tests | Benches | Notes |
|---|---|---|---|---|---|
| M0 Initialize | ✅ done 2026-11-09 | constants 3 + math 18,919: **14,875 ok, 0 failed, 0 measured, 4,046 pending (M1 helpers), 1 accepted (ADR 0009)**, identical bits on x86_64 ELF, x86_64 PE and aarch64 | — | 1 probe bench | oracle hooks; scaffold and repairs; `cyrius.cyml` with hisab 3.4.1; generator, `validate.py`, `src/constants.cyr`, parity harness; exp and atan2 on the polyfills (cross-target bits), K = 2; ADRs 0008, 0009 and the 0007 amendment (hisab off the step path until S1/H9); six probes ([`port-parity.md`](port-parity.md) §4). Next: M1 |
| M1a Foundation: math helpers, error, config, material, units, force | ✅ done 2026-11-09 | config 293 + material 2,046 + units 2,236 + force 1,308 + math (0 pending now): **24,431 ok, 0 failed, 281 pending (S2 280, M6 1), 93 accepted** on x86_64 ELF, x86_64 PE and aarch64; every row 0 ulp | 33 of 44 Rust tests ported (2 partial: the Display symbols), 11 serde n/a; 97 assertions | 6 (4.8–31× Rust; [benchmarks](../benchmarks-rust-v-cyrius.md)) | `src/error.cyr`, `mathx.cyr`, `vec.cyr`, `config.cyr`, `material.cyr`, `units.cyr`, `force.cyr`; `[lib]` + `dist/impetus.cyr`; `tests/lib_2d.cyr`; surface parity 100/100 (`scripts/surface_parity.py`); ADR 0008 #6 mapped |
| M1b Foundation: types, arena, contact tracker, spring | ✅ done 2026-11-09 | fmtf 37,576 + json 937 + spring 1,000 + types 1 (+ math 19,165 with the crmath threshold rows): **64,455 ok, 0 failed, 0 measured, 7 pending (S3 3, S4 3, M6 1), 103 accepted (ADR 11: 10)** on x86_64 ELF, x86_64 PE and aarch64; every spring step row 0 ulp | all 63 Rust tests of the M1b modules ported (body 6, collider 7, joint 12, event 6, query 3, arena 10, tracker 3, spring 16), plus Cyrius-only groups and `crmath.tcyr` (23) | 3 spring + 4 serde + Display + 2 crmath ([benchmarks](../benchmarks-rust-v-cyrius.md)) | part 1: ADR 0010, `src/fmtf.cyr`, `src/json.cyr` for the M1a types. Part 2: `src/types.cyr`, `arena.cyr`, `contact_tracker.cyr`, `spring.cyr`; codecs for every M1b type in both serde struct forms; S3, S4 found; surface parity 258/258. Part 3: [ADR 0011](../adr/0011-correctly-rounded-exp-sin-cos.md), correctly rounded exp, sin and cos (`src/crmath.cyr`, generated `crmath_data.cyr`), the corrected oracle (`*.cr.tsv`, 10 rows), class C exact on them |
| M2 2D collision | ✅ done 2026-11-09 | broadphase 76 + b2_mass 75 + b2_narrow 17,854 (contacts class C at 0 ulp against the corrected oracle, 3 corrected; touch class A) + b2_ray 14,754 (rays class C at 0 ulp, 39 corrected; overlaps class A, which also pin `world_aabb`) + `f64::hypot` gated at 0 (12 corrected): **97,160 ok, 0 failed, 0 measured, 7 pending, 157 accepted** on x86_64 ELF, x86_64 PE and aarch64; `validate.py` recomputes every row (narrowphase and queries through a Python transcription on glibc's libm, corrections on MPFR) | the 18 broadphase tests; 12 type tests; 32 narrowphase tests; 18 raycast tests (3 of `raycast.rs`, 15 of `backend_2d/tests.rs`, the 9 world-level ones on a test-local world until M3); Cyrius-only groups; 250 assertions, plus 14 hypot assertions in `crmath.tcyr` | 16 Cyrius-only (1,000 boxes: tree 4.7 ms, hash 1.95 ms against Rust's whole step at 0.51 ms; ball mass 545 ns against Rust's whole `add_body_with_collider` at 253 ns, S5; contacts 132 ns – 3.18 µs; rays 389 ns – 2.11 µs; sphere overlap 872 ns; `hypot` 168 ns against ganita's 33) | part 1: `src/aabb_tree.cyr`, `src/spatial_hash.cyr`. Part 2: `src/b2_types.cyr`, private constants generated. Part 3: `src/b2_narrowphase.cyr`. Part 4: correctly rounded `hypot` (ADR 0011 amendment), `src/b2_raycast.cyr`; the `PhysicsState2d` query loops pending M3. Surface parity 394/394 (4 pending M3). Next: M3 |
| M3 2D dynamics | ✅ done 2026-11-09 | b2_state 411 (scripts ending in the full backend dump; 3 orphan rows mapped, ADR 0008 #1) + b2_step 214 (single steps, class S/C, contact-free; 4 corrected) + b2_contact 418 (single steps with contacts, both broadphases; 2 corrected) + world_step 198 (world steps with particles, emitters and fields), and json 937 → 1,092 (the particle types): **98,548 ok, 0 failed, 0 measured, 7 pending, 165 accepted** on x86_64 ELF, x86_64 PE and aarch64; `validate.py` replays every script and transcribes the whole SI step and the world's particles | the 5 state tests, the mouse-joint test and the 24 stepping tests of `backend_2d/tests.rs`; the 22 world tests, 8 particle tests, 9 integration and 26 2D regression tests (class R); Cyrius-only groups; 210 assertions. Not ported: the XPBD tests (ADR 0008 #6), the snapshot tests (M6) | 24: the 6 state benches against Rust (10–52×, S5); Rust's `world/step_*` through the world (3.5–6.5×); its 5 `particles/*` rows (4.3–37×, S5 on the particle records); the correctly rounded pow 755 ns | part 1: `src/b2_state.cyr`. Part 2: `src/b2_joints.cyr`. Part 3: `src/b2_contacts.cyr`, `step`. Part 4: `_im_cr_pow` (ADR 0011 amended, 0009 superseded), `src/particle.cyr`, `src/world.cyr`, particle JSON in `src/json.cyr`; surface parity 507/507 (23 modules, 3 snapshot fns pending M6) |
| M4 3D collision | ✅ done 2026-11-09 | b3_math 7,316 (glam's operations, through hisab 1.4.0's re-export) + b3_body 234 (bodies and mass properties from world dumps) + b3_narrow 16,340 (every handled shape pair through `contact_3d`, class A) + b3_ray 4,138 (rays through the world API, class C; 2 corrected) + math 326 signum rows: 3D **28,026 ok, 0 failed, 2 accepted (ADR 11)**, 2D **98,874 ok**, all exact or 0 ulp on x86_64 ELF, x86_64 PE and aarch64 with identical digests; `validate.py` recomputes every row (the narrowphase and the raycast through Python transcriptions) | the type-level tests, the 37 narrowphase tests and the 7 raycast tests of `backend_3d/tests.rs` (the state-level ones on test-local arenas until M5), with Cyrius-only groups (a 600-step free spin, flat heaps, the GJK scratch reuse, the unhandled pairs, layers); 111 assertions | 24: 20 Cyrius-only (rotation 41 ns, inverse 50 ns; contacts 0.24–8.5 µs, 6–20× Rust's scratch timing; GJK + EPA 30–55 µs, 6–11×; ray tests 141–630 ns) and `queries/raycast_100_bodies` on the 3D build, 21.1 µs against 2.28 µs | part 1: [ADR 0012](../adr/0012-impetus-owned-3d-math-stopgap.md) (impetus-owned 3D math and GJK/EPA until S1/H9; H9 filed); `src/math3.cyr`, `src/b3_types.cyr`; `[lib.3d]`, `dist/impetus-3d.cyr`, `tests/lib_3d.cyr`, `tests/parity_3d.tcyr`; a cyrius field-store defect worked around and filed. Part 2: `src/b3_narrowphase.cyr`. Part 3: `src/b3_raycast.cyr` (Rust's 3D world has no overlap queries). Surface parity 604 (26 modules; the 3D state's two raycast methods pending M5). Next: M5 |
| M5 3D dynamics | ✅ done 2026-11-10 | b3_state 411 (scripts on `PhysicsState3d` ending in the 3D backend dump, class C; 5 corrected; 3 orphan rows mapped, ADR 0008 #1) + the 1,200 `f64::atan2` rows now gated at 0 ulp (`math.cr.tsv` 22): 3D **28,429 ok, 0 failed, 10 accepted (ADR 11: 7)**, 2D **98,873 ok, 0 failed**, all exact or 0 ulp on x86_64 ELF, x86_64 PE and aarch64 with identical digests (`f64::atan2` included); `validate.py` replays every script on a model of the 3D state + b3_step 333 (single steps at mask 0 on the contact-free sub-step, class C, all 0 ulp; `validate.py` transcribes the sub-step): 3D **28,762 ok, 0 failed** after part 2 + b3_contact 450 (single steps with contacts, both broadphases, class C; 1 corrected, a glibc `hypot`), the b3_step rows through `step`, and b3_state 414 with ADR 0008 #5's 3 unsupported scripts mapped: 3D **29,214 ok, 0 failed** after part 3 + b3_world 190 (world steps in 3D: emitters, fields, particles against every 3D shape, tumbling bodies, sub-steps, joints, 30 seeded scenes; class C, all 0 ulp, none corrected; `validate.py` transcribes the 3D particle collisions): 3D **29,404 ok, 0 failed, 11 accepted** after part 4, all on x86_64 ELF, x86_64 PE and aarch64 | the state-level tests of `backend_3d/tests.rs` that need no step, with Cyrius-only groups (refusals, a joint's reference rotation, removals, a flat heap): 25 assertions; the four state-level raycast tests now on the state; `crmath.tcyr` 61 → 78 assertions (atan2); part 2: the 11 stepping tests whose colliders can't meet, on `tests/step3.cyr`'s composition with its broadphase guard, and Cyrius-only groups: 31 assertions in `b3_joints.tcyr`; part 3: the 10 stepping tests with contacts and Cyrius-only groups, 23 assertions in `b3_contacts.tcyr`; `b3_joints.tcyr` on `step` (31); part 4: the 24 world tests, 8 particle tests, 9 integration tests and 12 shared regressions on the 3D world from the same source as on 2D (`tests/world_cases.cyr`, `tests/regression_cases.cyr`), with 19 Cyrius-only assertions (the 3D particle contacts, sensors, hulls, offset colliders, the empty overlaps, the unread solver kind, the world against the backend's step, a flat heap) and Rust's 3D-only `substeps_are_honoured_in_3d`: `b3_world.tcyr` 74, `b3_regressions.tcyr` 29 | 9: the six `bodies/*` and `forces/apply_*` against Rust's 3D build (7.6–48×), `b3/get_body_state` 338 ns, `math/atan2_cr` 634 ns against ganita's 62 ns; part 2: 3 Cyrius-only (the gyroscopic velocity step 1.21 µs, a 20-link chain's joint work 59.9 µs and its contact-free frame 97.4 µs; Rust's 3D step comparison waits for part 3); part 3: `world/step_*` and `substep/step_100_bodies_substep4` against Rust's 3D build, and a resting stack `world/step_*` 5.4–11× against Rust's 3D build, after a broadphase change that made the spatial hash 3.4× faster for both profiles; part 4: those rows through the world (5.6–11×; the world adds about 35 ns a frame) and Rust's 5 `particles/*` rows on the 3D build (5.2–37×, as in 2D; the floor row 8.4×, 1.84 µs of its frame the 3D broadphase's rebuild for one box); ADR 0008 #4: `mutation/get_body_state` 3.5× (a copy now, 338 → 40 ns) and `set_body_state` 14× on the 3D world | part 1: a correctly rounded atan2 (ADR 0011 amended; 1,355,242 MPFR pairs; added to ganita's tier filing); `src/b3_state.cyr`; the oracle's `dump3d.rs`, with `dumpcore.rs` shared by both dumps. Surface parity 627 (27 modules). Part 2: `src/b3_joints.cyr`, `src/b3_solver.cyr`'s contact-free stages, `tests/step3.cyr`. Surface parity 632 (28 modules). Part 3: `src/b3_solver.cyr` complete (broadphase, narrowphase stage, wake, shock order, contact solver, `step`); ADR 0008 #5 implemented (part 1 had missed it). Surface parity 637 (29 modules). Part 4: `src/world.cyr` shared by both profiles (§4 step 3), over `src/b2_world.cyr` (M3's 2D code, moved) and `src/b3_world.cyr`; the oracle's `m_b3_world.rs`. Surface parity 637 (29 modules). Then ADR 0008 #4 (§3.3): the 3D `BodyState` with its full quaternion and angular velocity, one `ImBodyState` per profile, compared against the oracle's dump (`port-parity.md` §24). Next: M6 |
| M6 Snapshots | ◐ parts 1–4 done 2026-11-10 | no oracle (ADR 0013): checked by round trip and step-equivalence; `constants.tsv`'s `MAX_SNAPSHOT_INDEX` row now generated and replayed (2D pending 7 → 6): 2D **98,874 ok, 0 failed, 6 pending**, 3D **29,404 ok**, on x86_64 ELF, x86_64 PE and aarch64 | part 1: `tests/snapshot.tcyr`, 31 assertions: Rust's `restore_rejects_huge_handle_index` and `restore_replaces_force_fields` (one of its two `#[ignore]`d) at the struct; an exact round trip in five scenes; step-equivalence (the restored world bit for bit, events and next handles included) in eight runs over contacts, sleep, every joint type, breaking, particles, removals, sub-steps and the AABB tree; 13 refusals, each leaving the world unchanged; a flat heap; three restore mutations each fail 5–6 checks; part 2: Rust's five byte-level `serialize.rs` tests and its two `#[ignore]`d restore tests through bytes, byte round trips, the header's refusals, every truncation, every one-bit change, 1,500 seeded corruptions behind a recomputed checksum (`fuzz_serialize`'s contract), a cross-target pin: 49 assertions; two codec mutations caught; part 3: `snapshot_serde_json`, JSON round trips, step-equivalence through JSON, an edit, seven refusals, a NaN refused by `to_json`: 58 assertions; part 4: the shared cases (`tests/snapshot_cases.cyr`, Rust's nine snapshot tests and the byte and JSON tests, 30) on 3D, with a tumbling body's full state through both forms, eight 3D step-equivalence runs and 13 refusals including ADR 0008 #5: `b3_snapshot.tcyr` 59 | part 1: 3 Cyrius-only struct rows on Rust's `snapshot/*` scenes (capture empty 1.39 µs, capture 100 bodies 208 µs, restore 234 µs); part 2: Rust's three `snapshot/*` rows through the binary form (serialize empty 1.7×, 100 bodies 19×, round trip 11×; the capture dominates); part 3: 2 Cyrius-only JSON rows (write 2.31 ms, read and restore 3.70 ms for 100 balls); part 4: Rust's `snapshot/*` on the 3D build (1.7×, 29×, 13×) | part 1: [ADR 0013](../adr/0013-complete-snapshots.md) (the struct of the backend's own records, a checksummed binary form, JSON; the user's three choices); `src/snapshot.cyr`, `src/b2_snapshot.cyr`, `src/world_snapshot.cyr`; `IM_MAX_SNAPSHOT_INDEX` generated; surface parity 662 (30 modules: `serialize` joins; `serialize_world` / `deserialize_world` pending part 2). Part 2: the binary form, version 1 (ADR 0013); surface parity 662, nothing pending. Part 3: the JSON form (ADR 0013 amended: non-finite values refused, the S3 handle limit). Part 4: `src/b3_snapshot.cyr`, the 3D profile's three forms. Next: part 5, the fuzz port, profiling and the close |
| M7 Release 2.0.0 | ☐ | | | | |
