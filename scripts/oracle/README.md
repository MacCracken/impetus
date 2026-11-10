# scripts/oracle: what the port derives from the Rust oracle

`oracle` drives the frozen Rust impetus 1.4.0 in [`rust-old/`](../../rust-old/), with its hidden
`oracle` hooks (port plan §4 step 0), and writes two kinds of output:
- **The golden corpus, `tests/golden/<module>.tsv`** (port plan §4 step 5). It has one row per
  call, with every float as its IEEE-754 bit pattern. The Cyrius parity harnesses
  (`tests/parity_2d.tcyr`, and `tests/parity_3d.tcyr` from M4) replay every row against the
  port and check it within its tolerance class.
- **The generated Cyrius sources**: `src/constants.cyr` (the public constants), `src/constants_private.cyr` (the
  ported modules' private ones), and data modules as milestones need them (step 6). No value the port reads is
  typed by hand.

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
| `math` | b2 | M0 | 19,165 | `src/m_math.rs` | — |
| `config`, `material`, `units`, `force` | b2 | M1a | 293, 2,046, 2,236, 1,308 | `src/m_<module>.rs` | — |
| `fmtf`, `json` | b2 | M1b | 37,576, 937 | `src/m_fmtf.rs`, `src/m_json.rs` | — |
| `spring`, `types` | b2 | M1b | 1,000, 1 | `src/m_spring.rs`, `src/m_types.rs` | — |
| `broadphase` | b2 | M2 | 76 | `src/m_broadphase.rs` | — |
| `b2_mass` | b2 | M2 | 75 | `src/m_b2_mass.rs` | — |
| `b2_narrow` | b2 | M2 | 17,854 (+ 3 corrected) | `src/m_b2_narrow.rs` | — |
| `b2_ray` | b2 | M2 | 14,754 (+ 39 corrected) | `src/m_b2_ray.rs` | — |
| `b2_state` | b2 | M3 | 411 | `src/m_b2_state.rs` | — |
| `b2_step` | b2 | M3 | 214 (+ 4 corrected) | `src/m_b2_step.rs` | — |
| (private constants) | b2 | M2 | — | — | `src/constants_private.cyr` (`src/t_private.rs`: every library `const` of `aabb_tree.rs`, `spatial_hash.rs`, `backend_2d/joints.rs` and `backend_2d/types.rs`, emitted or excused) |

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
- A handle is `i:` of its word: two's complement above `i64::MAX`, the port's own representation.
  A payload enum (`ColliderShape`, `JointType`, `CollisionEvent`) is `e:<Enum>.<Variant>` then the
  variant's fields; a `Vec` is `v:<len>` then its elements (`flat.rs`).
- Broadphase rows (`broadphase.tsv`) give the boxes in insertion order (`v:N`, then min and max per box; the hash
  rows start with the cell size) and the sorted candidate pair set (`v:P`, then the two input indices per pair).
  The structures are shared by both backends, so the 2D build emits the 3D rows too.
- Mass rows (`b2_mass.tsv`, the 2D build only) come from a world dump. Each row builds a `PhysicsWorld` with one
  body (`e:BodyType`, `fixed_rotation`), adds its colliders through the public `add_collider`, and reads the body
  back from `oracle_dump()`. Inputs: `v:<colliders>`, then per collider its shape (`flat::shape`), offset x and y,
  density, and `n:` or the explicit mass. Outputs: mass, inertia (about the body origin), inv_mass, inv_inertia.
- Narrowphase rows (`b2_narrow.tsv`, the 2D build only) go through the hooks `contacts_2d` and `shapes_touch_2d`.
  Inputs: shape A (`flat::shape`), A's x, y and rotation, then shape B and its pose. `b2::contacts` (class C)
  outputs `v:N` and each contact's normal x, y (A to B), depth and world point x, y, in generation order;
  `b2::shapes_touch` (class A) outputs `i:0` or `i:1`. Every box has finite, non-negative half extents, because a
  round shape against a box reaches Rust's `f64::clamp(-h, h)`, which panics on an inverted range.
- Query rows (`b2_ray.tsv`, the 2D build only) build a world of static bodies, one collider each, through the public
  API. Inputs: `v:<bodies>`, then per body its x, y, rotation, shape, collider offset x, y and layer (`i:`); then the
  query. `b2::raycast` (class C) takes origin x, y, direction x, y, max distance and layer mask
  (`raycast_filtered`), and outputs `n:` or the hit's collider handle, point x, y, normal x, y and distance.
  `b2::overlap_sphere` (centre, radius) and `b2::overlap_aabb` (min, max), class A, output `v:N` and the handles found.
  The emitters share their shape builders (`src/shapes.rs`).
- State rows (`b2_state.tsv`, the 2D build only) are scripts. Inputs: `v:<ops>`, then each op as `e:Op.<Name>` and its
  arguments (descriptors as `flat.rs` writes them, handles as `i:` words). Outputs: each op's result (a handle, `i:0`,
  an error `E:`, a `BodyState`, a count), then the backend dump, `src/dump2d.rs`. The dump gives the three arenas'
  slots (occupied, generation, next free), free-list heads and staleness, every body, collider and joint field,
  `body_colliders` in handle order, the manifolds in key order and the tracker. `b2::state_script_orphan` rows add a
  collider or joint for a missing body (ADR 0008 #1).
- Single-step rows (`b2_step.tsv`, class S at class C, the 2D build only): inputs are the world config (`flat::config`,
  its `step` the frame index) and the backend dump at frame k, taken after that frame's forces; outputs are the dump at
  k + 1 and the frame's events. M3b's scenes give every collider collision mask 0, so nothing pairs and the contact
  stages are empty.
- `Spring::step` rows (`spring.tsv`) carry a note `[<regime> S=<n>]`: the regime the transition took
  and its **sensitivity**, the most the two outputs move, in ulps, when one exp, sin or cos result
  on the path moves by one ulp. `m_spring.rs` computes it from a transcription of `transition` and
  `step` that it checks bit for bit against the oracle on every row, so the class-C bound comes from
  the port-parity rule (`docs/development/port-parity.md` §2) rather than from the port's results.

## The corrected oracle and the crmath tables (ADR 0011)

exp, sin, cos and hypot are correctly rounded in the port (`src/crmath.cyr`), and glibc 2.44 is not (it misrounds
about 0.1–0.3 % of their arguments). A class-C row's reference is therefore the **corrected oracle**: the oracle's own
operations with every exp, sin and cos result correctly rounded. Where glibc rounded every call correctly, that is
the golden row and nothing is written. Where it didn't, `tests/golden/<module>.cr.tsv` holds the corrected outputs
(kimiya's format, its ADR 0006):

```text
fn <TAB> inputs <TAB> corrected outputs <TAB> note
```

after five `#` header lines. The fn and inputs fields are the golden row's, byte for byte (the key). The note gives
the oracle's outputs, then each call glibc misrounded with both values. The harness compares such a row against the
corrected outputs only (accepted under ADR 11). A correction that matches no golden row fails its module.

`validate.py` builds the corrected oracle with MPFR 4.x (`mpfr_ref.py`, ctypes on `libmpfr.so.6`, no Python
package; copied from kimiya). It also renders `src/crmath_data.cyr` from `crtables.py`: 2/π, π/2 and ln 2 for the
slow path, and the fast paths' split constants, all from π and ln 2 computed exactly in integer arithmetic and
checked against MPFR to 2,048 bits.

```sh
python3 -I scripts/oracle/validate.py .           # checks both are current
python3 -I scripts/oracle/validate.py . --write   # rewrites them
```

The generator owns neither file. Its `--check` treats a `<module>.cr.tsv` as part of its module (stale only if the
module has no emitter) and never lists it in `modules.txt`.

## `validate.py`

Standard-library Python, run with `-I`. It never calls the oracle:
- every file parses (header, row count, fields, classes, tokens), and `modules.txt` lists
  exactly the files present;
- `constants.tsv` equals the `pub const` literals in `rust-old/src`;
- every class-A and class-B `math` row is recomputed in Python in Rust's operation order (7,197
  rows at M0): sqrt, abs, floor, ceil, min/max (NaN-ignoring; the second operand when equal,
  as rustc 1.99.1 lowers them), clamp, is_nan, is_finite, to_radians, powi2, powi3;
- class-C rows are checked for consistency: `sin_cos` equals `sin` and `cos`, and hypot of a
  Pythagorean triple is exact;
- the M1a value rows (config verdicts, combine rules, `effective_static_friction`, `to_radians`,
  magnitudes, gravity) are recomputed;
- every `fmtf` string reads back to its bits, and serde_json's digits are Python's `repr`;
- every JSON document the oracle reads or writes is read by Python's `json`, and its floats equal
  the oracle's fields (any JSON value: serde reads structs from arrays and unit variants from
  strings too);
- every `spring` row is recomputed: the constructors, `is_settled`, and `step` through a Python
  transcription. Python's `math` calls the same glibc `exp`, `sin` and `cos` as Rust's f64 methods,
  so the step rows match bit for bit;
- every `broadphase` row is recomputed: the tree's pairs by brute force on the fattened boxes, the hash's by an
  independent rasterisation, the automatic cell size;
- every `b2_narrow` row is recomputed by a Python transcription of the narrowphase on Python's libm (glibc, as the
  oracle's), bit for bit. The same transcription on MPFR's sin and cos writes `b2_narrow.cr.tsv`. Its folds are
  explicit loops, because Python 3.12's `sum()` of floats is compensated;
- every `b2_ray` row is recomputed by the same transcription extended with the ray tests, `world_aabb` and the overlap
  loop, on glibc's sin, cos and hypot (hypot through ctypes: CPython's `math.hypot` is its own implementation). On
  MPFR's it writes `b2_ray.cr.tsv`;
- every `b2_step` row is recomputed by a transcription of the contact-free step (sub-steps, integration, the eight
  joint types, islands and sleep, `finish_step`) on glibc's sin, cos and hypot; the same on MPFR's writes
  `b2_step.cr.tsv`. It asserts every collider's mask is 0;
- every `b2_state` script is replayed on a Python model of the arenas (Rust's free list and generations) and the
  state, rebuilding every result and dump token;
- every `b2_mass` body is recomputed: its own `shape_props` and `polygon_mass_props` in Rust's operation order,
  the floors, the parallel-axis sum and `refresh_inverse_mass`;
- `src/constants_private.cyr` holds every library `const` of the ported modules, each equal to Python's parse of
  the Rust literal;
- the corrected oracle and `src/crmath_data.cyr` are current (above).

A tampered row makes it exit 1.
