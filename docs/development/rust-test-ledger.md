# Rust-test ledger

> **Generated** by `python3 -I scripts/rust_test_ledger.py .`; `--check` gates it (port plan M7, the mela
> pattern). Every test of the frozen Rust oracle (`rust-old/`, impetus 1.4.0 with the `oracle` hooks) is
> ported to a Cyrius test group of the same name, or recorded here with its reason. The golden corpus
> (`tests/golden/`) checks every ported function on its grid besides; this ledger is about the oracle's own
> assertions. Profiles are those whose harnesses run the Cyrius files: the shared case files run on both.

**432 Rust tests:** 388 ported, 17 not applicable, 27 deferred.

| Rust file | Cyrius tests | Profiles | Rust tests | Ported | N/A | Deferred |
|---|---|---|---:|---:|---:|---:|
| `src/error.rs` | `tests/error.tcyr` | 2D | 3 | 3 | 0 | 0 |
| `src/config.rs` | `tests/config.tcyr` | 2D | 7 | 7 | 0 | 0 |
| `src/material.rs` | `tests/material.tcyr` | 2D | 14 | 14 | 0 | 0 |
| `src/units.rs` | `tests/units.tcyr` | 2D | 9 | 9 | 0 | 0 |
| `src/force.rs` | `tests/force.tcyr` | 2D | 11 | 11 | 0 | 0 |
| `src/body.rs` | `tests/types.tcyr` | 2D | 6 | 6 | 0 | 0 |
| `src/collider.rs` | `tests/types.tcyr` | 2D | 7 | 7 | 0 | 0 |
| `src/joint.rs` | `tests/types.tcyr` | 2D | 12 | 12 | 0 | 0 |
| `src/event.rs` | `tests/types.tcyr` | 2D | 6 | 6 | 0 | 0 |
| `src/query.rs` | `tests/types.tcyr` | 2D | 3 | 3 | 0 | 0 |
| `src/arena.rs` | `tests/arena.tcyr` | 2D | 10 | 10 | 0 | 0 |
| `src/contact_tracker.rs` | `tests/contact_tracker.tcyr` | 2D | 3 | 3 | 0 | 0 |
| `src/spring.rs` | `tests/spring.tcyr` | 2D | 16 | 16 | 0 | 0 |
| `src/aabb_tree.rs` | `tests/aabb_tree.tcyr` | 2D | 10 | 10 | 0 | 0 |
| `src/spatial_hash.rs` | `tests/spatial_hash.tcyr` | 2D | 8 | 8 | 0 | 0 |
| `src/backend_2d/types.rs` | `tests/b2_types.tcyr` | 2D | 2 | 2 | 0 | 0 |
| `src/backend_2d/narrowphase.rs` | `tests/b2_narrowphase.tcyr` | 2D | 11 | 11 | 0 | 0 |
| `src/backend_2d/raycast.rs` | `tests/b2_raycast.tcyr` | 2D | 3 | 3 | 0 | 0 |
| `src/backend_2d/tests.rs` | `tests/b2_types.tcyr`<br>`tests/b2_narrowphase.tcyr`<br>`tests/b2_raycast.tcyr`<br>`tests/b2_state.tcyr`<br>`tests/b2_joints.tcyr`<br>`tests/b2_contacts.tcyr`<br>`tests/config.tcyr`<br>`tests/material.tcyr` | 2D | 87 | 84 | 3 | 0 |
| `src/backend_3d/tests.rs` | `tests/b3_types.tcyr`<br>`tests/b3_narrowphase.tcyr`<br>`tests/b3_raycast.tcyr`<br>`tests/b3_state.tcyr`<br>`tests/b3_joints.tcyr`<br>`tests/b3_contacts.tcyr` | 3D | 83 | 83 | 0 | 0 |
| `src/particle.rs` | `tests/world_cases.cyr` | 2D, 3D | 8 | 8 | 0 | 0 |
| `src/world.rs` | `tests/world_cases.cyr` | 2D, 3D | 28 | 24 | 4 | 0 |
| `src/serialize.rs` | `tests/snapshot_cases.cyr` | 2D, 3D | 6 | 6 | 0 | 0 |
| `src/lib.rs` | `tests/world_cases.cyr` | 2D, 3D | 1 | 1 | 0 | 0 |
| `src/bridge.rs` | — | — | 27 | 0 | 0 | 27 |
| `tests/integration.rs` | `tests/regression_cases.cyr` | 2D, 3D | 9 | 9 | 0 | 0 |
| `tests/regressions.rs` | `tests/regression_cases.cyr`<br>`tests/regressions.tcyr`<br>`tests/b3_regressions.tcyr`<br>`tests/b3_joints.tcyr`<br>`tests/snapshot_cases.cyr` | 2D, 3D | 31 | 30 | 1 | 0 |
| `tests/oracle.rs` | — | — | 9 | 0 | 9 | 0 |
| doc tests (`README.md`) | `tests/spring.tcyr`<br>`tests/world_cases.cyr` | 2D, 3D | 2 | 2 | 0 | 0 |
| **Total** | | | **432** | **388** | **17** | **27** |

## Doc tests

`src/lib.rs` takes `README.md` as the crate's docs, so its runnable examples are doc tests. The `ignore`
and `text` fences in `spring.rs` and `bridge.rs` are not run.

- `rust-old/README.md`, the Quick Start example: `tests/world_cases.cyr`, `doc_readme_quick_start`.
- `rust-old/README.md`, the Spring Animation example: `tests/spring.tcyr`, `doc_readme_spring_animation`.

## Rust's `#[ignore]`d tests

Rust 1.4.0's incomplete snapshot made these fail, so they were ignored. The port's complete snapshot
(ADR 0013) runs them.

- `tests/regressions.rs` `restore_preserves_handle_allocation`: ported, live, in `tests/snapshot_cases.cyr`.
- `tests/regressions.rs` `restore_replaces_force_fields`: ported, live, in `tests/snapshot_cases.cyr`.

## Tests without a port

| Rust file | Test | Status |
|---|---|---|
| `src/backend_2d/tests.rs` | `xpbd_ball_on_floor_rests` | n/a: XPBD is not carried. `config_validate` refuses it with IM_ERR_UNSUPPORTED (ADR 0008 #6, port plan §3.2), which `adr_0008_xpbd_refused` in `tests/config.tcyr` checks |
| `src/backend_2d/tests.rs` | `xpbd_zero_gravity_no_drift` | n/a: XPBD is not carried. `config_validate` refuses it with IM_ERR_UNSUPPORTED (ADR 0008 #6, port plan §3.2), which `adr_0008_xpbd_refused` in `tests/config.tcyr` checks |
| `src/backend_2d/tests.rs` | `xpbd_distance_joint_maintains_length` | n/a: XPBD is not carried. `config_validate` refuses it with IM_ERR_UNSUPPORTED (ADR 0008 #6, port plan §3.2), which `adr_0008_xpbd_refused` in `tests/config.tcyr` checks |
| `src/world.rs` | `xpbd_gravity_moves_body` | n/a: XPBD is not carried. `config_validate` refuses it with IM_ERR_UNSUPPORTED (ADR 0008 #6, port plan §3.2), which `adr_0008_xpbd_refused` in `tests/config.tcyr` checks |
| `src/world.rs` | `xpbd_collision_prevents_fallthrough` | n/a: XPBD is not carried. `config_validate` refuses it with IM_ERR_UNSUPPORTED (ADR 0008 #6, port plan §3.2), which `adr_0008_xpbd_refused` in `tests/config.tcyr` checks |
| `src/world.rs` | `xpbd_collision_generates_events` | n/a: XPBD is not carried. `config_validate` refuses it with IM_ERR_UNSUPPORTED (ADR 0008 #6, port plan §3.2), which `adr_0008_xpbd_refused` in `tests/config.tcyr` checks |
| `src/world.rs` | `xpbd_joint_distance` | n/a: XPBD is not carried. `config_validate` refuses it with IM_ERR_UNSUPPORTED (ADR 0008 #6, port plan §3.2), which `adr_0008_xpbd_refused` in `tests/config.tcyr` checks |
| `tests/regressions.rs` | `xpbd_moves_kinematic_bodies` | n/a: XPBD is not carried. `config_validate` refuses it with IM_ERR_UNSUPPORTED (ADR 0008 #6, port plan §3.2), which `adr_0008_xpbd_refused` in `tests/config.tcyr` checks |
| `src/bridge.rs` | all 27 | deferred: the dravya, sharira and ushma bridges wait on a Cyrius consumer (port plan §3.2, §9 item 1); they port as a small parity-tested file from these tests |
| `tests/oracle.rs` | all 9 | n/a: the tests of the `oracle` hooks added to the frozen crate for this port (port plan §4 step 0), not of impetus 1.4.0's API. `scripts/oracle/` drives the hooks, and the golden corpus checks what they dump |
