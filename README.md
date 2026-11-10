# Impetus

**Impetus** (Latin: *impetus*, driving force, the medieval theory of why objects keep moving after being thrown) is
the physics engine for [AGNOS](https://github.com/MacCracken/agnosticos): 2D and 3D rigid bodies, collision
detection, contact and joint constraints, spatial queries, physics particles and standalone springs, with
deterministic stepping and serializable state.

impetus **2.0** is a Cyrius library: a parity port of the Rust crate **1.4.0**, which stays frozen in
[`rust-old/`](rust-old/) as the reference implementation. It is checked against Rust value by value:
- **128,278 golden rows** (2D 98,874, 3D 29,404) replay Rust's own results within their tolerance class on x86_64
  Linux, x86_64 Windows (PE) and aarch64, with the same output bits on all three. Every single-step, contact and
  world-step row matches at 0 ulp. 177 more rows are accepted under an ADR: inputs the 2.0 API refuses (ADR 0008),
  and the few where glibc misrounds and the port gives the correctly rounded value (ADR 0011). 6 wait on bayan (S3,
  S4); see [port parity](docs/development/port-parity.md).
- **The Rust tests:** all 432 are ported to a test of the same name or accounted for in the
  [Rust-test ledger](docs/development/rust-test-ledger.md) (388 ported, 17 not applicable, 27 deferred with the
  bridges).
- **Snapshots** have no oracle. A restored world must step to the original's bits and hand out its handles
  ([ADR 0013](docs/adr/0013-complete-snapshots.md)), and the fuzz targets run on both profiles.

2.0 changes the API in seven deliberate places ([ADR 0008](docs/adr/0008-port-api-divergences.md)). Moving from the
Rust crate: [the migration guide](docs/guides/migration-2.0.md). The [port plan](docs/development/impetus-port-plan.md)
records the decisions, and [`state.md`](docs/development/state.md) the current state.

## Features

- **2D and 3D**, as two bundles with the same names: `dist/impetus.cyr` and `dist/impetus-3d.cyr`.
- **Deterministic:** a fixed timestep, ordered iteration and no hash-map order. The same inputs give the same bits
  on every supported target: each transcendental the step calls is impetus's own and correctly rounded
  ([ADR 0011](docs/adr/0011-correctly-rounded-exp-sin-cos.md)).
- **Bodies and shapes:** dynamic, static and kinematic bodies; ball, box, capsule, segment and convex-hull
  colliders, plus trimesh and heightfield (accepted, no contacts yet, as in Rust 1.4.0); sensors, collision layers
  and masks.
- **Solver:** sequential impulses with warm starting, the block solver, restitution, split impulse and sub-steps;
  islands and sleep in 2D.
- **Joints:** fixed, revolute, prismatic, spring, distance, rope, wheel and mouse, with limits, motors and
  breaking. The 3D profile refuses revolute, prismatic and motors, which Rust 1.4.0's 3D world accepted and ignored.
- **Queries and events:** raycasts (with a layer filter), sphere and AABB overlaps in 2D, and Started / Ongoing /
  Stopped collision events per collider pair, once per step.
- **Snapshots:** a world's whole state as an editable struct, a checksummed binary form and JSON. Restore is exact
  and leaves the world unchanged on any error.
- **JSON:** each type Rust 1.4.0 serializes reads and writes the document serde_json does for it
  ([ADR 0010](docs/adr/0010-serde-compatible-json.md)); the 3D `ImBodyState` follows its 2.0 fields.
- **Particles:** particles, emitters with sub-emitters, force fields, drag and collider hits.
- **Springs:** a standalone damped harmonic oscillator in 1D, 2D and 3D for UI and compositor animation, solved
  exactly, so any frame time is stable.
- **Units and materials:** `ImQuantity` with its physical unit; ice, rubber, wood, steel and bouncy presets.
- **No aborts:** every fallible call returns a `Result` with an `IM_ERR_*` code. The one exception is noted under
  [known limitations](#known-limitations).

## Using it

Declare it in your `cyrius.cyml`; `cyrius deps` vendors it with the stdlib modules it needs:

```toml
[deps.impetus]
git = "https://github.com/MacCracken/impetus.git"
tag = "2.0.0"
modules = ["dist/impetus.cyr"]        # the 2D profile; "dist/impetus-3d.cyr" for 3D
```

A ball dropped on a floor and pushed sideways (Rust 1.4.0's README example):

```cyrius
include "lib/impetus.cyr"

fn main(): i64 {
    alloc_init();
    var world: ImPhysicsWorld = ImPhysicsWorld_new(ImWorldConfig_default());

    # A dynamic rubber ball
    var bd: ImBodyDesc = ImBodyDesc_default();
    bd.body_type = IM_BODY_DYNAMIC;
    bd.position = ImVec3_new(0.0, 10.0, 0.0);
    var ball = world.add_body(bd);
    var bc: ImColliderDesc = ImColliderDesc_new(ImColliderShape_ball(0.5));
    bc.material = ImPhysicsMaterial_rubber();
    var t1, c1 = world.add_collider(ball, bc);

    # A static wooden floor
    var fd: ImBodyDesc = ImBodyDesc_default();
    fd.body_type = IM_BODY_STATIC;
    var floor = world.add_body(fd);
    var fc: ImColliderDesc = ImColliderDesc_new(ImColliderShape_box(50.0, 0.5, 50.0));
    fc.material = ImPhysicsMaterial_wood();
    var t2, c2 = world.add_collider(floor, fc);

    # A sideways force, then one step
    world.apply_force(ball, ImForce_new(5.0, 0.0, 0.0));
    world.step();

    var state: ImBodyState;
    var t3, e3 = world.get_body_state(ball, &state);
    var vx: f64 = state.linear_velocity.x;          # > 0: pushed right
    var vy: f64 = state.linear_velocity.y;          # < 0: falling
    world.free();
    if (is_ok(t1) + is_ok(t2) + is_ok(t3) != 3 || vx <= 0.0 || vy >= 0.0) { return 1; }
    return 0;
}

var r = main();
sys_exit_group(r);
```

A spring for UI, with no world:

```cyrius
include "lib/impetus.cyr"

fn main(): i64 {
    alloc_init();
    var pos: ImSpring2d = ImSpring2d_critically_damped(ImVec2_new(0.0, 0.0), ImVec2_new(500.0, 300.0), 400.0);
    # Each frame:
    pos.step(1.0 / 60.0);
    var p: ImVec2 = pos.position();
    var x: f64 = p.x;
    var y: f64 = p.y;
    if (x <= 0.0 || x >= 500.0 || y <= 0.0 || y >= 300.0) { return 1; }
    return 0;
}

var r = main();
sys_exit_group(r);
```

**The API follows Rust's** (the [migration guide](docs/guides/migration-2.0.md) has the full mapping):
- **Names.** Structs are `Im<Name>` with methods of the Rust names (`world.add_body(desc)`, `spring.step(dt)`);
  constructors are `Im<Name>_<constructor>` (`ImSpring2d_critically_damped`); enum members and constants carry
  `IM_`. Handles are 64-bit words, as Rust's `u64` handles.
- **Results.** A fallible call returns a value-form `Result`: `var t, v = world.add_collider(b, desc);` then
  `is_ok(t)`. On `Err`, `v` is an `IM_ERR_*` code, and `im_error_message(v)` gives its text.
- **Options and collections.** `Option<T>` returns become an out-pointer and a 1/0 return (`world.raycast(.., &hit)`);
  `Vec` returns fill an `ImWordList` the caller owns; slices are a pointer and a count.
- **Memory.** What you create, you free: `world.free()`, and `free()` on snapshots, word lists and shapes that hold
  arrays. A world's storage is its own, and a step allocates nothing that outlives it.

## Profiles

| Bundle | Profile | Replaces (Cargo, 1.4.0) |
|---|---|---|
| `dist/impetus.cyr` | 2D (`[lib]`), the default | the `2d` feature |
| `dist/impetus-3d.cyr` | 3D (`[lib.3d]`) | the `3d` and `full` features |

Snapshots and JSON are in both bundles; Rust's `serialize` feature has no counterpart. A program includes one bundle.

## Known limitations

- **JSON integers above `i64::MAX`** (bayan, S3): a handle whose slot was reused more than 2^31 times is written to
  JSON but can't be read back. The binary snapshot holds it.
- **Duplicate JSON keys** (bayan, S4) are refused, where serde ignores repeats of unknown fields.
- **Out of memory in the JSON writers:** the stdlib's `str_builder` exits the process when it can't grow.
- **Speed:** the median benchmark is about 10× Rust 1.4.0's in 2D and 13× in 3D, and a whole step is 2.6–11× and
  5.7–17× ([the record](docs/benchmarks-rust-v-cyrius.md)). These are first numbers; the upstream issues behind
  them are filed (port plan §8).
- **Not yet ported:** the dravya, sharira and ushma bridges, which wait for a Cyrius consumer (port plan §9).

## Where it sits

impetus is a pure physics library: no ECS, no rendering, no scene loading. Its consumers are
[kiran](https://github.com/MacCracken/kiran) (game engine), [joshua](https://github.com/MacCracken/joshua)
(simulation) and [aethersafha](https://github.com/MacCracken/aethersafha) (compositor window physics), all of which
used the Rust crate; none consumes the Cyrius library yet. Its math is its own while hisab's calls allocate
unscoped ([ADR 0012](docs/adr/0012-impetus-owned-3d-math-stopgap.md)), so a consumer's resolve carries only impetus
and a few stdlib modules.

## Building

Cyrius (the toolchain is pinned in `cyrius.cyml`):

```sh
cyrius deps                                 # vendor the stdlib and hisab (tests only) into lib/
cyrius build src/main.cyr build/impetus     # compile
cyrius test                                 # tests/*.tcyr: the parity harnesses and the ported Rust tests
cyrius bench                                # tests/*.bcyr
cyrius fuzz                                 # tests/*.fcyr: the fuzz targets on both profiles
cyrius audit                                # fmt, lint, docs, tests and benches
cyrius distlib --all                        # dist/impetus.cyr and dist/impetus-3d.cyr
```

The oracle and the checks against it:

```sh
cargo test --manifest-path rust-old/Cargo.toml --features serialize,oracle    # the Rust 1.4.0 oracle (2D)
cargo test --manifest-path rust-old/Cargo.toml --all-features                 # (3D)
cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml -- --check   # the corpus is current
python3 -I scripts/oracle/validate.py .               # the corpus, recomputed without Rust
python3 -I scripts/surface_parity.py .                # every Rust item has its Cyrius name
python3 -I scripts/rust_test_ledger.py . --check      # every Rust test is accounted for
python3 -I scripts/bench_record.py . --check          # every Rust bench has its twin
python3 -I scripts/doc_examples.py .                  # the docs' code blocks compile and run
```

## Documentation

- [Migration guide: Rust 1.4 to Cyrius 2.0](docs/guides/migration-2.0.md)
- [Getting started](docs/guides/getting-started.md)
- [CHANGELOG](CHANGELOG.md)
- [Port plan](docs/development/impetus-port-plan.md), [state](docs/development/state.md),
  [roadmap](docs/development/roadmap.md)
- [Port parity](docs/development/port-parity.md), [Rust-test ledger](docs/development/rust-test-ledger.md),
  [benchmarks against Rust](docs/benchmarks-rust-v-cyrius.md)
- [Architecture decisions](docs/adr/)
- The Rust 1.4.0 [architecture overview](rust-old/docs/architecture/overview.md) and
  [testing guide](rust-old/docs/guides/testing.md), kept for reference

## License

GPL-3.0-only. See [LICENSE](LICENSE).
