# impetus port parity — measured baselines

Measurements that set the parity classes (port plan §5). The oracle is Rust impetus 1.4.0 (with its
hidden `oracle` hooks) on **glibc 2.44**, x86_64 with FMA: Rust std f64 methods call the host libm,
and the 3D backend's vector math is glam 0.29.3 through hisab 1.4.0. The port is Cyrius 6.7.18 with
ganita 1.2.15 and hisab 3.4.1.

## 1. Semantics probe (M0 step 4, 2026-11-09, x86_64 Linux ELF)

A compiled probe of each Cyrius counterpart. Bit patterns where they matter.

| Rust (std) | Cyrius | Result | Consequence |
|---|---|---|---|
| `f64::min(NaN, 1.0)`, `min(1.0, NaN)` → `1.0` both | `f64_min` | `min(NaN, 1)` = 1, but **`min(1, NaN)` = NaN** | asymmetric: Rust semantics need `_im_min` / `_im_max` (106 call sites) and `_im_clamp` (28) |
| `f64::min(+0, −0)` → −0 (the second operand, as rustc lowers it) | `f64_min` | −0 | same on this case; `_im_min` keeps it |
| `sin`, `cos` | `f64_sin`, `f64_cos` | the polyfill on x86_64 too (same bits as `_f64_sin_polyfill` / `_f64_cos_polyfill`) | identical on every target (§2) |
| `exp(−0.7)` | `f64_exp` (x87 on x86_64), `_f64_exp_polyfill` | equal at this point; they differ on 74 of 1,525 grid rows (§2) | the polyfill, for cross-target bits |
| `atan2(1, −1)` | `ganita_f64_atan2` | `0x4002d97c7f3321d2` (3π/4) | built on the native `f64_atan`: per-target bits (§2) |
| `powf(2, 0.5)` | `ganita_f64_pow` | `…3bcc` vs √2 `…3bcd`, **1 ulp** | class C |
| `hypot(3, 4)` | `ganita_f64_hypot` | exactly 5 | class C (≤ 2 ulp, §2) |
| `floor`, `ceil`, `sqrt` | builtins | exact | class A |
| `is_nan`, `is_finite` | — (no public predicate) | `x != x`; `x − x == 0` | `_im_is_nan`, `_im_is_finite` (M1) |
| `sin_cos` | — | two calls | `f64_sin` + `f64_cos` |

## 2. Corpus baselines (M0 step 8, 2026-11-09)

`tests/golden/math.tsv` (18,919 rows, 18 functions, glibc 2.44 x86_64 with FMA) replayed by
`tests/parity_2d.tcyr` through each Cyrius counterpart on **x86_64 ELF, x86_64 PE (wine) and aarch64
(qemu)**. Histogram = ulp distance from the oracle. The digest column is the harness's 64-bit FNV-1a
fold of every output word the port returned, in row order: equal digests mean bit-identical results
on every row.

| Rust → Cyrius (dispatched) | Rows | Every target | Digest, every target |
|---|---|---|---|
| sin_cos → `f64_sin` + `f64_cos` | 2,084 (4,168 values) | 0: 4,077 · 1: 91 | `0x155cec6fd3d57439` |
| sin → `f64_sin` | 2,084 | 0: 2,039 · 1: 45 | `0x7a5a391b471b452b` |
| cos → `f64_cos` | 2,084 | 0: 2,038 · 1: 46 | `0x872183aee82e102b` |
| atan2 → ganita's atan2 on `_f64_atan_polyfill` | 1,200 | 0: 962 · 1: 238 | `0x7d234dec79df8a94` |
| hypot → `ganita_f64_hypot` | 1,715 | 0: 1,301 · 1: 411 · 2: 3 | `0xd6d60db0f1471e6a` |
| exp → `_f64_exp_polyfill` | 1,525 | 0: 1,410 · 1: 115 | `0x50fac509f4200cbb` |
| powf → `ganita_f64_pow` | 1,030 | 0: 941 · 1: 88 · ∞: 1 (ADR 0009) | `0x8bfeead3e7d9c6ea` |
| sqrt, abs, floor, ceil (class A) | 3,152 | all 0 | `0x5298ceb67dbeff0d` |

The native forms, kept as shadow lines (reported, never gated):

| Shadow | x86_64 ELF and PE | aarch64 |
|---|---|---|
| `f64_exp` (x87 on x86_64) | 0: 1,484 · 1: 41, digest `0x87bbca9b888d91a1` | the polyfill: `0x50fac509f4200cbb` |
| `ganita_f64_atan2` (on the native `f64_atan`) | 0: 969 · 1: 231, digest `0x437185336b2f7645` | the polyfill form: `0x7d234dec79df8a94` |

**Decisions.**
- **exp and atan2 use the polyfill forms** (port plan §7.1, ADR 0008 #7). They are the only primitives
  whose native bits differ between targets. The polyfills are 1 ulp from glibc slightly more often on
  x86_64 (exp 115 vs 41 rows, atan2 238 vs 231), all within K, and give the same bits everywhere. The
  atan2 is ganita's own code with `_f64_atan_polyfill` in place of `f64_atan`; it lives in the
  harness as `_cand_atan2_poly` until M1 moves it to `src/mathx.cyr` as `_im_atan2`.
- **Class-C bound for the math primitives: K = 2 ulp** on every target. Each side documents about
  1 ulp, so two implementations can be 2 apart; the observed maximum is 1, except hypot's 3 rows at 2.
  Set in `_par_c_bound` and gated from now on.
- **One accepted row:** `powf(-1, DBL_MAX)`, ganita's |x| == 1 defect ([ADR 0009](../adr/0009-accept-ganita-pow-minus-one-divergence.md)).
- **Pending until the M1 helpers:** min, max, clamp, is_nan, is_finite, to_radians, powi2, powi3
  (4,045 rows) and the snapshot index constant (M6).
- **Result, every target (M0):** 2 modules, 0 failing; **14,875 ok, 0 failed, 0 measured, 4,046
  pending, 1 accepted**.

Rule for impetus's own class-C functions (from M1): K_F = 2 · S · (class-C calls on F's path) + 2,
where S is F's measured sensitivity, set before F's first comparison. If K_F > 64, make the primitive
exact instead (tanmatra transcribed libm; kimiya made glibc's ill-conditioned calls correctly rounded,
its ADR 0005). Candidates to watch: `hypot` feeding `length − rest_length` in distance, spring and
rope joints (cancellation), and the spring's exp terms near critical damping.

## 3. Oracle provenance findings (M0 steps 5–8)

- **glibc for the transcendentals.** sin, cos (and `sin_cos`), atan2, hypot, exp and pow call glibc's
  libm; sqrt, abs, floor, ceil, min, max, clamp, the predicates, `to_radians` and the literal-exponent
  `powi` are inline. Every TSV header records `libm: glibc 2.44 x86_64 fma`; `--check` refuses another
  glibc or CPU class.
- **`sin_cos` equals the separate calls** on every one of its 2,084 rows (`validate.py`).
- **`powi` with a literal exponent** is a product: `x·x` and `(x·x)·x`. `validate.py` recomputes all
  1,220 rows.
- **hisab 1.4.0 links glam 0.29.3**, not the newer glam also in the lock.

## 4. Probes (M0 step 9, 2026-11-09, x86_64 Linux ELF)

Each probe is a program in `scripts/probes/`, run as shown in its header.

### 4.1 Memory on the step path (`p1_memory.cyr`)

The global `alloc` is a bump allocator with no free and **no redirection hook**. hisab calls it
directly (574 sites in the 3.4.1 bundle), and a library can't `alloc_reset` (it wipes the
consumer's heap too). So every hisab call that returns a value leaks for the life of the process:

| Call | Global heap per call, never returned |
|---|---|
| `hvec3_add`, `hvec3_cross`, `hquat_rotate_vec3` | 24 B |
| `m3_mul_vec3` | 48 B |
| `gjk_epa_3d`, two overlapping spheres | **105,920 B** (correct answer: depth 0.5) |
| `detect_islands`, 100 bodies / 50 contacts | 10,936 B |
| `bvh_build`, 100 boxes | 20,792 B |

A 3D scene with 100 touching pairs would leak about 10 MB per step through GJK/EPA alone, 600 MB a
second at 60 Hz. The allocators impetus can own are flat: **freelist** grew the global heap by 0 bytes
over 100,000 alloc/free pairs, and a **growable arena** reset every step held its capacity at 131,072
bytes from step 1 to step 1,000 (2,000 records per step).

**Decision** ([ADR 0007 amendment](../adr/0007-cyrius-port-hisab-first.md)): world storage on freelist,
step scratch on a world-owned arena reset per step, and **no hisab call on the step path or in a
query** until the allocation can be scoped (port plan §8: S1 in the stdlib, or hisab's H9). hisab is
used at setup (inertia at `add_collider`).

### 4.2 Vector representation in the solver loop (`p2_vectors.bcyr`, also probes 3 and 4)

One contact-solve kernel (relative velocity with two cross products, the normal impulse, two
velocity updates) over 1,000 contacts × 100 passes, four ways. All four end with **the same bits**
(digest `0xa08fe9563d5c0996`).

| Form | Per kernel | Heap |
|---|---|---|
| hisab handles, components read into `f64` locals, written back with the setters | **56 ns** | 0 |
| one `f64` stride per contact, each `s[k]` read where used | 63 ns | 0 |
| struct values `ImV3 { x: f64; y: f64; z: f64; }` with `+` / `−` overloads, in a typed stride array of nested structs | 154 ns | 0 |
| hisab handles, every operation returning a new `HVec3` | 176 ns | 216 B per kernel, leaked |

- **Decision:** hot loops load components into `f64` locals once, compute in scalars, and store back.
  Struct values are the readable form and cost 2.5× today: a compiler finding (port plan §8), not a
  reason to avoid them off the hot path.
- **Probe 3 (typed stride arrays):** `var c: *ImContact = base + i;` steps by `sizeof(ImContact)`,
  and nested struct fields (`c.va.x`) read and write in place. Assigning an untyped word (what
  `fl_alloc` returns) to a `*T` warns, so a one-line typed-return helper (`fn as_contacts(p):
  *ImContact { return p; }`) converts it. `--bounds` can't check a `*T` stride (a pointer has no
  length), so arena index and generation checks carry the bounds safety.
- **Probe 4 (struct values):** returns of 24 bytes come through the hidden pointer and are exact;
  overloads dispatch on struct-typed fields (`c.vb + v3_cross(..)`); a struct value assigns into a
  field (`c.va = c.va − ..`). `&c.va` is refused ("the address of a field is not taken"), so a
  field's bits are read as an `f64` local through an untyped-parameter helper (`fn bits(x): i64`).
  The small-struct `Result` payload bug is still open upstream: a `Result` carries a handle or a
  code, never a struct.

### 4.3 Sort stability (probe 5)

Three sorts in the oracle's library code:

| Site | Sort | Key | In the port |
|---|---|---|---|
| `contact_tracker.rs:85` | `sort_unstable` + `dedup` | the pair key, unique after dedup | any sort: the key is total |
| `backend_2d/solver.rs:734` | **stable** `sort_by_key` | min body level (shock propagation) | **not total**: equal levels keep manifold-key order |
| `backend_3d/solver.rs:125` | **stable** `sort_by_key` | min body level | **not total**: equal levels keep narrowphase order |

Sequential impulses depend on order, so the tie order is part of the result. `vec_sort_by` is
introsort (not stable), so the port sorts on **(level, index in the input order)**, which is total
and gives exactly the stable sort's order.

### 4.4 Per-profile tests (probe 6)

`cyrius help manifest`: `[lib]` / `[lib.*]` are read only by `cyrius distlib`; nothing prepends them
to a test compile. `[test] modules` is one list prepended to every test, bench and fuzz compile, so
it can't differ by profile. `#define` is capped at 16 names. **Decision:** `[test] modules` stays
empty; `tests/lib_2d.cyr` and `tests/lib_3d.cyr` include their profile's module list in `[lib]` /
`[lib.3d]` order, and every test file includes one of them. A check (M1, with surface parity)
fails when either drifts from `cyrius.cyml`.

## 5. M1a: the math helpers, config, material, units, force (2026-11-09)

- **The math helpers** (`src/mathx.cyr`) close the 4,045 pending `math` rows: `_im_min` / `_im_max` /
  `_im_clamp` (Rust's NaN-ignoring semantics, the second operand when equal), `_im_is_nan`,
  `_im_is_finite`, `_im_to_radians` (one multiply by the folded π/180), `_im_powi2` / `_im_powi3`,
  `_im_exp` (the polyfill), `_im_atan2` (ganita's on `_f64_atan_polyfill`), `_im_hypot`,
  `_im_sin_cos`. Every one is **0 ulp** on its class-A/B rows.
- **The value modules**, every row 0 ulp on x86_64 ELF, x86_64 PE and aarch64:

| Module | Rows | Result |
|---|---|---|
| `config` | 293 | `default` exact; `validate` 200 ok, **92 accepted (ADR 0008 #6)**: XPBD configs the oracle accepts and the port refuses |
| `material` | 2,046 | combine rules (min/max exact, mean/product 0 ulp), presets exact, `effective_static_friction` 0 ulp |
| `units` | 2,236 | constructors exact, `to_radians` 0 ulp ((value·π)/180, not `f64::to_radians`' value·(π/180)); `Quantity::fmt` 280 rows **pending** on S2 |
| `force` | 1,308 | constructors exact, magnitudes and `gravity` 0 ulp |

- `validate.py` recomputes 5,230 of these rows independently (every `validate` verdict, combine
  rule, `to_radians`, magnitude and gravity vector).
- **Result, every target:** 6 modules, 0 failing; **24,431 ok, 0 failed, 0 measured, 281 pending
  (S2's 280 and the M6 constant), 93 accepted** (ADR 0008 #6 92, ADR 0009 1). The dispatched math
  digests are unchanged and equal on all three targets.

## How to reproduce

```sh
cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml -- --check
python3 -I scripts/oracle/validate.py .
cyrius test tests/parity_2d.tcyr                  # x86_64 ELF
cyrius test --win tests/parity_2d.tcyr            # x86_64 PE, needs wine
cyrius test --aarch64 tests/parity_2d.tcyr        # aarch64, needs qemu-aarch64
cyrius run scripts/probes/p1_memory.cyr           # §4.1
cyrius bench scripts/probes/p2_vectors.bcyr       # §4.2
```

Compare the `digest` lines of the three runs: they must be equal for every dispatched function.
