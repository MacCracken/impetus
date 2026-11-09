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

## 6. M1b part 1: float text and serde-compatible JSON (2026-11-09, ADR 0010)

- **The shortest-digit printer** (`src/fmtf.cyr`: Burger & Dybvig on a 40-limb bignum) against both Rust
  printers, `fmtf.tsv`, 37,576 rows: every binade, every power of ten 1e-323..1e308, their neighbours, the
  2^53 edge, serde_json's −5/+15 thresholds, short decimals, 6,000 random bit patterns and 300 exact ties.
  **All exact** on x86_64 ELF, x86_64 PE and aarch64.
- **The two Rust printers disagree on exact ties.** When a double lies exactly halfway between its two
  closest shortest candidates, Rust's core (`Display`, `Debug`) rounds up and serde_json's zmij rounds to
  even: 2^-25 prints 2.9802322387695313e-8 and 2.9802322387695312e-8 respectively. `_im_shortest` takes
  the tie rule as a parameter. `validate.py` counts 751 such Display rows against Python's (half-even) repr,
  and every serde_json row equals it.
- **`Quantity::fmt`**: the 280 rows pending since M1a are ok.
- **JSON**, `json.tsv`, 545 rows. Every `to_string` document (248) is serde_json's byte for byte, and
  every `from_str` row (297) decodes to the oracle's fields or its error. The edge documents cover:
  - missing required fields (refused) and missing defaulted ones (default applied);
  - `null` for f64, `4.0`, `-1` and 2^32 for u32;
  - wrong-length arrays and unknown variant names;
  - unknown fields (ignored) and duplicate keys (refused);
  - integers for floats, exponent forms, underflow to 0, whitespace, non-objects, and empty and truncated
    documents.
- **serde_json's default float parse is not exact.** Without its `float_roundtrip` feature it is
  "best-effort": 1 ulp off on some long decimals, so it can't always read back what it wrote (0.013716891524587275
  read as …277). The port's parse is correctly rounded (bayan). impetus itself only implements
  `Deserialize`, and the parse precision is the consumer's serde_json choice, so the oracle uses
  `float_roundtrip`: every decode then agrees, and `validate.py` checks each decoded float against
  Python's correctly rounded parse.
- **Result, every target:** 8 modules, 0 failing; **62,832 ok, 0 failed, 0 measured, 1 pending (M6), 93
  accepted**; the math digests are unchanged.

## 7. M1b part 2: descriptors, arena, contact tracker, spring (2026-11-09)

- **The modules**: `src/types.cyr` (bodies, colliders, joints, events, queries; payload enums as a kind
  enum plus one struct), `src/arena.cyr` (the generational arena on freelist, handles `gen << 32 | idx`),
  `src/contact_tracker.cyr` (sorted pair lists, unsigned handle order, events once per frame) and
  `src/spring.cyr` (the exact damped oscillator, transition cached on (dt, k, c)).
- **Golden rows**, new or grown:
  - `types.tsv` (1): `BodyDesc::default`, exact.
  - `spring.tsv` (1,000). The constructors (153) and `is_settled` (47) are exact. The 800 `step` rows
    (the three regimes and the critical band × dt ∈ {1e-6, 1/60, 1, 100}, 200 random springs, the
    edges, and the Rust tests' scenarios) are **measured**, not gated; see below.
  - `json.tsv` (545 → 937): every M1b type both ways, plus serde's sequence form and one-key unit
    variants for every type (ADR 0010 findings). All exact, except 7 rows pending on S3 and S4 (bayan).
- **The spring's sensitivity sets no usable bound.** Each step row's note carries S, the most its
  outputs move per one-ulp change in an exp, sin or cos result (scripts/oracle/README.md):

| Regime | Rows | S ≤ 1 | 2–10 | 11–31 | 32–1,024 | > 1,024 | max S |
|---|---|---|---|---|---|---|---|
| under-damped | 377 | 58 | 68 | 27 | 129 | 95 | 6.1·10^9 |
| critical | 45 | 12 | 8 | 2 | 6 | 17 | 1.0·10^10 |
| over-damped | 366 | 83 | 32 | 16 | 100 | 135 | 4.5·10^18 |

  Two things drive it: `position = target + y` cancels when the spring is near zero and the target is
  far (2^26 at dt = 1e-6), and the over-damped branch divides `e1 − e2` by `r1 − r2` near the critical
  point. By the rule (§2) the bound K = 2·S·c + 2 is far past 64 on most rows, so the rule's answer is
  exact primitives (correctly rounded exp, sin, cos on the spring path), not a bound. **Done in §8 (ADR
  0011): the step rows are gated at 0 ulp.**
- **What the port measures meanwhile**, identical on all three targets: of the 1,600 step outputs,
  **1,531 are bit-identical**, 12 are 1 ulp off, 6 are 2 ulp, 12 are 3–16 ulp, and 39 are over 16 (max
  2.4·10^7, at S = 2.4·10^7). Every mismatch is where the polyfill exp or the builtin sin/cos is 1 ulp
  from glibc (§2: 7.5% and 2% of inputs) on a sensitive row. `validate.py` recomputes all 1,000 spring
  rows in Python on glibc, bit for bit, so the oracle rows and the transcription behind S are right.
- **Arena, tracker**: no golden rows (both are `pub(crate)`; M3's dump rows compare their state). The 10
  Rust arena tests and 3 tracker tests are ported, plus Cyrius-only tests: insert_at and the lazy free
  list (lowest index first), retain order, generation wrap at 2^32, LIFO reuse, records wider than a word;
  and, for the tracker, event order, unsorted and duplicate input, the seen-union across sub-steps,
  forget_colliders, and handles at generation ≥ 2^31 sorting after the rest (unsigned).
- **Result, every target** (x86_64 ELF, x86_64 PE, aarch64): 10 modules, 0 failing; **63,419 ok, 0
  failed, 800 measured (`Spring::step`), 7 pending (S3 3, S4 3, the M6 constant), 93 accepted**; the
  math digests are unchanged.

## 8. Correctly rounded exp, sin and cos; the spring gated (2026-11-09, ADR 0011)

- **glibc 2.44 is not correctly rounded** (MPFR 4.2.2 reference): it misrounds 0.105 % of random exp arguments in
  [−700, 700], 0.14 % of sin and 0.15–0.17 % of cos arguments (in [−10, 10] and [−1000, 1000] alike). On the spring
  grid: 2 of 377 sin calls, none of its 1,154 exp or 377 cos calls.
- **`src/crmath.cyr`** gives the correctly rounded value on every argument:
  - a table-driven phase 1 and a double-double phase 2, each with a proven rounding test;
  - a 364-bit fixed-point slow path, with Payne–Hanek reduction for sin and cos;
  - its tables generated from π and ln 2, which are computed exactly and equal MPFR's to 2,048 bits.

  Verified beyond the corpus (scratch probes against MPFR):

| Set | Arguments | Equal to MPFR | Slow calls |
|---|---|---|---|
| whole range: exp over [−745.2, 709.8] and every binade from 2^-55 up; sin and cos to 1.8·10^308; the edges | 180,062 | all, on the slow path alone and through the full functions, on x86_64 and aarch64 | 39,100 (the sin/cos arguments above 2^20, by design) |
| the fast paths' ranges | 90,000 | all | 0 (phase 2: 246) |
| arguments where glibc misrounds (each within about 0.01 ulp of a rounding boundary) | 1,200 | all | 222 (those above 2^20) |

- **Three phases since the speed-up** (ADR 0011):
  - Phase 1 is table-driven, in double with exact products, with bound 2^-62.
  - Its measured worst error over 200,000 arguments each, against phase 2 (2^-100), is 2^-67 for exp, 2^-66 for
    sin and 2^-67 for cos. That is 4–5 bits inside the bound.
  - It decides 89,754 of the 90,000 fast-range results. Phase 2 takes the other 246.
  - Results are unchanged: every probe set and the corpus give the same bits, and the digests below are those
    from before the speed-up.
- **The corrected oracle**:
  - `tests/golden/math.cr.tsv` has 9 rows: exp, sin, cos and sin_cos where glibc misrounds; 4 of them (cos and
    sin_cos at 2 arguments) are new threshold rows;
  - `tests/golden/spring.cr.tsv` has 1 row, where glibc's sin(0.1531…) misrounds and the step's outputs move by 2
    and 17 ulp.

  The harness compares those rows against the corrected outputs: all 10 equal them, and none equals glibc's.
- **Bounds:** 0 for `f64::exp`, `f64::sin`, `f64::cos`, `f64::sin_cos` and `Spring::step`. Every one of their rows
  is 0 ulp on x86_64 ELF, x86_64 PE and aarch64:

| Function | Rows | Result |
|---|---|---|
| `Spring::step` | 800 | 799 ok, 1 accepted (ADR 11); 1,600 outputs at 0 ulp |
| `f64::sin_cos` | 2,156 | 2,152 ok, 4 accepted (ADR 11) |
| `f64::sin` / `f64::cos` | 2,156 each | 2,154 ok, 2 accepted (ADR 11) each |
| `f64::exp` | 1,555 | 1,554 ok, 1 accepted (ADR 11) |

- The math grid gains 246 rows (18,919 → 19,165): 72 angles for sin_cos, sin and cos, and 30 exp arguments. They
  straddle every threshold of `crmath.cyr`, and include the closest double to a multiple of π/2. The polyfills are now shadow lines: exp 1 ulp on 118 rows, sin on 47, cos on 46.
- **Digests**, equal on all three targets (ELF and PE identical in full; aarch64 also, apart from the native-exp
  shadow, which is x87 on x86_64):

| Function | Digest |
|---|---|
| exp | `0x9763e0c4a6dc5471` |
| sin | `0xe3107150ef837dbc` |
| cos | `0x79eede887b756dc1` |
| sin_cos | `0xa38fff9ee8badd98` |
| atan2, hypot, powf | unchanged |

- **Result, every target:** 10 modules, 0 failing; **64,455 ok, 0 failed, 0 measured, 7 pending (S3 3, S4 3, the
  M6 constant), 103 accepted (ADR 0008 #6 92, ADR 0009 1, ADR 0011 10)**.

## 9. M2a: the broadphase (2026-11-09)

- **The modules:**
  - `src/aabb_tree.cyr`, the dynamic AABB tree: SAH insertion, Box2D's AVL balance, LIFO node reuse,
    0.1 fattening;
  - `src/spatial_hash.cyr`, the spatial hash: cell rasterisation, and the oversize list past 4,096 cells or
    for non-finite or out-of-range boxes.

  Both keep their storage across rebuilds (`clear` / `reset`), and both order keys as unsigned words.
- **The grid's cells are sorted records, not a map.** Rust's BTreeMap of cells becomes a list of (cell, key)
  records sorted by an injective two-word packing of the cell. The pair set is a set, so the order cells are
  visited in can't show.
- **`broadphase.tsv`**, 76 rows through the oracle's pair hooks; the 2D build emits the 3D rows too, since the
  structures are shared:
  - `aabb_tree_pairs_2d` (25 rows) and `_3d` (8): the Rust tests' layouts, the fattening edge (boxes 0.2 apart,
    ± 1 ulp), NaN, infinite, inverted, empty and huge boxes, random sets of 5 to 1,000, and sorted rows of 1,000;
  - `spatial_hash_pairs_2d` (24) and `_3d` (6):
    - cell sizes 0, negative, 1e-11, 1e-10, NaN and ±∞;
    - the oversize cut (64×64 against 64×65 cells; 16×16×16 against 16×16×17);
    - exact cell boundaries and the edges of the i32 cell range;
    - random sets, and 1,000 boxes at the automatic cell size;
  - `spatial_hash_auto_cell_size` (13, class B).

  **All exact** on x86_64 ELF, x86_64 PE and aarch64. `validate.py` recomputes every row independently (the
  tree's pairs by brute force on the fattened boxes, the hash's by its own rasterisation), and they agree.
- **The harness** now takes 8,192 tokens per list (kimiya's core took 1,024): a 1,000-box row carries 4,001
  input tokens.
- **Result, every target:** 11 modules, 0 failing; **64,531 ok, 0 failed, 0 measured, 7 pending, 103 accepted**.

## How to reproduce

```sh
cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml -- --check
python3 -I scripts/oracle/validate.py .           # also checks src/crmath_data.cyr and tests/golden/*.cr.tsv
python3 -I scripts/oracle/validate.py . --write   # regenerates them (needs libmpfr.so.6)
cyrius test tests/parity_2d.tcyr                  # x86_64 ELF
cyrius test --win tests/parity_2d.tcyr            # x86_64 PE, needs wine
cyrius test --aarch64 tests/parity_2d.tcyr        # aarch64, needs qemu-aarch64
cyrius run scripts/probes/p1_memory.cyr           # §4.1
cyrius bench scripts/probes/p2_vectors.bcyr       # §4.2
```

Compare the `digest` lines of the three runs: they must be equal for every dispatched function.
