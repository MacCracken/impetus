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

## 10. M2b: the 2D backend's types (2026-11-09)

- **The module:** `src/b2_types.cyr`, every expression in Rust's operation order:
  - `RigidBody2d`: `from_desc`, `refresh_inverse_mass`, `solver_inv_mass`, both integrations, `clear_forces`;
  - `Collider2d`: `world_aabb`, `shape_props`, `polygon_mass_props`, `mass_properties`;
  - `Aabb2d`, and the `Joint2d`, `Contact`, `ManifoldPoint` and `ContactManifold` records M3 fills;
  - `IslandManager`, whose `union` is `unite` (`union` is a reserved word).

  `ColliderShape` gained `clone`: a collider keeps its own copy of the shape's arrays, as Rust's does.
- **Private constants are generated.** `scripts/oracle/src/t_private.rs` reads every `const` in the library part
  of `aabb_tree.rs`, `spatial_hash.rs` and `backend_2d/types.rs`. It writes `src/constants_private.cyr`: 11 values,
  named `_IM_*` and `_IM_B2_*`, each literal parsed by Rust's f64 parser and written positionally.
  - Every const is emitted or excused. Two are excused: `NULL_NODE` (the port's null node is −1) and `FALLBACK`
    (a tuple local to `shape_props`).
  - `validate.py` checks each value against Python's own parse of the literal.
  - The broadphase's 0.1 margin and 4,096-cell cap now come from this file.
- **`b2_mass.tsv`**, 75 rows from world dumps. Each row builds a `PhysicsWorld` with one body, adds its colliders
  through the public `add_collider`, and reads mass, inertia and the inverses from `oracle_dump()`. The grid:
  - every shape kind at zero, negative, NaN, tiny and huge sizes;
  - hulls in both windings; degenerate hulls (collinear points, two points, none); seeded polygons of 3–16
    vertices. Their vertices are rational points on an ellipse, so the grid uses no transcendental;
  - triangle meshes, including one with an out-of-range index and one with no triangles;
  - offsets (the parallel-axis shift); densities 0, 0.5 and 10⁶;
  - explicit masses 42, 0, −5, NaN and 10⁻⁹ (the `MIN_MASS` floor);
  - static, kinematic, fixed-rotation and collider-less bodies;
  - 13 bodies with several colliders (one fixed set, 12 seeded).

  Class B, and **all 0 ulp** on x86_64 ELF, x86_64 PE and aarch64. `validate.py` recomputes all 75 with its own
  `shape_props`, `polygon_mass_props`, accumulation and `refresh_inverse_mass`, and they agree.
- **Not golden yet:**
  - `world_aabb` is `pub(crate)`, so no hook reaches it. The ported tests cover it now; the M2 `overlap_aabb` rows
    and M3's step rows go through it.
  - The integrations and the islands arrive with M3's step rows.
- **Result, every target:** 12 modules, 0 failing; **64,606 ok, 0 failed, 0 measured, 7 pending, 103 accepted**.

## 11. M2c: the 2D narrowphase (2026-11-09)

- **The module:** `src/b2_narrowphase.cyr`, every expression in Rust's operation order. It covers:
  - the round tests (circle–circle, circle–box, capsule–circle and capsule–capsule, segment–circle,
    hull–circle with the centre-inside case);
  - Box2D's reference/incident-face clipping for boxes, hulls and segments, with the SAT fallback;
  - capsule end-cap contacts against boxes, hulls and segments;
  - `generate_contacts_multi` and `shapes_touch`.

  A contact is a five-word record in a caller's buffer of room for two, and functions return the count. Rust's
  `capsule_end_contacts` closure is a box-or-hull switch. Polygon vertices live on the stack up to 16 per polygon,
  and on freelist beyond that.
- **One `sin_cos` per polygon and per contact.** Rust's box `polygon_vertices` calls `local_to_world` per vertex,
  and `box_to_world` rotates and translates separately. That is four and two `sin_cos` calls of one angle. The port
  takes one, which gives the same bits because `sin_cos` is pure. The hull path already does this, as Rust's
  `transform_hull` does.
- **`b2_narrow.tsv`**, 17,854 rows through the oracle's `contacts_2d` and `shapes_touch_2d`:
  - `b2::contacts` (class C, bound 0, 16,133 rows): every ordered pair of a 12-shape catalog at five rotation
    pairs (0, π/4, π/2, and 0.3 against −1.1), along three directions at six distances from coincident to apart.
    The catalog: ball, box, capsule, hulls in both windings, a triangle, a segment, a two-point hull, and
    degenerate ones (a zero ball, a flat box, a zero-length capsule, a collinear hull).
  - Exactly touching, 10⁻⁹ apart and 10⁻⁹ overlapping poses; the Rust tests' poses; NaN and infinite poses; hulls
    of 24 and 40 vertices; heightfields and meshes, which generate nothing; and 2,400 seeded pairs.
  - `b2::shapes_touch` (class A, 1,721 rows).

  Every box's half extents are finite and non-negative. A round shape against a box reaches Rust's
  `f64::clamp(-h, h)`, which panics on an inverted or NaN range, so the port's behaviour there is never compared.
- **The corrected oracle.** `validate.py` transcribes the narrowphase in Python. On glibc's `sin` and `cos` it
  reproduces all 17,854 rows bit for bit. On MPFR's it writes `b2_narrow.cr.tsv`: 3 rows where glibc misrounds a
  `sin`, and the port equals the corrected value on all 3. The transcription folds with explicit loops, because
  Python 3.12's `sum()` of floats is compensated and Rust's is not.
- **A port defect the rows caught.** Multi-return values are untyped words, so `-ax` on one negated the integer
  bits, not the float, and the compiler gave no f64 diagnostic. That produced a non-unit SAT normal on 2 rows.
  The fix binds the value to an `f64` local first, as the f64-hygiene rule asks. No other multi-return site applies
  an operator to an untyped value.
- **Result, every target:** 13 modules, 0 failing; **82,457 ok, 0 failed, 0 measured, 7 pending, 106 accepted**.
  Every narrowphase row is 0 ulp on x86_64 ELF, x86_64 PE and aarch64, and the digests are unchanged.

## 12. M2d: correctly rounded hypot, and the 2D raycast and overlap queries (2026-11-09)

- **hypot is correctly rounded** ([ADR 0011, hypot amendment](../adr/0011-correctly-rounded-exp-sin-cos.md)).
  - The rays normalise their direction and normals with `f64::hypot`, and M3's joints call it six times. The
    port's `hypot` was ganita's, which differs from glibc on 414 of the 1,715 golden rows. glibc's own misrounds
    0.31 % of 300,000 seeded arguments against MPFR.
  - `src/crmath.cyr` now has `_im_cr_hypot`. Phase 1 works in double-double from exact products after an exact
    power-of-two scaling, with a 2^-96 rounding test. The slow path is exact: x² + y² in fixed point, compared
    with the squares of the midpoints. `hypot` has exact midpoints (Pythagorean triples with a 54-bit odd
    hypotenuse), and they round to even.
  - It matches MPFR on 404,234 probe pairs, including 4,134 exact midpoints, subnormal results, overflow and the
    specials. x86_64 and aarch64 give byte-identical output.
  - In the corpus, `f64::hypot` is gated at 0 ulp: 1,703 rows equal glibc, and 12 equal `math.cr.tsv`. ganita's
    `hypot` is a shadow line, 1 to 2 ulp off on 408 rows.
- **The module:** `src/b2_raycast.cyr`. It covers:
  - the ray tests: circle, the oriented box in its own frame, capsule (two discs and the shaft rectangle,
    unioned in Rust's order), segment, and convex hull by Cyrus–Beck clipping;
  - `RayInterval` with `first` and its union;
  - the per-collider parts of the world's queries, `_im_b2_raycast_collider` and `_im_b2_overlap_collider`.

  `PhysicsState2d::raycast`, `raycast_filtered`, `overlap_sphere` and `overlap_aabb` loop them over the colliders.
  They arrive with ImPhysicsState2d in M3, and surface parity lists them as pending. A rotation's `sin_cos` is
  taken once where Rust takes it per vertex.
- **No hisab.** The plan had pointed the round shapes at hisab's `geo_ray_*`. Queries stay off hisab, as the step
  does, until allocation can be scoped (ADR 0007 amendment, S1/H9). The port's rays also rest on correctly
  rounded `sin_cos` and `hypot`, which hisab's don't.
- **`b2_ray.tsv`**, 14,754 rows through the public world API. Each row is a world of static bodies with one
  collider each, then one query:
  - `b2::raycast` (class C, bound 0, 7,290 rows, 2,883 of them hits). Every catalog shape, including fields,
    meshes and two-point hulls, which take no ray. Four rotations and two offsets. Rays from eight directions
    at six lateral offsets, and from inside. Directions that are zero, below EPSILON, NaN, infinite or not unit.
    Max-distance cuts. Layer masks. Several colliders, where the nearest wins, the first of equals, and the cut
    applies before the choice. The Rust tests' scenes, big hulls, and 1,800 seeded worlds.
  - `b2::overlap_sphere` and `b2::overlap_aabb` (class A, 4,772 and 2,692 rows). They also pin `world_aabb`, the
    pre-filter, which accepts fields and meshes on their boxes alone. They include exactly touching boxes (closed
    intervals) and an inverted probe.
- **The corrected oracle.** `validate.py`'s transcription adds the ray tests, `world_aabb` and the overlap loop,
  with `hypot` as a parameter: glibc's own through ctypes (CPython's `math.hypot` is a different implementation),
  or MPFR's. On glibc it reproduces all 14,754 rows bit for bit. `b2_ray.cr.tsv` holds the 39 raycast rows where
  glibc misrounds a call, 38 through `hypot` and 1 through `sin`, and the port equals all 39.
- **Result, every target:** 14 modules, 0 failing; **97,160 ok, 0 failed, 0 measured, 7 pending, 157 accepted**
  (ADR 0011: 64). Every query row is 0 ulp or exact. The `hypot` digest is identical on x86_64 ELF, x86_64 PE and
  aarch64.

## 13. M3a: the 2D backend's state (2026-11-09)

- **The module:** `src/b2_state.cyr`, ImPhysicsState2d:
  - the bodies, colliders and joints in M1b's generational arenas, so its handles are Rust's, handle for handle;
  - each body's collider list;
  - the manifold store, sorted by collider pair as unsigned words (Rust's BTreeMap order), with removal by
    collider (the solver fills it in M3's later parts);
  - the contact tracker and the island union-find;
  - every method of `state.rs`: add and remove bodies, colliders and joints; `recompute_mass`; the forces,
    impulses and torques; waking neighbours; the snapshot inserts (`add_*_at`); `get_body_state`,
    `set_body_state` and `set_body_type`;
  - `raycast`, `raycast_filtered`, `overlap_sphere` and `overlap_aabb`, pending since M2d, looping M2's
    per-collider queries.
- **`body_colliders` beside the slots.** Rust's `add_collider` creates a map entry even for a missing body. The
  port refuses that orphan (ADR 0008 #1), so the map holds exactly the live bodies, and the port keeps each list
  beside its body's arena slot. The dump sorts them by handle, as the BTreeMap iterates.
- **`b2_state.tsv`**, 411 rows driven through the public world API. Each row is a script of ops ending in a full
  `oracle_dump()` of the backend (`scripts/oracle/src/dump2d.rs`): the three arenas' slots, generations and free
  lists; every field of every body, collider and joint; `body_colliders`; the manifolds; the tracker.
  - 8 hand scripts: slot reuse and LIFO free lists; mass that adds up, recomputes on removal, and follows
    `set_body_type`; forces with points and impulses on bodies without mass; reference angles from rotations;
    removals that wake and cascade; teleports; errors for missing handles.
  - 400 seeded scripts of 5 to 30 ops over live and stale handles. Every op runs hundreds of times, and 530
    results are errors.
  - 3 orphan scripts (`b2::state_script_orphan`), mapped under ADR 0008 #1 and counted.

  Class B, and **all 0 ulp** (53,812 floats) on three targets.
- **The model.** `validate.py` replays every script on a Python model of Rust's arenas (free list, generations,
  the stale rebuild) and the state, using the b2_mass helpers for the mass sums. It rebuilds every result and dump
  token, orphans included, and a tampered float fails it.
- **The queries move onto the state.** The 14,754 `b2_ray` rows now build an ImPhysicsState2d and call its
  methods, and they still all pass. So do the 9 world-level raycast tests, which no longer need a test-local world.
- **Result, every target:** 15 modules, 0 failing; **97,568 ok, 0 failed, 0 measured, 7 pending, 160 accepted**
  (ADR 0008 #6 92, #1 3; ADR 0009 1; ADR 0011 64).

## 14. M3b: joints, integration, islands and sleep; the first single-step rows (2026-11-09)

- **The modules:**
  - `src/b2_joints.cyr`:
    - the velocity-level joint solver, with Rust's `Row`, `PointBlock`, `BodyRef`, `Vel` and `PreparedJoint` as
      records;
    - `prepare_joints` for all eight joint types, with limits, motors, damping and the soft constraints;
    - `solve_joints_once` and `break_joints`.

    The prepared joints and their rows (at most 6 a joint) live in the state's scratch, reused from step to step,
    so the heap stays flat. `JOINT_BAUMGARTE` is generated with the other private constants.
  - `src/b2_solver.cyr`, the step stages that run when nothing touches:
    - velocity and position integration over every body;
    - `build_islands_and_sleep`, whose islands are grouped by root as Rust's BTreeMap orders them;
    - `finish_step`.

    The contact stages and `step` come with M3c.
- **Single-step rows (class S, the plan's §5).** A `b2::step` row is the world config and the backend dump at frame k,
  taken after that frame's forces; its outputs are the dump at k + 1 and the frame's events. The harness loads the dump
  into an ImPhysicsState2d (every arena slot, generation and free list as dumped), steps it, and compares the whole
  state. Every frame is therefore checked on its own, and errors can't compound.
- **Why these scenes pair nothing.** The broadphase drops a pair when neither collider's layer meets the other's mask.
  Every M3b collider has mask 0, so the contact stages are provably empty, and the colliders are there for mass. A body
  with no collider has no mass and would never move. Until M3c the harness composes the contact-free sub-step from the
  library's stages.
- **`b2_step.tsv`**, 214 rows. Scenes at frames 0, 1, 2, 10, 60 and 300:
  - a pendulum at 1, 4 and 8 sub-steps, and a 20-link revolute chain;
  - a fixed pair pinned to the world; a revolute with limits, a motor and damping; a prismatic slider with limits and
    a motor; a wheel on a suspension;
  - a spring, a damped distance joint and a rope that goes from slack to taut; a mouse drag;
  - joints that break and that hold; a kinematic driver; a push every frame;
  - a pair at rest that falls asleep (frames 25 to 31);
  - 40 seeded scenes of random joints at random frames.

  Class C (`sin_cos` in the rotations, `hypot` in normalisation, lengths and impulse caps). **All 0 ulp** (30,078
  floats) on three targets.
- **The corrected oracle.** `validate.py` transcribes this part of the step. It parses each dump, steps it (sub-steps,
  integration, all eight joints, islands, sleep, `finish_step`) and writes the dump again. On glibc's `sin`, `cos` and
  `hypot` it reproduces all 214 rows bit for bit, and it asserts that every collider's mask is 0. On MPFR's it writes
  `b2_step.cr.tsv`: 4 rows (3 through `hypot`, 1 through `sin`/`cos`), and the port equals all 4.
- **Result, every target:** 16 modules, 0 failing; **97,778 ok, 0 failed, 0 measured, 7 pending, 164 accepted**
  (ADR 0011: 68).

## 15. M3c: the contact pipeline and `step` (2026-11-09)

- **The modules:**
  - `src/b2_contacts.cyr`, the contact stages of `solver.rs`, each keeping Rust's operation order:
    - the broadphase, both kinds, with each collider's AABB swept by its body's velocity, then the pair filters;
    - the narrowphase and the manifold rebuild, with warm-start matching in either body's frame;
    - waking on contact and warm starting;
    - the contact solver: shock-propagation order, block-solved two-point normals, Coulomb friction with a static
      threshold, rolling resistance and restitution targets;
    - the split-impulse position solver.
  - `src/b2_solver.cyr` gains `step`: Rust's `step_si`, then the touching pairs recorded. XPBD is not ported
    (ADR 0008 #6), so `step` runs the sequential-impulse solver for every solver kind.
- **Three departures that change no bit:**
  - **The narrowphase and the manifold rebuild are fused.** The broadphase pairs arrive sorted, and each is its own
    manifold key, so the fresh manifolds come out in key order.
  - **Lever arms are computed once a sub-step.** Positions don't change from the rebuild to the position solver, so
    each point's lever arms come from one `sin_cos` per manifold, and Rust's repeated expression gives the same bits.
  - **Warm-start impulses are applied as they are computed.** Computing them reads no velocity, so the order and the
    amounts are Rust's.
- **The harness now runs the library.** `b2::step` rows run `step` once per sub-step, then `finish_step`, for both
  modules. The 214 contact-free rows of §14 pass unchanged through it.
- **`b2_contact.tsv`**, 418 rows. Every collider can pair, and every scene runs on the default config unless noted:
  - drops (a ball, a box, a tilted box), each under both broadphases;
  - a five-box stack and a six-box pyramid;
  - restitution under four combine rules; friction on a slope (wood, ice, a static-friction override); a shove along
    flat ground; rolling resistance at three strengths;
  - capsules, hulls, a segment floor and an offset collider;
  - a sensor; layer masks; a kinematic paddle; sleep, then wake on contact;
  - five position-solver settings (Baumgarte, soft constraints, slop, iterations, sub-steps);
  - fast bodies through the swept AABB; a revolute chain draped over the ground;
  - 40 seeded scenes of random shapes, materials and velocities between walls.

  Class C. **All 0 ulp** (83,695 floats) on three targets.
- **The corrected oracle.** The transcription in `validate.py` now covers the whole sequential-impulse step: contacts,
  the contact tracker and events. It reproduces all 632 step rows (b2_step and b2_contact) bit for bit on glibc.
  - Its broadphase is brute force over every pair. Both of Rust's broadphases return a superset of the overlapping
    pairs: the tree's boxes are fattened, cells are monotone in the coordinate, and a non-finite box pairs with
    everything. The overlap test then keeps exactly the same set.
  - On MPFR it writes `b2_contact.cr.tsv`, 2 rows, each a `cos` that glibc misrounds by one ulp. The port equals both.
- **The harness reports where a row fails.** A failing row now names its first differing output and prints the pair
  there, instead of the row's first output.
- **Result, every target:** 17 modules, 0 failing; **98,194 ok, 0 failed, 0 measured, 7 pending, 166 accepted**
  (ADR 0011: 70).

## 16. M3d: correctly rounded pow, the world and its particles (2026-11-09)

- **pow first** (ADR 0011, amended for pow). The radial force field is the port's one `powf`, and ganita's misrounds
  9.62 % of ordinary arguments. So crmath gained `_im_cr_pow`, correctly rounded everywhere.
  - Two double-double phases: log x by the atanh series, relative near 1 (measured worst 2^-73.2 in phase 1 and
    2^-103.4 in phase 2), then exp.
  - A slow path with an exactness test for the dyadic results. 25^11.5 = 5^23 is a 54-bit tie, and glibc 2.44
    misrounds the subnormal ties of §8. Otherwise the 364-bit fixed point.
  - It matches MPFR on 1,290,769 pairs on the final code.
  - The 1,030 `f64::powf` rows are now gated at 0 ulp, the ADR 0009 row among them. glibc is correctly rounded on all
    1,030, so there is no correction file. ADR 0009 is superseded.
- **The modules:**
  - `src/particle.cyr`: `ImParticle`, `ImParticleEmitter`, `ImSubEmitter` and the payload form of `ForceField`, with
    Rust's builders, which chain on a pointer.
  - `src/world.cyr`: `ImPhysicsWorld` and its 2D step. That is the frame counter, the backend's sub-steps and events,
    then the particles: emitter spawns with the R3 spread and the 10,000 cap, gravity, fields, drag, damping,
    collisions with balls, boxes, capsules and segments, sub-emitters, and removal of the dead.
  - The API follows Rust's with ADR 0008's changes (#1, #6). Slices come back as (address, count). Snapshots wait for
    M6. Particle JSON came last (below).
  - `FIELD_SOFTENING`, `MAX_SPAWNS_PER_EMITTER_STEP` and the `R3_ALPHA` array are generated (`t_private.rs` now reads
    multi-line array constants).
- **`world_step.tsv`**, 198 rows: the config, the world's own state (counters, particles, emitters with their spawn
  accumulator, fields) and the backend dump in; the same after `step()`, with the frame's events, out.
  - The generator reads the emitter's private `accumulator` through `serde_json`, which carries it exactly, so
    `rust-old/` is untouched.
  - The scenes:
    - emitters: valid, inactive, NaN rate, negative rate, sub-stepped, several spawns a frame;
    - radial fields at falloffs 0, 0.7, 1, 1.5 and 2, with a reach limit, a particle at the centre, and repulsion;
    - a directional wind box;
    - drag, one case of which would reverse the motion, and damping;
    - collisions with every shape kind: a sensor, a hull that collides with nothing, a particle starting inside a
      box, a zero radius, three restitutions;
    - a kinematic paddle and a dynamic box;
    - sub-emitters bursting into 1 to 13 children;
    - particles under a field beside contact events;
    - 30 seeded scenes.

  Class C. **All 0 ulp** (68,166 floats) on three targets, on the first run.
- **The transcription.** `validate.py` now transcribes the world step: the backend frame, then the particles on
  glibc's sin, cos and pow. It reproduces all 198 rows bit for bit. MPFR changes none of them, so there is no
  `world_step.cr.tsv`.
- **Rust tests.** All 22 of `world.rs` are ported (its four XPBD tests excepted), all 8 of `particle.rs`, 9 of
  `tests/integration.rs`, and 26 of `tests/regressions.rs` (the 2D ones; the three
  snapshot tests wait for M6). That makes `tests/world.tcyr` (50 assertions, with Cyrius-only checks of `try_new`, the
  spawn cap, emitter removal and a flat heap) and `tests/regressions.tcyr` (44).
- **Particle JSON**, the last piece of M3. `src/json.cyr` writes and reads `ForceField` (externally tagged, both
  variants with fields), `SubEmitter`, `Particle` (`on_death_emit` defaults to None) and `ParticleEmitter`, its private
  `accumulator` required, as serde derives it.
  - `json.tsv` grows 937 → 1,092 rows: 70 written values and 85 documents read back. The documents include the
    sequence form at every length, a bare variant name, two keys, a missing or extra field, a u32 out of range or
    written as a float, and a `null` for an f64.
  - All 155 pass on the first run, writes byte for byte. The two particle serde tests are ported (`world.tcyr`, 55
    assertions).
- **Result, every target:** 18 modules, 0 failing; **98,548 ok, 0 failed, 0 measured, 7 pending, 165 accepted**
  (ADR 0011: 70). Before particle JSON it was 98,393 ok. The ELF and PE digests are equal (their `powf` digest is new). On aarch64 only the native shadows
  differ.

## 17. M4a: glam's 3D math and the 3D backend's types (2026-11-09, ADR 0012)

- **The decision** (ADR 0012). The 3D step path can't call hisab until its allocations can be scoped (§4.1). The
  maintainer chose the stopgap over waiting: impetus transcribes glam's math and ports its own GJK/EPA, gated as the
  2D backend is (classes A–C, 0 ulp against the corrected oracle). H9 is filed in hisab
  (`hisab/docs/development/issues/2026-11-09-allocator-scoped-calls.md`). Its repro exits 1 while 7 of 7 calls leak,
  the same numbers as §4.1.
- **`src/math3.cyr`.** These are glam 0.29.3's `DVec3`, `DQuat` and `DMat3` operations as the 3D backend calls them,
  read from the registry source of the version the oracle links. hisab 1.4.0 enables only `std` and `serde`, so all
  of it is scalar f64 code. Every formula keeps glam's order: the cross product's operand order, `mul_vec3`'s chained
  adds, the inverse by adjugate then transpose, `DVec4`'s dot for a quaternion's length. `+` and `−` are overloaded.
  There is no `*` overload: glam's `*` means four different operations by operand type, and Cyrius dispatches on the
  left operand only.
- **`b3_math.tsv`**, 7,087 rows from the `b3` build. It drives glam through hisab 1.4.0's re-export, which is now an
  optional dependency of that build and was already in its lock.
  - The inputs are edge vectors (±0, axes, subnormal, overflowing squares, ±∞, NaN), seeded ones, unit vectors for
    `any_orthonormal_pair`, angles from 0 to 1e300, unit and non-unit quaternions, and dense, rotation, diagonal,
    singular and non-finite matrices.
  - Class A for one IEEE operation per output, B for compositions, and C for the three rotations built on `sin_cos`.
  - **All 0 ulp on three targets** with one digest (`0xe3ab711457c42b4d`). `validate.py` recomputes every row from
    glam's formulas. MPFR corrects none, so there is no `b3_math.cr.tsv`.
- **`f64::signum`** (glam's `any_orthonormal_pair`): `_im_signum` returns Rust's own positive NaN for a NaN of
  either sign. That adds 326 `math.tsv` rows (19,491), all exact.
- **`src/b3_types.cyr`**, a transcription of `backend_3d/types.rs` on math3:
  - bodies, with `from_desc`'s z rotation, R·I⁻¹·Rᵀ, both integrations, and the gyroscopic step (one Newton step of
    backward Euler, projected onto the pre-step energy);
  - colliders, with world AABBs for every shape, mass and inertia (capsule as cylinder plus hemispheres, hull and
    mesh by their points' AABB);
  - the parallel-axis shift.

  `recompute_mass_properties` is three steps (reset, add each collider, finish) for M5's state to loop. The six
  private constants are generated as `_IM_B3_*`.
- **`b3_body.tsv`**, 234 rows. Each builds a 3D world, adds a body from a full descriptor and its colliders through
  the public API, and reads the dump: 60 outputs, every stored field but the handle. The rows cover:
  - every shape alone, including a zero radius, a flat box, a flat hull (the 1 m³ fallback), one point and none;
  - densities from 0 to 1,000 and explicit masses (0, −1, 1e12);
  - offsets, several colliders, and a symmetric set;
  - static and kinematic bodies, fixed rotation, rotations from −0 to 10;
  - 150 seeded bodies.

  Class C. All 0 ulp on three targets, and `validate.py` recomputes all of them.
- **A compiler defect, found by these rows.** On the first run 202 of the 234 bodies failed, each with its inverse
  inertia equal to its inertia.
  - cyrius 6.7.18 compiles `s.f = g(.., t.e, ..)` to store `t.e` itself when `g` returns a struct and `t.e` is a
    struct-typed field, at any depth of the arguments. Every struct size is affected.
  - A local destination, `var t: T = g(..)`, an f64-field argument, a method on a field and the operator form are
    all correct.
  - Workaround: bind the call to a local, then store it (7 sites in `b3_types.cyr`).
  - `scripts/surface_parity.py` resolves argument types and fails on any new site; the 2D code has none.
  - Filed as `cyrius/docs/development/issues/2026-11-09-field-store-of-struct-call-copies-a-field-argument.md`, with
    a repro that exits 1 on 7 wrong forms.
- **Performance, within the work loop.** Each helper call copies its by-value struct arguments, so `ImQuat_mul_vec3`
  as a chain of nine helpers cost 128 ns. The composite operations now read components into f64 locals in glam's
  order:
  - rotation 41 ns, against hisab's allocating `hquat_rotate_vec3` at 51 ns;
  - 3×3 product 86 ns, against `m3_mul` at 89 ns;
  - inverse 50 ns.

  The digest and every row are unchanged. `docs/benchmarks-rust-v-cyrius.md` has the rows.
- **Rust tests.** `tests/b3_types.tcyr` (23 assertions) ports the type-level tests of `backend_3d/tests.rs`:
  - quaternion identity and z rotation, the sphere, box and hull masses, the capsule tensor;
  - accumulation over colliders, the parallel-axis theorem with its off-diagonal term, and removal as a recompute
    over none;
  - gyroscopic precession, and its absence for a symmetric body.

  It adds a free spin integrated for 600 steps (energy never grows, |L| holds; the Rust test steps the state, M5),
  the AABB rule, and a flat heap (global and freelist) across the math, integration, AABBs and mass.
- **The 3D profile.** `[lib.3d]`: the shared modules, `math3`, `b3_types`, then `particle` and `json`, which touch no
  backend. `distlib` writes `dist/impetus-3d.cyr`. `tests/lib_3d.cyr` mirrors the list, and `surface_parity.py` now
  fails if either mirror drifts. The descriptor token helpers moved to `tests/parity_types.cyr`, shared by both
  harnesses.
- **Still unpinned by golden rows:** `integrate_velocities`, `integrate_positions` and the gyroscopic step (M5's
  single steps), and `world_aabb` (M5's broadphase: Rust 1.4.0's 3D world has no overlap queries, so M4 part 3 has
  none to pin it with). The reference tests above exercise them.
- **Result, every target:** 3D **7,321 ok, 0 failed** (2 modules); 2D **98,874 ok, 0 failed, 0 measured, 7 pending,
  165 accepted** (18 modules). The digests are equal on ELF, PE and aarch64, except the 2D native shadows, as before.

## 18. M4b: the 3D narrowphase (2026-11-09, ADR 0012)

- **`src/b3_narrowphase.cyr`** transcribes `backend_3d/narrowphase.rs`:
  - `generate_contact_3d`'s dispatch, and the closed forms (ball–ball, ball–box, capsule–ball, capsule–capsule,
    capsule–box with its five candidate points, segment–ball, segment–box with its nine samples);
  - the AABB fast path, and the OBB SAT over 15 axes with reference-face clipping and edge-pair points;
  - impetus's own GJK + EPA (ADR 0012), for hull pairs and hull against ball.

  One contact per pair: normal from A to B, depth, point.
- **The details that decide bits:**
  - glam's `DVec3::clamp` (`max` then `min`) where Rust clamps a vector, and `f64::clamp` where it clamps a parameter.
    They differ on a NaN and on ±0. `ImVec3_clamp` joins math3, with 229 `b3_math` rows;
  - `ZERO − la`, a subtraction, in `capsule_box_3d`'s centre projection, against `−la`, a negation, in
    `segment_box_3d`'s;
  - `Sum` folds from +0, so the clip fallback starts there;
  - the NaN polarity of each test, e.g. `if len < 1e-6 { continue }` doesn't skip a NaN length.

  Three inversions the first draft had were caught in review, before any run.
- **GJK and EPA's state** is an `ImGjkScratch3d` the caller owns and reuses: the polytope's vertices and faces
  (`ImRecordList`, now in `src/aabb_tree.cyr` for both profiles) and the horizon edges (`ImWordList`). The simplex is
  four records on the stack. Rust's `Vec` semantics are kept, since they decide which face wins: `retain` keeps
  order, `swap_remove` moves the last edge into the hole, and the closest face is the first minimum by `total_cmp`
  (its key built in `_im_b3_total_key`). If a list can't grow (no memory), the pair returns no contact rather than
  aborting. The narrowphase's function-local constants (`REL_TOL`, `ABS_TOL`, `GJK_MAX_ITER`, `EPA_MAX_ITER`) are
  generated.
- **`b3_narrow.tsv`**, 16,340 rows through the oracle's `contact_3d` hook:
  - every ordered pair of a 15-shape catalog (two balls, three boxes, three capsules, two segments, five hulls with
    the flat and two-point ones), at 4 rotation pairs × 3 directions × 5 distances;
  - the mesh, heightfield, empty and one-point hulls against all of them;
  - targeted cases: exact touching, concentric, ball centres inside the box near every face, edge and corner,
    face-touching and edge–edge boxes, the 1e-12 identity test, lying and standing capsules, segments through and
    along a box, degenerate hulls (flat, collinear, duplicate points) and 24- and 64-point clouds, coordinates to
    1e12, NaN and ±∞;
  - 2,000 seeded pairs.

  Every input rotation is a quaternion normalized from rational components, so no input depends on the host's sin
  or cos. **Class A** (the narrowphase has no transcendental): **all 16,340 rows match bit for bit**, 6,828 contacts
  and 47,796 floats with signs of zero, on x86_64 ELF, x86_64 PE and aarch64, on the first run.
- **The transcription.** `validate.py` ports the narrowphase to Python, Rust's `Vec`s as lists with the same
  semantics, and reproduces every row. A tampered output and a dropped contact both fail it.
- **Rust tests.** `tests/b3_narrowphase.tcyr` ports the 37 narrowphase tests of `backend_3d/tests.rs`:
  - the shape-pair tests and the closest-point tests (the 300-pair brute-force check on the test's own LCG);
  - the normal-direction sweep;
  - GJK on a collinear start, EPA at its cap on two balls, the hull–sphere depths, the AABB and OBB contact points;
  - the edge-axis separation and edge–edge contact.

  Cyrius-only groups cover the unhandled pairs and the scratch reuse (no heap growth over 600 GJK pairs after
  warm-up): 65 assertions.
- **Performance** (`docs/benchmarks-rust-v-cyrius.md`). Against Rust timing the same pairs through the same hook:
  - the closed forms are 6–20×;
  - the OBB SAT 14–17× after a pass that reads the overlap test, the clip and the hull support from f64 words. It
    was 25×; the bits are identical;
  - GJK + EPA 6–11×.
- **Result, every target:** 3D **23,890 ok, 0 failed** (3 modules); 2D unchanged at **98,874 ok**.

## 19. M4c: the 3D raycast; M4 done (2026-11-09)

- **Scope.** `backend_3d/raycast.rs` is the state's `raycast` and `raycast_filtered` over `raycast_impl`, and three
  ray tests. Rust 1.4.0's 3D world has no overlap queries: `PhysicsWorld::overlap_sphere` and `overlap_aabb` return an
  empty list in 3D ("not implemented yet"). So there is nothing more to port, and nothing in M4 pins `world_aabb`;
  M5's broadphase will.
- **`src/b3_raycast.cyr`:**
  - `ray_sphere` (the nearer non-negative root, normal `normalize_or(+y)`);
  - `ray_aabb_3d`, by slabs: a near-zero direction component needs the origin inside its slab, and the normal is the
    entry face's, zero from inside;
  - `ray_capsule`: six candidates (the infinite cylinder's two roots, both end balls' two), the nearest
    non-negative one within 1e-9·max(r, 1) of the surface. `EPSILON * EPSILON` is computed, as Rust writes it, not
    the `EPSILON_SQ` constant;
  - `raycast_impl`'s loop as `_im_b3_raycast` over a body arena and a collider arena in slot order (Rust's
    `values()`), for M5's state to call.

  The layer filter is a has-mask flag: Rust's unfiltered `raycast` passes `None`, which skips nothing, not even a
  layer-0 collider, so it isn't the same as an all-ones mask.
- **`b3_ray.tsv`**, 4,138 rays through the public world API, static bodies with one collider each:
  - every shape kind (balls, boxes and capsules with zero sizes among them; a hull, a segment and a heightfield,
    which the raycast doesn't hit) at four poses, with rotations about z and collider offsets;
  - 52 rays per pose: from each axis at the four max distances (100, 3.9, 4.0, ∞), obliques, a tangent and a near
    miss, from inside, parallel to a slab inside and out, along the axis, and away from the body, each filtered and
    unfiltered;
  - degenerate rays: zero, 1e-300 (its length underflows, so no direction), NaN and infinite directions,
    overflowing ones, non-finite origins, and max distances 0, −1 and NaN;
  - several bodies: the nearest hit, an exact tie (the first collider keeps it), hits behind the origin, and layers
    0, 1 and 2 under masks 0 to 4, all-ones and none;
  - 2,000 seeded scenes of 1–6 bodies.

  Class C (a body's rotation is `from_rotation_z`, a `sin_cos`). 4,136 rows are 0 ulp. Two rows have a `sin` that
  glibc misrounds; one of them, cancelling, was 2,765 ulp off the oracle. `validate.py --write` corrected them
  (`b3_ray.cr.tsv`, 2 rows), and the port matches both corrections exactly. Same on all three targets.
- **The transcription.** `validate.py` transcribes the three ray tests and the loop, and reproduces all 4,138 rows
  on glibc's sin and cos.
- **Rust tests.** `tests/b3_raycast.tcyr` ports all 7 raycast tests of `backend_3d/tests.rs`. The four state-level
  ones run on a test-local pair of arenas, filled as the state fills its own. A Cyrius-only group covers layer 0
  under both entry points, and there is a flat heap over 1,000 rays: 23 assertions.
- **Performance.** `queries/raycast_100_bodies` on the 3D build takes 21.1 µs against Rust's 2.28 µs (criterion on
  `--all-features`; the bench history holds only the 2D build), 9.3×.
- **M4 result, every target:** 3D **28,026 ok, 0 failed, 2 accepted (ADR 0011)** over b3_body, b3_math, b3_narrow and
  b3_ray; 2D unchanged at **98,874 ok, 0 failed, 7 pending, 165 accepted**. Surface parity is 604 across 26 modules.
  Pending M5: the two 3D state raycast methods; M6: the 3 snapshot functions.

## 20. M5a: correctly rounded atan2 and the 3D state (2026-11-09, ADR 0011 amended)

- **atan2 first.** The 3D state's `get_body_state` returns a body's rotation as 2·atan2(q.z, q.w). The port's atan2
  was ganita's on the atan polyfill, which misrounds 25.70 % of ordinary arguments, so the `f64::atan2` rows had a
  2-ulp bound and the 3D state's rows could not have been gated at 0. The maintainer chose a correctly rounded atan2
  before the state, as for pow (§16). `_im_cr_atan2` (ADR 0011's amendment has the design):
  - r = min/max of |y| and |x|. Below 2^-60, with no quadrant move, the answer is the IEEE quotient, except at an exact
    midpoint of the subnormal grid (num = A·2^α, den = B·2^β, A and B odd, B | A, α − β = −1075), which rounds
    toward zero;
  - phase 1 in double-double: r exactly with its remainder, c = i/64, t = (r − c)/(1 + rc), atan(c) from a generated
    table, the series through t^11, the quadrant moves with π and π/2 in double-double. Tested against 2^-63·|z|, it
    decides 99.9 % of arguments;
  - the slow path: vectoring CORDIC in the 364-bit fixed point, each atan(2^-k) from its series on demand, so there
    is no static table.

  It matches MPFR on 1,355,242 pairs in two seeded sets (wide magnitudes, |y| ≈ |x|, ratios at the threshold and at
  the table's boundaries, constructed subnormal ties, random bit patterns, the specials). glibc 2.44 misrounds 0.56 %
  and 0.14 % of the two sets. `tests/crmath.tcyr` adds the specials, the tiny path and its ties, MPFR values, and fast
  = slow over a sweep (78 assertions).
- **The atan2 rows.** All 1,200 `f64::atan2` rows are gated at 0 ulp (the bound was 2). glibc misrounds one, so
  `math.cr.tsv` grows to 22 rows and the 2D corpus moves one row from ok to the corrected oracle. The former atan2 stays
  in the harness as an ungated shadow, `f64::atan2 ~ atan polyfill`, 1 ulp off on 239 rows.
- **`src/b3_state.cyr`**, `ImPhysicsState3d`: the body, collider and joint arenas, each body's collider list (a
  word list per body slot), the contact tracker and the world-owned GJK scratch.
  - Rust's add, remove and `_at` methods. A body's removal forgets its tracker pairs and takes its colliders and joints;
    a joint's keeps normalize(q_a⁻¹·q_b) and its anchors at z = 0.
  - `recompute_mass_properties` over every collider, with M4's three steps.
  - Forces add p × f to the torque; impulses go through R·I⁻¹·Rᵀ; torques act about z.
  - `get_body_state` (rotation 2·atan2(q.z, q.w), angular velocity's z), `set_body_state` (`from_rotation_z`; wakes
    the body and refreshes its world inverse inertia) and `set_body_type`.
  - `raycast` and `raycast_filtered` over `_im_b3_raycast`.
  - `add_collider` and `add_joint` return a Result and refuse a body that isn't live (ADR 0008 #1).
- **`b3_state.tsv`**, 411 scripts on `PhysicsState3d` through the public world API, each ending in the 3D backend
  dump (`scripts/oracle/src/dump3d.rs`: full quaternions, angular velocity vectors and both inertia tensors; the 3D
  backend keeps no manifolds). The token writers both dumps share moved to `dumpcore.rs`, and the 2D corpus is
  byte-identical. The scripts:
  - an empty world, slot reuse and the LIFO free list;
  - mass: colliders adding up through the parallel-axis shift, a removal's recompute, `set_body_type` both ways;
  - forces, impulses and torques with and without points, on static, dynamic and sleeping bodies, and the wakes;
  - joints' reference rotations, removals that wake both bodies, a body's removal taking its joints and colliders;
  - teleports, and the missing-body errors of get, set and `set_body_type`;
  - 3 orphan scripts (`b3::state_script_orphan`);
  - 400 seeded scripts of 5–30 ops over live and stale handles, every shape kind, rotations and offsets.

  Class C. 403 scripts at 0 ulp (100,842 floats). The 5 where glibc misrounds an atan2, a cos or a sin match the
  corrected oracle (`b3_state.cr.tsv`), and the 3 orphan scripts are mapped and accepted (ADR 0008 #1).
- **The transcription.** `validate.py` replays every script on a Python model of the 3D state on glibc's sin, cos and
  atan2, and on MPFR's for the corrected oracle.
- **Rust tests.** `tests/b3_state.tcyr` ports the state-level tests of `backend_3d/tests.rs` that need no step (body
  counts, teleports, the missing body, both type changes, colliders accumulating mass, the parallel-axis theorem with
  its removal, the world-space angular impulse), with Cyrius-only groups for the refusals, a joint's reference
  rotation, removals and a flat heap: 25 assertions. The four state-level raycast tests now run on
  `ImPhysicsState3d`. The stepping ones (`remove_cleans_collision_pairs_3d`,
  `static_body_switched_to_dynamic_gains_mass`) wait for the step.
- **Result:** 3D **28,429 ok, 0 failed, 10 accepted (ADR 11: 7)** over 5 modules; 2D **98,873 ok, 0 failed, 7
  pending, 166 accepted (ADR 11: 71)**. The same on x86_64 ELF, x86_64 PE and aarch64, with equal digests;
  `f64::atan2`'s, now the correctly rounded one, is `0x6b403ab79185f15f` on all three. On aarch64 only the native
  shadows differ. Surface parity is 627 across 27 modules, with the M6 snapshot functions the only ones pending.

## 21. M5b: the 3D joints and the step when nothing touches (2026-11-09)

- **The modules:**
  - `src/b3_joints.cyr`, all of `backend_3d/joints.rs`:
    - `ImJointConstraint`, Rust's `JointConstraint` with `JointRows` and `FixedAngular` inline;
    - the Fixed joint's coupled 6×6 block through the Schur complement, falling back to the point block alone when
      K_ll or the complement can't be inverted;
    - the Distance and Rope rows, the Wheel's two perpendicular rows, and the split-impulse position passes;
    - the explicit forces once per sub-step: Spring, the Wheel's suspension, Mouse and the per-joint damping.
      Revolute and Prismatic are no-ops in Rust 1.4.0's 3D backend apart from that damping;
    - `break_joints`, on the rows' impulse over dt or the explicit force, whichever is larger.

    The constraints live in the state's scratch (`jc`), reused from step to step. Rust's paths for a joint whose
    body is missing (no rows; removed when its explicit force exceeds its break force) are ported, but the port
    can't reach them: `add_joint` and `add_joint_at` refuse a missing body (ADR 0008 #1), and `remove_body` takes
    the body's joints with it.
  - `src/b3_solver.cyr`, the stages of `step` that run when nothing touches: velocity integration with each body's
    world inverse inertia refreshed, the pseudo-velocities cleared, position integration, the per-body sleep check
    (the 3D backend has no islands), `finish_step` and `error_reduction`. The broadphase, narrowphase, wake, shock
    propagation, contact solve and `step` itself come with part 3.
- **The contact-free sub-step.** As in M3b (§14), the harness composes Rust's `step` from the library's stages, its
  contact stages empty (`tests/step3.cyr`). That is Rust's step only while the broadphase pairs nothing, so the
  composition also runs the broadphase's own filters on every collider pair (same body, two static bodies, two
  sensors, the layer test) and checks that no two speculative world AABBs overlap. A pair that could touch fails the
  row.
- **`b3_step.tsv`**, 333 rows, every collider at mask 0:
  - free fall at 1, 4 and 8 sub-steps, damping with a gravity scale, the speed cap;
  - free spins with gravity off: an asymmetric box tumbling off its principal axes (the gyroscopic step), a ball, a
    fixed-rotation box, an offset capsule, a fast spin; a torque every frame, and a force at a point every frame;
  - Distance pendulums at 1 and 4 sub-steps and a damped Distance joint;
  - Fixed joints: the cantilever, a rotated pair on a rotated static pin, a fixed-rotation body (the point block
    alone);
  - a Wheel under a pinned chassis, a Wheel with a zero axis, and one with a diagonal axis;
  - a Spring (and one at zero length), a Rope from slack to taut, and Mouse joints capped, uncapped and with a NaN cap;
  - Revolute and Prismatic with damping and a motor, and a joint between two static bodies;
  - breaking: a Distance joint by its impulse, a Spring and a Mouse by their force, a Wheel by its two impulses'
    hypot, a Fixed joint and a Rope, beside ones that hold;
  - a kinematic driver, a push every frame, a 10-link chain of mixed joints, two bodies falling asleep;
  - the solver settings: Baumgarte, no position iterations, 1 and 8 velocity iterations, a 1/30 s timestep;
  - 50 seeded scenes: two to six bodies of every collider kind, kinematics, fixed rotations, offsets, spins, every
    joint type, damping and break forces, random iteration counts and sub-steps.

  Class C, since a Wheel's break test is a `hypot`. **All 333 at 0 ulp** (91,829 floats) on the first run. These are
  the first golden rows for `integrate_velocities` (the gyroscopic step included) and `integrate_positions`. Perturbing
  the rotation error's factor of 2 by 1e-7 fails 32 rows, so the rows pin the angular lock.
- **The transcription.** `validate.py` transcribes the sub-step from rust-old: integration and the gyroscopic step,
  every joint path, sleep, and the tracker. It reproduces all 333 rows bit for bit, and asserts every collider's mask
  is 0. MPFR's hypot changes none of them, so there is no `b3_step.cr.tsv`.
- **Rust tests.** `tests/b3_joints.tcyr` ports the 11 stepping tests of `backend_3d/tests.rs` whose colliders can't
  meet: gravity, `fixed_joint_3d`, the type switch with its mass, the sleeping body that doesn't drift, the hanging
  ball and the pendulum, the rope, the cantilever's angular lock, the wheel, explicit forces independent of
  iterations, and breaking on reaction force. Each steps through the composition, and its guard confirms that no frame
  could have paired two colliders. Cyrius-only groups cover the Prismatic no-op, breaking by explicit force, the
  point block alone, `error_reduction` and a flat heap. `impulse_changes_velocity_3d`, which needs no step, joined
  `tests/b3_state.tcyr`.
- **Result:** 3D **28,762 ok, 0 failed, 10 accepted (ADR 11: 7)** over 6 modules; 2D unchanged at **98,873 ok**.
  The same on x86_64 ELF, x86_64 PE and aarch64: every b3_step row at 0 ulp on each, so the bits are equal. Surface parity is 632 across 28 modules; `backend_3d/solver` stays pending until part 3.

## 22. M5c: the 3D contacts and `step` (2026-11-09)

- **The modules.** `src/b3_solver.cyr` now holds all of `backend_3d/solver.rs`:
  - the broadphase: each collider's world AABB swept along its body's velocity, the spatial hash (cell size from the
    largest extents) or the AABB tree, then Rust's filters and the overlap test, in candidate order;
  - the narrowphase stage through M4's `generate_contact_3d` on the state's GJK scratch;
  - wake on contact;
  - the shock-propagation order, a stable counting sort on the lower level of each contact's bodies;
  - `prepare_contacts` (the combine rules, `sanitize_coefficient`, the restitution target), `solve_contacts` (the
    normal impulse, two-direction friction with the static cone, rolling resistance), `solve_contact_positions`;
  - `step` and `body_pseudo_velocity`.

  `STATIC_FRICTION_SLIP_SPEED` and `RESTITUTION_VELOCITY_THRESHOLD` are generated. Cyrius takes no field addresses, so
  a collider's shape and the state's GJK scratch are reached by computed address. Two helpers do it, and
  `tests/b3_state.tcyr` pins both offsets. A shape copy would be a 184-byte byte-loop `memcpy` per pair.
- **`b3_step` through `step`.** The harness now runs the library's `step`, and `tests/step3.cyr`'s composition is
  gone. All 333 contact-free rows still match at 0 ulp, so the whole step with an empty broadphase equals part 2's
  stages.
- **`b3_contact.tsv`**, 450 rows:
  - for each broadphase, a ball, a box, a box at an angle, a cube hull (GJK + EPA) and a lying capsule dropped on the
    ground;
  - a stack of five boxes and a pyramid of six (the shock-propagation levels);
  - restitution at 0.9, 0.6 and 0 on fast drops, and the Max, Multiply and Min combine rules;
  - friction on a slope at three settings (one a static override), a box shoved and a ball spun along flat ground,
    rolling resistance;
  - capsules standing and crossing, a ball on a segment and on a hull, a box on a lying capsule, an offset collider, a
    tumbling box;
  - a sensor, a ball whose layer and mask meet nothing on the ground, two overlapping sensors, a kinematic paddle;
  - a box falling asleep and a ball dropped later waking it;
  - the position solver's settings (Baumgarte at two factors, a stiff soft constraint, more slop, more iterations,
    four sub-steps) and a push every frame;
  - fast bodies (the swept boxes), joints with contacts;
  - 40 seeded scenes in a walled box.

  Class C. 449 rows are 0 ulp (176,143 floats). In the other, glibc rounds one friction `hypot` 1 ulp low, which ends
  64 ulp off the oracle in a few outputs; the port equals the corrected oracle (`b3_contact.cr.tsv`). Perturbing the
  restitution target by 1e-7 fails 5 rows.
- **The transcription.** `validate.py` adds the contact stages to its 3D sub-step: a 3D `world_aabb`, the broadphase
  by brute force (as for `b2_contact`: both broadphases report a superset of the overlapping boxes in sorted order),
  M4's narrowphase transcription, wake, the order, and the contact solver. It reproduces all 450 rows on glibc, and
  on MPFR writes the one correction.
- **ADR 0008 #5, which part 1 missed.** The ADR says the 3D profile's `add_joint` refuses Revolute and Prismatic
  joints and motors with `Err(IM_ERR_UNSUPPORTED)`; Rust 1.4.0 accepts them and does nothing with them but their
  damping. Part 1 accepted them as Rust does. Now `add_joint` and `add_joint_at` refuse them, after the missing-body
  check. `b3_state.tsv`'s seeded scripts no longer draw them, and three `b3::state_script_unsupported` scripts are
  mapped and counted, as the orphans are. The corpus is now 414 scripts with 2 corrected rows (the seeds changed),
  every plain script at 0 ulp. Rust's `explicit_joint_forces_independent_of_iterations` damps a Revolute joint, so its
  port damps a Wheel with a zero axis, which also has no rows and no force: the same Δv. The step rows load dumps
  directly and still hold Revolute and Prismatic joints, so they keep pinning how the step treats them.
- **A faster shared broadphase.** Timing the 3D step stage by stage put most of a frame in the spatial hash's
  `query_pairs`. Its record sort now moves words instead of byte-copying records, and its pair sort compares inline
  (`src/spatial_hash.cyr`, `src/contact_tracker.cyr`). Neither changes an order. Both profiles use them, and the 2D
  corpus (98,873 ok) and the 3D one pass unchanged on all three targets.
- **A comment fixed.** Writing the layer scene showed that `b2_contact`'s "a ball whose mask misses the ground's layer
  (it falls through)" lands: a pair is dropped only when neither layer meets the other's mask, and the ground's mask
  is all layers. The 2D emitter's comment now says so; its data is unchanged. The 3D scene gives its ground layer 1
  and mask 1, so its ball does fall through.
- **Rust tests.** `tests/b3_contacts.tcyr` ports the 10 stepping tests with contacts: collision events, the tracker
  on removal, the spatial hash's pair and no false pair, the split impulse's per-step fraction, restitution at any
  iteration count, kinetic and static friction, bad coefficients, and a box at rest off centre. Cyrius-only groups
  cover the two broadphases stepping to the same bits, Started, Ongoing and Stopped, a sensor, the layer filter,
  sleep and wake on contact, and a flat heap: 23 assertions. `tests/b3_joints.tcyr` now steps through `step` (31
  assertions, with the #5 refusals). Its flat-heap check warms for 30 frames, the broadphase's high-water mark.
- **Result:** 3D **29,214 ok, 0 failed, 11 accepted (ADR 11: 5, ADR 0008: 6)** over 7 modules; 2D unchanged at
  **98,873 ok**. The same on x86_64 ELF, x86_64 PE and aarch64. Surface parity is 637 across 29 modules, every 3D backend module but the tests file.

## 23. M5d: the 3D world; M5 done (2026-11-10)

- **One world, two profiles.** Port plan §4 step 3 makes `world.cyr` shared, but M3 wrote it over the 2D backend. It
  is now split. `src/world.cyr` is the part both profiles share: the step's frame counter and sub-step loop, the
  particles' spawn, integration, sub-emitters and removal, and every accessor. Each profile's file comes before it
  and holds what touches its backend:
  - the `ImPhysicsWorld` storage over that backend, and `ImPhysicsWorld_new`;
  - `_substep` (the 2D one passes the solver kind; the 3D backend has only sequential impulses);
  - `_collide_particles`;
  - `overlap_sphere` and `overlap_aabb`.

  `src/b2_world.cyr` is M3's 2D code, moved unchanged (the 2D corpus passes as before). `src/b3_world.cyr` ports
  Rust's 3D paths: `particle_vs_local_box` and `particle_vs_rounded_segment` with N = 3, and
  `particle_vs_collider_3d`. A Cyrius struct can take a second `impl` block, so the shared methods and the profile's
  methods are one type's.
- **Rust's 3D world, not 2D's.** Three behaviours differ from the 2D world, and the port keeps each:
  - the config's solver kind is never read: an unvalidated XPBD config steps as sequential impulses
    (`ImPhysicsWorld_try_new` still refuses it, ADR 0008 #6);
  - particles test every non-sensor collider with no bounding-box pre-check (a pre-check on rounded boxes could
    reject a pair the exact test accepts);
  - the overlap queries return no colliders ("3D overlap not implemented yet").

  Rust computes a collider's centre for every particle; the port computes it once a step per collider. That is the
  same value from the same operations, since no body moves while particles collide. Each sum runs x, y, z from the
  left, which is what Rust 1.99's `Iterator::sum` does, folding from −0.0.
- **`b3_world.tsv`**, 190 rows, `world_step.tsv`'s layout with the 3D dump:
  - an emitter over a floor with spread on every axis, an inactive emitter and two with no valid rate; a fast
    emitter at two sub-steps;
  - radial fields on rings in all three coordinate planes (falloffs 1 and 2, repulsion, a reach limit, a particle at
    the centre); a directional box thin in z, with a radial field;
  - collisions with every shape: boxes plain and turned, a ball, a turned capsule, a segment across z, a hull (no
    contact), a sensor, an offset box on a turned body, at restitutions 0, 0.3 and 1, and a particle inside a box, one
    of radius 0 and one meeting a z face;
  - streams along z into a box face, a ball, a turned box and a tumbling box, so every frame has hits with a z normal;
  - bodies spun about skew axes by an off-centre impulse, with gravity scale 0, under an emitter's particles. Rust's
    3D world builds bodies turned about z only, so this is how the corpus reaches other orientations;
  - a kinematic paddle with an offset collider, sub-emitters, a ball landing beside a field's particles, a pushed box,
    a distance-joint pendulum and a spring pair under a shower;
  - 30 seeded scenes at 1 to 3 sub-steps in either broadphase.

  Class C, bound 0 by the rule (§5): the world adds only crmath's `pow`, `sin` and `cos` to the 3D step. **All 190
  rows are 0 ulp** (97,299 floats) on the first run. MPFR agrees with glibc on every call in them, so there is no
  correction file. Perturbing the z restitution term by 1e-7 fails 9 rows, and using the rotation for its inverse in
  the box test fails 13. The z-stream scene was added after the first mutation failed only 2 rows (most hits are on
  floors, whose normals have no z).
- **The transcription.** `validate.py`'s particle step is now a class both step models share, each supplying its own
  `collide_particles`. The 3D model adds `frame` (the sub-steps and `finish_step`), `world_step`, and the three 3D
  contact functions. It reproduces all 190 rows on glibc.
- **Rust tests on both profiles.** Rust's `--all-features` build runs `world.rs`'s tests, `integration.rs` and the
  shared regressions on its 3D world, from the same source as on 2D. The port now does the same:
  - `tests/world_cases.cyr` holds the 24 world tests, the 8 particle tests and the shared Cyrius-only groups;
  - `tests/regression_cases.cyr` holds the 9 integration tests and the 12 shared regressions.

  `tests/world.tcyr` (55 assertions) and `tests/regressions.tcyr` (44) run them on 2D, the latter with its 14
  2D-only tests. `tests/b3_world.tcyr` runs them on 3D with 19 Cyrius-only assertions:
  - the contact for each shape, turned and off the origin;
  - particles against a sensor, a hull and an offset collider in a world;
  - the empty overlaps;
  - the unread solver kind, bit for bit;
  - 60 frames of three sub-steps through the world against the backend's own `step` at a third of the timestep, bit
    for bit;
  - a flat heap with contacts, joints, an emitter and a field.

  That is 74 assertions. `tests/b3_regressions.tcyr` runs the 21 shared cases with Rust's 3D-only
  `substeps_are_honoured_in_3d` (29 assertions).
- **Shared plumbing.** The oracle's world-state tokens moved from `dump2d.rs` to `dumpcore.rs`. The harness's
  particle, emitter and field readers and writers moved to `tests/parity_types.cyr`, with `_par_got_config` and
  `_par_in_f`.
- **The audit's docs step.** `cyrius audit` exits 1 when a public function has no doc comment directly above it, and
  the committed tree had 44: test helpers and `main`s, three constructors in `src/`, and `ImCollider3d_from_desc`,
  whose comment part 3 separated from it by inserting `_im_b3_coll_shape` between them. The earlier parts read the
  audit's output and missed its exit code. All 44 now have one, and the audit exits 0.
- **ADR 0008 #4** (the full 3D `BodyState`) was not part of this section. It came next, in §24.
- **Result:** 3D **29,404 ok, 0 failed, 11 accepted (ADR 11: 5, ADR 0008: 6)** over 8 modules; 2D unchanged at
  **98,873 ok**. The same on x86_64 ELF, x86_64 PE and aarch64. Surface parity is 637 across 29 modules.

## 24. ADR 0008 #4: the full 3D body state (2026-11-10)

- **The type.** `ImBodyState` left the shared `src/types.cyr` for each profile's types file. The 2D one is Rust's.
  The 3D one (`src/b3_types.cyr`) has `rotation: ImQuat` (x, y, z, w) and `angular_velocity: ImVec3`. Its JSON codec
  went with it, from `src/json.cyr` to `src/b2_json.cyr` (unchanged) and the new `src/b3_json.cyr`, each after the
  shared JSON module in its profile. The 3D document is serde's for the 2.0 struct, `"rotation":[x,y,z,w]` and
  `"angular_velocity":[x,y,z]`. Rust 1.4.0's 3D document is refused.
- **Get and set.** `get_body_state` copies the body's quaternion and angular velocity (no more `atan2`).
  `set_body_state` stores them as given, then wakes the body and refreshes R·I⁻¹·Rᵀ. Rust's call with angle θ and spin ω
  is the 2.0 call with `ImQuat_from_rotation_z(θ)` and (0, 0, ω), bit for bit. The ADR's amendment records the
  choices: a type per profile, no normalizing on set, `BodyDesc` unchanged.
- **`b3_state.tsv`** keeps its 414 scripts; 313 of them read or write a state. A get's outputs are now the body's
  full state from the oracle's dump, 16 tokens with the quaternion and the angular-velocity vector. A set's inputs
  are the state Rust stores for its angle and spin, `DQuat::from_rotation_z(θ)` and (0, 0, ω). The generator asserts,
  after every set, that the dump holds that state bit for bit. Both compare by value, with no mapping. The scripts
  still need 2 corrections, both from `add_body`'s `from_rotation_z`. A get no longer needs `atan2`, and a set no longer
  needs `sin_cos`. **All 408 plain scripts are 0 ulp** (101,874 floats). Reverting the get to Rust's spin about z alone
  fails 7. `validate.py`'s model reads and writes the same 16 tokens and no longer takes an `atan2`.
- **Tests.** `tests/b3_state.tcyr` (34 assertions, +6):
  - Rust's `set_body_state_teleports` with its angle as `ImQuat_from_rotation_z(1.0)`, reading the whole state back;
  - an arbitrary orientation and spin read back exactly, the body woken, R·I⁻¹·Rᵀ for the new orientation, and a
    frame integrating from the state set;
  - the 3D JSON: an exact round trip, the document's text, and Rust 1.4.0's document refused.

  `off_centre_box_rests_without_spin`, shared by both profiles, reads the rotation through a helper each runner
  defines: the 2D angle, or the quaternion's whole angle in 3D. That is stricter than Rust's angle about z. The
  body-state token helpers moved from `tests/parity_types.cyr` to each harness.
- **Result:** 3D **29,404 ok, 0 failed, 11 accepted (ADR 11: 5, ADR 0008: 6)**; 2D unchanged at **98,873 ok**. The same
  on x86_64 ELF, x86_64 PE and aarch64. Surface parity is 637 across 29 modules.

## 25. M6a: 2D snapshots, the struct and `restore` (2026-11-10, ADR 0013)

- **No oracle.** Rust 1.4.0's snapshot is incomplete and its bytes are bitcode, so a 2.0 snapshot has nothing to
  compare against (port plan §2 item 5). It is checked three ways: by round trip (snapshot, restore, snapshot again:
  the same state), by step-equivalence (a restored world steps to the original's bits), and by refusal tests. The
  content needs no separate proof of completeness: it is what the oracle's dump holds, and every step corpus row
  loads exactly that into a fresh world and reaches Rust's bits.
- **The modules.**
  - `src/snapshot.cyr` holds the pieces both profiles share: an arena's slots and free list (`ImArenaSnapshot`), the
    tracker's lists and last events (`ImTrackerSnapshot`), checked shape copies, and the shape, material and
    joint-type checks.
  - `src/b2_snapshot.cyr` holds the 2D `ImWorldSnapshot` over the backend's own records: `ImBodySnapshot` (the body
    record and its collider list), the collider record, `ImJointSnapshot` (handle and record) and the manifold
    record with its points. It also holds the backend half of capture, checks and load.
  - `src/world_snapshot.cyr` holds the world's half and the public `snapshot` and `restore`. `restore` checks
    everything, builds a fresh world beside the old one and only then swaps it in.

  The order in `[lib]`: JSON, then `snapshot.cyr`, the profile's file, `world_snapshot.cyr`. The 3D profile has
  `snapshot.cyr` alone until part 4.
- **The constant.** `IM_MAX_SNAPSHOT_INDEX` (Rust's `MAX_SNAPSHOT_INDEX`, 2^20) is generated with the config limits.
  Its `constants.tsv` row, pending since M1, now replays: 2D **98,874 ok, 6 pending** (S3 3, S4 3).
- **Tests** (`tests/snapshot.tcyr`, 31 assertions). A comparison follows every owned array, so two snapshots are equal
  only bit for bit.
  - Rust's `restore_rejects_huge_handle_index` (a huge index, a duplicated collider, an orphan collider), and
    `restore_replaces_force_fields`, one of Rust's two `#[ignore]`d tests, at the struct. Rust's byte-level tests come
    with part 2.
  - Round trips in five scenes: a ball on a floor, a stack hit by a ball, every joint type with a motor, limits, a
    breaking rope and damping, particles with emitters, fields and sub-emitters, and removals with a sensor.
  - Step-equivalence in eight runs. Snapshot after k frames, restore into a world that held another scene, then step
    both. The checks are the state every 25 frames and at the end, the events every frame, and the next body and
    collider handles. The runs cover Rust's `step_exact_determinism` (50 + 100 frames), the stack in contact and
    asleep, the joints under a force every frame, mid-interval emitters, the free list, four sub-steps on the AABB
    tree with a force applied before the snapshot, and a snapshot before the first step.
  - 13 refusals: an invalid config, XPBD, an enum out of range, a list missing or doubling a collider, a stale
    generation, a three-point manifold, a manifold keyed larger handle first, a tracker list out of order, a free list
    that loops or passes an occupied slot, too many slots, and a joint on a missing body. Each leaves the world it was
    restored into bit for bit unchanged; a Mouse joint's missing second body is accepted, as Rust's check allows.
  - A flat heap over 60 snapshot-and-restore rounds.
- **Mutations.** Zeroing the manifolds' normal impulses on load fails 5 checks, rebuilding the free list instead of
  carrying it fails 6, and skipping the tracker fails 5.
- **Result:** 2D **98,874 ok, 0 failed, 6 pending, 166 accepted**; 3D **29,404 ok, 0 failed**; the same on x86_64
  ELF, x86_64 PE and aarch64, where `tests/snapshot.tcyr` also passes. Surface parity is 662 across 30 modules:
  `serialize` joins, with `serialize_world` and `deserialize_world` pending part 2.

## How to reproduce

```sh
cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml -- --check
python3 -I scripts/oracle/validate.py .           # also checks src/crmath_data.cyr and tests/golden/*.cr.tsv
python3 -I scripts/oracle/validate.py . --write   # regenerates them (needs libmpfr.so.6)
cyrius test tests/parity_2d.tcyr                  # x86_64 ELF
cyrius test --win tests/parity_2d.tcyr            # x86_64 PE, needs wine
cyrius test --aarch64 tests/parity_2d.tcyr        # aarch64, needs qemu-aarch64
cargo run --release --locked --manifest-path scripts/oracle/Cargo.toml --no-default-features --features b3 -- --check
cyrius test tests/parity_3d.tcyr                  # the b3_* modules; also --win and --aarch64
cyrius run scripts/probes/p1_memory.cyr           # §4.1
cyrius bench scripts/probes/p2_vectors.bcyr       # §4.2
```

Compare the `digest` lines of the three runs: they must be equal for every dispatched function.
