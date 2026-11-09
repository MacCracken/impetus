# 0010 — Serde-compatible JSON codecs on bayan, with a shortest-digit float printer

**Status**: Accepted
**Date**: 2026-11-09

## Context

Every public Rust type derives serde's `Serialize` / `Deserialize`, and the crate's tests round-trip
them through `serde_json` (the bitcode snapshot is separate: port plan M6). The first version of the
port plan, copying tanmatra and kimiya, said serde was "not carried over". That was wrong. The Cyrius
stdlib has:
- `#derive(Serialize/Deserialize)` (JSON through bayan);
- bayan's parsers (JSON, TOML, YAML, CSV);
- `lib/protobuf.cyr`.

The derive writes a different document from serde's, in three places:
- **Enums:** the Cyrius member name (`"IM_COMBINE_AVERAGE"`) where serde writes the Rust variant
  (`"Average"`).
- **`Option<T>`:** a flag plus a value where serde writes `null` or the value.
- **Payload enums** (`ColliderShape`, `JointType`): serde writes them externally tagged
  (`{"Ball":{"radius":0.5}}`), which the derive can't produce from a kind-plus-fields struct.

Its scanner decoder (`T_from_json_str`) also has two open defects on malformed input: it reads past
the terminating NUL, and it loops forever on a stray `}` in a `Vec<struct>`.

serde_json 1.0.154 prints an f64 with zmij (Schubfach): the shortest digits that round-trip, closest
to the exact value. Exponents from −5 to 15 print positionally, with `.0` on an integer; others print
as `d.ddde±x`. Non-finite values print as `null`. Rust's `Display`, which `Quantity` uses, prints the
same digits, always positionally. The stdlib has no such printer (port plan §8, S2). bayan's is
"shortest or near-shortest" and formats differently.

## Decision

The port reads and writes **the same JSON documents as the Rust crate's serde_json**, by hand-written
codecs:
- **Writers** on `str_builder`.
- **Readers** on `bayan_json_parse`'s tree, not the derive scanner, so malformed and untrusted input
  can't reach its open defects.
- **Serde's semantics:**
  - required fields, `#[serde(default)]` fields, and `Option` fields that may be absent;
  - unknown fields ignored, duplicate fields refused;
  - an integer accepted for an f64, a fraction refused for a `u32`;
  - `null` refused for an f64.

  A malformed document is `Err(IM_ERR_DESERIALIZE)`.
- **Floats** go through a ported shortest-digit algorithm (Burger & Dybvig's free-format algorithm on
  a small bignum, `src/fmtf.cyr`), formatted by zmij's rules for JSON and by Rust's rules for
  `Display`. It closes S2's 280 pending `Quantity::fmt` rows as well.

**Parity:**
- every serde_json document the oracle writes decodes to the oracle's values bit for bit (class A);
- the port's document for every golden value is serde_json's text, byte for byte (class A);
- `f64::fmt_display` and serde_json's f64 text are golden rows of their own.

## Consequences

- **Positive**
  - A Cyrius consumer reads the configs, scenes and descriptors a Rust consumer saved.
  - The serde round-trip tests port, rather than being dropped.
  - Display parity is reached too.
- **Negative**
  - impetus owns codecs and a float printer that the stdlib should provide.
  - Gaps to file upstream: S2 (the shortest printer), and derive variant renaming, `Option` as
    `null`, and externally tagged payload enums, so the codecs can move to the derive later.
- **Neutral**
  - The M6 snapshot is a separate decision; `lib/protobuf.cyr` is the candidate.

## Findings while implementing (2026-11-09)

- **The two Rust printers break exact ties differently**: core rounds up, zmij to the even digit (2^-25).
  `_im_shortest` takes the rule as a parameter; 300 ties are golden rows (`port-parity.md` §6).
- **serde_json's default float parse is best-effort** (1 ulp off on some long decimals), so the oracle
  uses its `float_roundtrip` feature, which matches the port's correctly rounded parse. Writing is the same
  in both modes.

## Alternatives considered

- **`#derive` now, file the gaps**: less code today, but no Rust-written document would load until
  the derive grows the three features. Rejected by the maintainer (2026-11-09).
- **bayan's f64 printer for the text**: round-trips, but isn't always shortest and formats
  differently, so the text wouldn't match serde_json's.
- **A Ryu or Schubfach port**: exact and fast, but needs 128-bit power-of-ten tables. The bignum
  algorithm is shorter, needs no tables, and printing isn't on the step path. A golden sweep against
  both Rust printers proves the digits.
