# 0013 — Complete snapshots: an inspectable struct, a checksummed binary form and JSON

**Status**: Accepted
**Date**: 2026-11-10

## Context

ADR 0008 #2 and #3 give the 2.0 port one fallible `restore` and a complete, versioned snapshot format. Rust 1.4.0's
`WorldSnapshot` is a struct of descriptors: bodies as a `BodyDesc` and their kinematic state, colliders, joints,
particles, emitters, the config and the two id counters, encoded by bitcode. It leaves out the force fields, the
contact manifolds and warm-start impulses, sleep timers, mass properties, the force accumulators, the arena slot
generations and free lists, and the contact tracker. A restored Rust world therefore steps differently from the
original and hands out different handles; its two regression tests for that are `#[ignore]`d. bitcode has no Cyrius
implementation, so the bytes can't be shared with Rust anyway, and with no oracle for a complete snapshot the format
is the port's to design.

Three choices were the user's (2026-11-10): an inspectable snapshot struct as well as bytes, a checksum on the bytes as
well as structural checks, and a JSON form as well as the binary one.

## Decision

A 2.0 snapshot holds **the whole state of a world, exactly what the oracle's `oracle_dump()` holds**, as one
`ImWorldSnapshot` per profile, with a checksummed binary form and a JSON form. A restored world is the original: it
steps to the same bits and hands out the same handles.

- **Content.** The world's config, particle and emitter counters, particles, emitters (with their spawn
  accumulators) and force fields; each arena's slots (occupied, generation, next free) with its free-list head and
  stale flag; every body, collider and joint record, field for field; each body's collider list, in attachment
  order; in 2D the contact manifolds with their warm-start impulses; the contact tracker's three pair lists and its
  flag; the last step's collision events. The step corpus is the proof that this is enough: every `b2_step`,
  `b2_contact`, `world_step`, `b3_step`, `b3_contact` and `b3_world` row loads such a state into a fresh world and
  steps it to Rust's bits. The step's scratch (broadphase grids, constraint rows, the island union-find) is rebuilt
  every step and is not part of it.
- **The struct.** `ImWorldSnapshot` holds the backend's own records rather than a copy of each type: the body record
  with its collider list (`ImBodySnapshot`), the collider record (Rust's `ColliderSnapshot`), the joint record with its
  handle (`ImJointSnapshot`), the manifold record, arena slots as `ImSlotSnapshot` in an `ImArenaSnapshot`, and the
  world's own particle, emitter and field records. Callers can read and edit it; `restore` checks it. A snapshot owns
  its arrays (shapes, manifold points, lists) and releases them with `free`. The 2D and 3D profiles each define it,
  as they do `ImPhysicsWorld` and `ImBodyState` (ADR 0008 #4: the 3D body record has the full orientation).
- **API.** `world.snapshot(out)` fills a snapshot: Ok(0), or Err(IM_ERR_SERIALIZE) with no memory.
  `world.restore(snap)` validates, builds a fresh world beside the old one and swaps it in. On any error the world is
  unchanged: Err(IM_ERR_INVALID_CONFIG) for a config `validate` refuses, otherwise Err(IM_ERR_DESERIALIZE). It is the
  one fallible restore of #2, Rust's `try_restore`. The bytes are `snap.to_bytes(out)`, `ImWorldSnapshot_from_bytes`,
  and Rust's `serialize_world` / `deserialize_world`. JSON is `snap.to_json(sb)` and `ImWorldSnapshot_from_json`.
  `IM_MAX_SNAPSHOT_INDEX` is Rust's `MAX_SNAPSHOT_INDEX` (2^20), generated.
- **The binary form, version 1.** Little-endian 64-bit words, the same bytes on every target (ADR 0008 #7). A header
  of five words: the magic, the format version, the profile (2 or 3), the payload's length in words, and a 64-bit
  FNV-1a over the payload words (h ← (h ⊕ w) × 0x100000001b3 from 0xcbf29ce484222325). Any one changed word changes it,
  since each step maps h one to one. Integers are words, f64 are their bits, flags are 0 or 1, an absent optional is a
  0 flag and zero value words, a list is its count then its items, and a payload enum (shape, joint type, force
  field) is its kind then that variant's own fields. So the bytes depend only on the state, never on a record's
  unused fields. The magic is "IMPSNAP\0". A wrong magic, version, profile, length or checksum is refused before any
  field is read, and every count is checked against the words left before anything is allocated.
- **JSON.** serde's form for the struct, each record's field names as keys, enums by variant name, an absent optional
  as `null` (ADR 0010's conventions), with the format and profile as the first keys. It has no checksum, since it is
  meant to be read and edited; it gets the same validation.
- **Validation** (`validate_snapshot`, a superset of Rust's):
  - the config passes `validate`;
  - an arena has at most `IM_MAX_SNAPSHOT_INDEX + 1` slots, each slot's words are in range, and its free list visits
    only free slots, once each;
  - each record's handle names an occupied slot at that slot's generation, with one record per occupied slot;
  - a collider's body is live, and each live body's list holds exactly its colliders;
  - a joint's bodies are live (a Mouse joint's second body needn't be), and enums are in range;
  - a manifold's colliders and bodies are live and consistent, with at most two points (2D);
  - the tracker's lists are sorted.

  Values are restored as stored, finite or not. A world that stepped into NaN restores to the same NaN; damaged bytes
  are the checksum's to catch.
- **Not carried over.** Rust's bitcode bytes and its 1.4.0 JSON of `WorldSnapshot`: neither holds the complete state,
  and neither has a Cyrius reader. `restore` without a Result (deprecated in 1.4.0) is gone (#2).

M6 lands in five parts: the struct and `restore` in 2D; the binary form; JSON; the 3D profile; benches, the fuzz port
and the close.

## Consequences

- **Positive**
  - Restore is exact: the step-equivalence tests compare the restored world bit for bit, events and next handles
    included, and Rust's two `#[ignore]`d regression tests run.
  - A snapshot saved on one target restores on the others to the same bits.
  - Reusing the backend's records keeps one definition of each piece of state.
- **Negative**
  - The struct's record types are the backend's, so a change to a record changes the snapshot API, and the binary
    and JSON forms bump their version.
  - No oracle: the format is checked by round trip, step-equivalence and refusal tests only.
  - The JSON form is a large codec with no Rust counterpart to compare against.
- **Neutral**
  - A consumer moving from Rust re-saves snapshots; the 1.4.0 bytes can't be read.

## Alternatives considered

- **Bytes only, no struct.** Simpler, but the user chose an inspectable struct.
- **A snapshot type for every record** (`ImBodySnapshot` with its own copy of every body field, and so on). It gives
  the snapshot an API apart from the backend's records, at the cost of a second definition of each record and a
  copy function for each, kept in step by hand. Rejected: the records are already public, documented types.
- **Structural checks only.** A flipped bit in a float would restore silently. Rejected by the user.
- **Binary only.** Smaller, but the user chose a JSON form too.
- **Refusing non-finite values.** Rust's `validate_snapshot` doesn't, and refusing would make a diverged world
  impossible to save and inspect. The checksum covers corruption.
