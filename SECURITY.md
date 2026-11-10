# Security Policy

## Reporting a Vulnerability

If you discover a security vulnerability in impetus, please report it
responsibly through **GitHub Security Advisories**:

1. Go to the [Security tab](../../security/advisories) of this repository.
2. Click **"Report a vulnerability"**.
3. Fill in the details and submit.

**Do not open a public issue for security vulnerabilities.**

## Response Timeline

| Action | Target |
|---|---|
| Acknowledgement | Within **48 hours** |
| Initial assessment | Within **5 business days** |
| Fix for critical severity | Within **14 days** |
| Fix for high severity | Within **30 days** |
| Fix for moderate/low severity | Next scheduled release |

## Scope

This policy covers the impetus library, the Cyrius bundles `dist/impetus.cyr` and
`dist/impetus-3d.cyr`, and its public API. The Rust crate 1.4.0 in `rust-old/` is
frozen as the port's parity oracle. Vulnerabilities in dependencies (the Cyrius stdlib,
bayan) should be reported to the respective upstream projects, and flagged here if they
affect impetus users.

## Attack Surface

impetus does no I/O and no network access; it computes on what its caller gives it. The
inputs that can come from outside a program are the ones it parses:

| Input | Risk | Mitigation |
|---|---|---|
| Snapshot bytes (`ImWorldSnapshot_from_bytes`, `deserialize_world`) | crafted or damaged bytes | the header's magic, version, profile, length and FNV-1a checksum are checked before any field is read; every count is checked against the words left before anything is allocated; `restore` validates the whole snapshot and leaves the world unchanged on any error ([ADR 0013](docs/adr/0013-complete-snapshots.md)) |
| Snapshot JSON (`ImWorldSnapshot_from_json`) | crafted documents | the same validation as the bytes, after bayan's parser |
| Type JSON (`Im<Type>_from_json`) | crafted documents | enum names and required fields checked; values read as serde does ([ADR 0010](docs/adr/0010-serde-compatible-json.md)) |
| `ImWorldConfig` | out-of-range settings | `ImPhysicsWorld_try_new` and `restore` validate it; `ImPhysicsWorld_new` does not, as Rust's `new` |

The fuzz targets (`tests/impetus.fcyr`, `tests/impetus_3d.fcyr`) feed mutated snapshot
bytes, JSON and configs on both profiles, and require a refused input to leave the world
unchanged. Library code never aborts, except that the JSON writers exit the process when
the stdlib's `str_builder` can't grow (out of memory).

## Disclosure

We follow coordinated disclosure. Once a fix is released, we will publish a
security advisory crediting the reporter (unless anonymity is requested).
