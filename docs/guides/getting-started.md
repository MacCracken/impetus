# Getting started with impetus

## Build

```sh
cyrius deps                              # resolve dependencies
cyrius build src/main.cyr build/impetus    # compile
cyrius test                              # run tests/*.tcyr
```

## Layout

- `src/main.cyr` — entry point. Top-level `var r = main(); sys_exit_group(r);`.
  Use `sys_exit_group`, not `syscall(SYS_EXIT, r)` — the latter is exit(2) and ends
  only the calling thread, so it hangs once the program spawns its first worker.
- `tests/` — test suite (`.tcyr` files, auto-discovered by `cyrius test`).
- `rust-old/` — the Rust 1.4.0 crate, the parity oracle. Do not modify it.
- `docs/development/impetus-port-plan.md` — the port plan: milestones, parity classes, mappings.

## Porting a module

1. Add `src/<module>.cyr` and list it in `cyrius.cyml` `[lib]` (and `[lib.3d]` when shared), in
   dependency order. No `src/` file includes another; the list order is the resolution order.
2. Add its golden rows (`scripts/oracle/`) and its ported Rust tests (`tests/<module>.tcyr`).
3. Run the cleanliness check in `CLAUDE.md`; `cyrius test` must be warning-free.
4. Record its benches against the Rust numbers and update `state.md`, the CHANGELOG and the
   plan's tracking table.

See [`../adr/template.md`](../adr/template.md) when a non-trivial design choice deserves an ADR.
