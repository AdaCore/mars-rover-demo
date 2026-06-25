# `simulator_interface` — Ada desktop static library

Second Ada build root. Produces `libMars_Rover.a` (+ `libgnat`) from the same
`src/` used by the RP2040 firmware, for linking into the Rust workspace.

## Why this exists

The repository has two Ada build roots:

| GPR | Target | Source dirs | Output |
|---|---|---|---|
| `/mars_rover.gpr` (repo root) | RP2040 via Pico BSP | `src/ src/device/` | executable (`bin/mars_rover`) |
| `simulator_interface/mars_rover.gpr` | desktop (native) | `src/ src/binding/` | static library `libMars_Rover.a` |

The root GPR pulls in `pico_bsp` and cross-compiles for the Pico — the result
cannot be linked into a desktop binary. This GPR is the platform-neutral
sibling: same Ada source, different HAL body (`binding/` instead of
`device/`), library-kind output instead of executable.

## Who builds this

You normally don't build it directly. `crates/mars-rover-ada-link/build.rs`
invokes `alr build` here and emits the `-L$OUT_DIR/lib -lMars_Rover -lgnat`
directives that let the Rust side link against the result.

If you want to build it by hand for debugging:

```
cd simulator_interface
alr build
ls lib/libMars_Rover.a
```

## Not the FFI layer

The Rust↔Ada FFI surface — the `#[no_mangle]` callbacks Ada invokes and the
Ada `pragma Import` bodies — lives in:

- Rust side: `crates/mars-rover-ada-link/src/lib.rs`
- Ada side: `src/binding/rover_hal.adb` (+ `src/binding/rover-gnc.adb`)

This directory only builds the library; it does not define the FFI contract.

## `test/`

`test/` contains a small C-based regression harness that was used early in the
Ada-side FFI work, before the Rust workspace had enough surface to drive the
library directly. It is preserved for reference; the live regression gate is
now `crates/mars-rover-replay` + the fixtures under `simulator/tests/fixtures/`.
