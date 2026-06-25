# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Mars Rover demo combining formally-verified Ada/SPARK control software with a 3D Rust/Bevy simulator. The Ada code targets the Raspberry Pi Pico (RP2040) for real hardware, while the simulator runs on desktop via FFI.

## Commands

### Running the Simulator
```bash
cd simulator
cargo run --release
```
The Rust build script (`simulator/build.rs`) automatically calls `alr build` in `simulator_interface/` to compile the Ada library before linking.

### Building the Embedded Ada Target
```bash
alr build          # builds for RP2040 via Alire
```

### Running Tests (Rust Simulator)
```bash
cd simulator
cargo test
```

### SPARK Formal Verification
```bash
# Run GNATprove with project configuration (--no-subprojects --level=1)
alr exec -- gnatprove -P mars_rover.gpr
```
Or use the `/gnatprove` skill for interactive proof work.

### Building the Simulator Interface Test Harness
```bash
cd simulator_interface/test
make main
```

## Architecture

There are three separate build roots:

| Directory | Language | Purpose |
|---|---|---|
| `/` (root) | Ada/SPARK | Embedded firmware for RP2040 — main entry `src/mars_rover.adb` |
| `simulator_interface/` | Ada | Builds the same Ada source as a static library (`libMars_Rover.a`) for linking into the simulator |
| `simulator/` | Rust | Desktop simulator; `build.rs` triggers `alr build` in `simulator_interface/` then links the result |

### Ada Control Software (`src/`)

- **`rover_hal.ads`** — Hardware Abstraction Layer. Defines abstract state (`HW_Init`, `Power_State`, `Turn_State`, `Distance_State`) and all hardware interfaces (clock, sonar, motors, remote input, display). Two implementations exist: `src/device/` for real hardware, `src/binding/` for the simulator.
- **`rover.ads/adb`** — Core rover logic with SPARK safety contracts. Key property: `Cannot_Crash` ghost function guarantees sonar will detect obstacles before the rover reaches them (`Safety_Distance = 20`).
- **`rover-autonomous.adb`** — Obstacle-avoidance loop using sonar.
- **`rover-remote_controlled.adb`** — Manual gamepad control; auto-returns to autonomous after ~10s of inactivity.
- **`rover-mast_control.ads/adb`** — Rotatable distance sensor (-60° to +60°).

### Rust Simulator (`simulator/src/`)

Four Bevy plugins:

- **`domain.rs`** — Pure domain model: `Robot` (4WIS4WID kinematics, wheel state, distance sensor), `Environment` (obstacles, distance queries), `Collision` detection.
- **`simulator.rs`** — Advances robot physics each frame; blocks moves that would cause collisions; updates sonar distance.
- **`controller.rs`** — Spawns Ada control software in a background thread via C ABI FFI (`Mars_Roverinit`, `Mars_Roverfinal`, `mars_rover_demo_task`). Thread-safe exchange via atomics: sonar distance/angle in, wheel angles/power out.
- **`visualizer.rs`** — 3D rendering; drag-and-drop rover/obstacles; keyboard shortcuts (arrow keys = manual override, `R` = reset, `T` = toggle status text).

### FFI Boundary

The Ada HAL binding (`src/binding/rover_hal.adb`) reads/writes the same atomics that `controller.rs` exposes. `mars_rover_clock()` is provided by the Rust side so Ada timing works in simulation.

## Key Constraints

- The root `mars_rover.gpr` targets RP2040 (Pico BSP) — do not use it for the simulator build. The simulator uses `simulator_interface/mars_rover.gpr` which builds a platform-neutral static library.
- SPARK proof level is 1; ghost variables and loop invariants in `rover.ads` must be maintained when changing safety-critical logic.
- Rust toolchain is pinned to 1.77.2 via `simulator/rust-toolchain.toml`.
- Requires GNAT Pro 25+ and Alire configured with the GNAT Pro toolchain.
