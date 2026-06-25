# Mars Rover Demo

This branch holds the code described in
[our blog post](https://github.com/AdaCore/mars-rover-demo/tree/mars-rover-gnc)
that presents the work. Note that the Rover hardware lacks the sensors needed
to execute this code on-target in a meaningful way.

A small 4WIS4WID rover driven by formally-verified Ada/SPARK control software.
The same Ada sources run on a Raspberry Pi Pico (RP2040) with real motors and
sensors, and inside a 3D Rust/Bevy simulator on the desktop. A headless driver
and a pair of replay tools round the picture out so the control software can be
exercised, recorded, and regression-tested without a display.

```
                 ┌──────────────────────────────────────────┐
                 │      Ada / SPARK control software        │
                 │  Rover.GNC · Estimation · Path_Following │
                 │       Autonomous · Remote_Controlled     │
                 │              Rover_HAL                   │
                 └──────────────────────────────────────────┘
                         ▲                     ▲
           binding/      │                     │   device/
           (desktop)     │                     │   (RP2040)
                         │                     │
         ┌───────────────┴───────┐             │
         │   libMars_Rover.a     │             │
         │  (simulator_interface)│             │
         └───────────────────────┘             │
                 ▲                             │
                 │ linked via                  │
                 │ mars-rover-ada-link         │
                 │                             │
  ┌──────────────┴───────────┐         ┌───────┴──────────┐
  │   mars-rover (GUI)       │         │  RP2040 hardware │
  │   mars-rover-headless    │         │  (Pico BSP)      │
  │   mars-rover-replay      │         └──────────────────┘
  └──────────────────────────┘
```

## Jump to a workflow

- [Run the interactive GUI](#workflow-1--interactive-gui) — `cargo run -p mars-rover --release`
- [Tune GNC / EKF with headless + GUI replay](#workflow-2--tune-gnc--ekf-headless-recording--gui-replay) — record a trace, play it back visually
- [Deterministic headless runs & automation](#workflow-3--deterministic-headless-automation) — byte-identical CSVs for CI / batch tuning
- [FFI regression testing](#workflow-4--ffi-regression-testing) — byte-compare Ada outputs against golden traces
- [Build for the RP2040](#workflow-5--flash-real-hardware) — same Ada, real motors and sonar
- [Run SPARK proofs](#workflow-6--run-the-spark-proofs) — `gnatprove` on the verified control software

## What this repository contains

The repository is deliberately mixed-language. Ada lives at the top level (`src/`)
for historical reasons — everything Rust is either the GUI binary (`simulator/`)
or a workspace crate under `crates/`. A second Ada build root
(`simulator_interface/`) compiles the same Ada sources as a desktop static
library so Rust can link them. The root `mars_rover.gpr` instead targets the
RP2040 via the Pico BSP and cannot be linked into a desktop binary.

```
mars-rover-demo/
├── src/                   Ada / SPARK control software (see src/README.md)
│   ├── binding/           HAL implementation for the simulator
│   └── device/            HAL implementation for RP2040
├── simulator_interface/   Ada build root → libMars_Rover.a (desktop)
├── mars_rover.gpr         Ada build root → RP2040 executable
├── crates/
│   ├── mars-rover-core       pure-Rust domain model, sim step, sidecar
│   ├── mars-rover-ada-link   libMars_Rover.a linkage + FFI atomics
│   ├── mars-rover-headless   Bevy-free driver (CSV logger, lockstep)
│   └── mars-rover-replay     FFI-level regression harness
├── simulator/             Bevy GUI binary (package name: `mars-rover`)
└── bin/                   Pre-built RP2040 ELF (artefact only)
```

## Workflows

The interesting thing about this demo isn't any single entry point — it's that
the same Ada software can be driven five different ways, each with a different
tradeoff between realism, determinism, and speed. The workflows below are
arranged from "nicest to show off" to "most useful when you're debugging a
Kalman filter at 2 a.m.".

### Workflow 1 — interactive GUI

> "Show me the rover moving around, let me drag it and toss rocks in its path."

```bash
cargo run -p mars-rover --release
cargo run -p mars-rover --release -- --waypoints path.wpts
cargo run -p mars-rover --release -- --text HELLO
```

Bevy window, terrain mesh, the rover, a rotating sonar mast, draggable
obstacles, keyboard override, an EKF-estimate ghost marker, and a TRON-style
trail of where the rover has been. Ada runs in a background thread; sensors and
actuators are exchanged through atomics in `mars-rover-ada-link`. Every
on-screen update has a physical interpretation — the yellow estimate marker
drifting behind the real rover is the EKF working against slip on the heightmap.

The GUI is where the demo *looks* like a demo. But because each frame is 16 ms
of wall time with a real GPU in the loop, the GUI is the least friendly
environment for studying numerical behaviour — which is what motivates the
next workflow.

### Workflow 2 — tune GNC / EKF (headless recording → GUI replay)

> "Rover veered off the path at t=14s. Why?"

The headless driver and the GUI share a CSV schema (see
`crates/mars-rover-headless/README.md`). The headless driver is faster than
real time (unless instructed otherwise with `--wall-clock`). A run of

```bash
cargo run -p mars-rover-headless --release -- \
    --seed 42 --noise-seed 7 \
    --waypoints path.wpts \
    --duration-sim-seconds 30 \
    --log /tmp/drift.csv
```

records 24 columns per simulator step — truth pose, EKF estimate, steering,
power, encoder totals, GPS fixes, sonar, IMU gyro, mast angle, and Ada's own
step timestamp. The logger also writes two sidecars next to the CSV:
`drift.csv.meta` (terrain seed + GPS interval + noise seed + slip-free flag)
and `drift.csv.wpts` (the waypoint list).

Feed that trace back into the GUI with `--replay`:

```bash
cargo run -p mars-rover --release -- --replay /tmp/drift.csv
```

The GUI rebuilds the same terrain from the `.meta` sidecar's seed, renders the
logged truth pose, and pushes the logged EKF estimate through the visualiser's
existing ghost-marker pipeline. No Ada thread is spawned; no physics is
simulated; the render is purely data-driven. Keyboard `R` restarts playback.

This lets you:

- Record one slow, careful run and iterate visually on it dozens of times
  without rerunning the sim.
- Record on a headless box (CI, a cloud VM, a nightly job) and replay on a
  workstation with a GPU.
- Watch the EKF estimate marker diverge from truth in conditions you can
  reproduce exactly by re-using the same `--seed` + `--noise-seed`.

This is the workflow the GNC and EKF tuning lived in. The narrative loop is:
tweak Ada contracts or noise parameters → headless record → GUI replay →
notice the estimate ribbon curling away from truth at a specific obstacle →
tweak again. Orders of magnitude faster than live debugging.

### Workflow 3 — deterministic headless automation

> "I want two runs of the same scenario to produce byte-identical CSVs."

Headless has a `--lockstep` mode that inserts an explicit barrier between the
Rust sim thread and Ada's control loop at every Ada cycle. Combined with fixed
`--seed` and `--noise-seed` this removes every source of non-determinism the
GUI exposes:

```bash
cargo run -p mars-rover-headless --release -- \
    --seed 42 --noise-seed 7 --lockstep \
    --duration-sim-seconds 5 --log a.csv

cargo run -p mars-rover-headless --release -- \
    --seed 42 --noise-seed 7 --lockstep \
    --duration-sim-seconds 5 --log b.csv

diff a.csv b.csv   # empty
```

Virtual time replaces wall-clock time, so the run is also much faster than
real time. Useful for batch EKF tuning, bisecting regressions, and as a
determinism gate in CI.

Headless drops the movable rocks that the GUI lets you drag — without a
renderer there's no way to clear one that happens to sit on the path. The
perimeter walls remain.

### Workflow 4 — FFI regression testing

> "Did my last Ada change alter the actuator outputs for any of our canonical
> scenarios?"

`mars-rover-replay` is a separate Rust binary (not to be confused with the
GUI's `--replay` flag). It reads a CSV trace, *drives sensor values through
ada-link* at each Ada step, steps Ada forward one control cycle via a gate
hook, reads Ada's actuator outputs, and compares them to the logged values
within a numeric tolerance.

```bash
cargo build --release -p mars-rover-replay
cd simulator && ./scripts/test-replay-fixtures.sh
```

The fixtures in `simulator/tests/fixtures/` are short synthetic CSVs (≤ 50
rows each) paired with golden stdout files; the script diffs the replay
binary's stdout against the golden and fails on any drift. This is the
regression gate for the Ada control software plus the FFI atomic surface.
When Ada behaviour changes intentionally, regenerate the goldens with
`./scripts/test-replay-fixtures.sh --regenerate` and commit them.

The GUI's `--replay` flag (Workflow 2) is a *renderer* — it visualises a
logged trajectory. `mars-rover-replay` is a *verifier* — it re-drives Ada
against the same inputs and checks the outputs match. They share the CSV
format but serve different purposes.

### Workflow 5 — flash real hardware

> "Same Ada, no simulator, real motors."

***IMPORTANT***: The real hardware does not have the sensors needed for GNC.

```bash
alr build                # at the repo root; targets RP2040 via Pico BSP
```

The root `mars_rover.gpr` selects `src/` + `src/device/` (the hardware HAL
implementation) and cross-compiles for the Pico. The resulting executable is
at `bin/mars_rover`. Flash it to a Pico with the usual `picotool` / BOOTSEL
dance. On hardware, the EKF, waypoint, and GPS paths are stubbed out (the HAL
returns 0 for GPS, IMU, waypoints) — the rover falls through to the
remote-controlled / autonomous modes driven by the sonar and gamepad. Adding
a real GPS / IMU is a matter of filling in the device HAL.

### Workflow 6 — run the SPARK proofs

> "Show me the formal verification."

```bash
alr exec -- gnatprove -P mars_rover.gpr
```

The project targets proof level 1 overall; `Rover.Estimation` and `Rover.GNC`
carry contracts at Silver. `src/proof-status.md` and `src/campaign-journal.md`
are the ongoing work-products of the proof campaigns. See
[`src/README.md`](src/README.md) for the package tour and the current proof
state.

## Component reference

| Component | Role |
|---|---|
| [`src/`](src/README.md) | Ada/SPARK control software. Modes (Autonomous, Remote_Controlled, GNC), EKF (Estimation), path follower, mast control, task dispatch, HAL spec. Two HAL bodies: `binding/` for desktop, `device/` for RP2040. |
| [`simulator_interface/`](simulator_interface/README.md) | Second Ada build root. Compiles `src/` + `src/binding/` as `libMars_Rover.a` for desktop linking. Invoked by `mars-rover-ada-link/build.rs`. |
| [`crates/mars-rover-core`](crates/mars-rover-core/README.md) | Pure-Rust domain model: robot kinematics, environment, terrain, collision, sim step, CSV sidecar writer, glyph→waypoints. No Bevy. No Ada linkage. |
| [`crates/mars-rover-ada-link`](crates/mars-rover-ada-link/README.md) | Owns the `libMars_Rover.a` link and every `#[no_mangle]` callback Ada invokes. Public accessors for sensor/actuator atomics. |
| [`crates/mars-rover-headless`](crates/mars-rover-headless/README.md) | Bevy-free driver. Fixed-step main loop, virtual-time + optional lockstep barrier, CSV logger, determinism gate. |
| [`crates/mars-rover-replay`](crates/mars-rover-replay/README.md) | FFI-level regression harness. Injects sensors, step-gates Ada, compares actuator outputs to golden traces. |
| [`simulator/`](simulator/README.md) | Bevy GUI (Cargo package name `mars-rover`). Live sim + `--replay` trace renderer. Keybinds, drag-drop, camera, ribbon, overlays. |

## Prerequisites

- GNAT Pro 25 or newer, configured via Alire.
- Alire (`alr`) on the `PATH`.
- Rust 1.77.2 (pinned in `simulator/rust-toolchain.toml`).
- For the GUI: the Bevy Linux dependencies
  ([list](https://github.com/bevyengine/bevy/blob/release-0.13.2/docs/linux_dependencies.md)).
- `git submodule update --init --recursive` before first build.

The Rust build script in `mars-rover-ada-link` invokes `alr build` in
`simulator_interface/` and emits the `-L` / `-l` directives automatically —
there's no separate "build Ada first" step for the Rust side.
