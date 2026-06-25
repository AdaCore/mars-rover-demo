# `mars-rover` — Bevy GUI

The interactive 3D simulator. Cargo package name is `mars-rover`; the directory
is kept as `simulator/` for historical reasons. For the workspace overview see
the [repository root README](../README.md).

Two modes, dispatched by CLI:

- **Live** — spawns the Ada control thread, steps physics every frame, lets
  you drag the rover and obstacles, optionally records a CSV trace.
- **Replay** — no Ada thread, no physics; renders a trace recorded by either
  the live GUI or `mars-rover-headless`. The terrain seed and waypoints come
  from sidecar files the original run wrote next to the CSV.

## Running

```bash
cargo run -p mars-rover --release                                         # live
cargo run -p mars-rover --release -- --waypoints path.wpts                # live, with path
cargo run -p mars-rover --release -- --text HELLO                         # live, glyph path
cargo run -p mars-rover --release -- --log /tmp/drift.csv                 # live, recording
cargo run -p mars-rover --release -- --replay /tmp/drift.csv              # replay a recording
cargo run -p mars-rover --release -- --help                               # full flag list
```

## CLI

| Flag | Description |
|---|---|
| `--seed <u64>` | Terrain generation seed. Default 42. |
| `--gps-interval <u32>` | GPS update interval in ms; 0 disables GPS. Default 500. |
| `--waypoints <path>` | Waypoint file (`x y` or `x,y` per line; `#` comments). |
| `--text <string>` | Generate waypoints that trace the given text (glyph font). |
| `--log <path>` | Write per-frame CSV to `<path>`; also writes `<path>.meta` and `<path>.wpts`. |
| `--replay <path>` | Render a recorded trace instead of running a live sim. |

In replay mode, CLI `--seed` / `--waypoints` / `--text` are ignored — the
trajectory is reconstructed entirely from the CSV and its sidecars, so the
overlay reflects what the original run was following. A warning is printed if
the `.meta` sidecar is missing (terrain may not match the recording).

## In-simulator controls

| Key / input | Action | In replay |
|---|---|---|
| Arrow keys | Manual wheel override (velocity + steering) | disabled |
| Home / End | Manual sonar mast angle | disabled |
| `R` | Reset rover position (live) / rewind trace (replay) | restart |
| `F` | Toggle slip-free mode (ideal traction) | disabled |
| `T` | Toggle rover status text overlay | active |
| `P` | Toggle waypoint path overlay | active |
| `L` | Toggle TRON ribbon trail | active |
| `C` | Toggle Ada console pane | active (blank) |
| Drag rover / rock | Move the entity (live only) | disabled |
| Scroll wheel | Zoom camera | active |

If you drive with the arrow keys, the Ada control software yields for a few
seconds before taking control back — this matches how a real remote-controlled
session would preempt the autonomous loop.

## Recording traces

`--log <path>` activates the per-frame CSV logger. The 24-column schema is
shared with `mars-rover-headless`; it is the same format `mars-rover-replay`
consumes, so any trace recorded here can also be used as a regression input.
The logger also writes:

- `<path>.meta` — `seed`, `gps_interval_ms`, `noise_seed`, `slip_free` (key=value).
- `<path>.wpts` — the waypoints, one per line.

These sidecars let the GUI `--replay` mode reconstruct the terrain and overlay
without the user having to remember the original flags. The sidecar writer
lives in `mars-rover-core::sidecar` and is shared with the headless logger.

## What is *not* here

- The `#[no_mangle]` FFI callbacks Ada invokes — those live in
  `mars-rover-ada-link`.
- Domain model (robot kinematics, environment, terrain, collision, sim step) —
  `mars-rover-core`.
- The CSV logger details and sidecar format — `mars-rover-core` and
  `mars-rover-headless` for the format contract.

## Replay mode internals (if you're modifying it)

- Source code: `src/replay_source.rs` (trait + `FileTraceSource`), `src/replay_mode.rs` (Bevy plugin).
- Trace rows populate the existing `RobotRes` and push logged EKF estimates
  through `ada_link::set_ekf_estimate` so the visualizer's `draw_ekf_marker`
  system needs no changes.
- `CORNER_WHEELS = [FL, FR, RL, RR]` in `replay_mode.rs` pins the CSV steering
  column order independent of `WheelID`'s discriminant layout. See the comment
  in that file; do not "simplify" it to `WheelID as usize`.
- Drag-drop handlers in `visualizer.rs` early-return when `ReplayActive.0` is
  true.
