# `mars-rover-core`

Pure-Rust shared substrate for every driver of the Mars Rover simulator. No
Bevy, no `libudev`, no Ada linkage — which is why `cargo test -p
mars-rover-core` runs cleanly anywhere with a Rust toolchain. For the
workspace overview see the [repository root README](../../README.md).

## What lives here

| Module | Purpose |
|---|---|
| `domain::robot` | 4WIS4WID kinematics, wheel state, distance sensor model |
| `domain::environment` | Obstacles + distance-to-next-obstacle queries |
| `domain::terrain` | Seeded noise heightmap; bilinear interpolation; slope/traction |
| `domain::collision` | Broad-phase + narrow-phase collision detection |
| `sim` | Single simulator step: advance kinematics, resolve collisions, update sensors. `SimInput` queue + `SimInputs` drain helper. |
| `world` | `World::build(seed)` — factory for (robot, environment, terrain) tuples with canonical starting pose and perimeter walls. |
| `sidecar` | Shared writer/reader for `*.meta` + `*.wpts` sidecar files (single source of truth used by both GUI and headless loggers). |
| `glyph` | Text → waypoint path rasteriser (`--text HELLO` path in the drivers). |

## Why this separation exists

Before the workspace split, all the code here was tangled with Bevy resources
and the Ada link in one `simulator/` crate. Splitting it out has three
practical consequences:

- `cargo test -p mars-rover-core` links no Ada library and pulls in no
  window/GPU deps — it runs on any headless build box.
- The domain model has no `Res<>` / `ResMut<>` signatures, so swapping the
  driver (GUI, headless, a future socket-streaming viewer) doesn't need any
  changes here.
- The `sidecar` module is the only writer of `.meta` + `.wpts` on disk.
  GUI and headless both call it, so the file formats cannot drift between
  them.

## `sim::SimInput` control plane

Drivers that want to push state changes into the sim (reset rover, load new
waypoints, toggle slip-free, request an EKF reset) do so by pushing
`SimInput` variants into a `SimInputs` queue; `sim::apply` drains it at the
top of each step. The GUI's drag-drop handlers push here; the headless
driver's queue is present but empty (a future network viewer is the intended
user).

## Testing

```
cargo test -p mars-rover-core
```

59+ tests covering robot kinematics, collision, terrain interpolation, world
construction, and the glyph path generator. Tests use `insta` for snapshot
comparison and `rstest` for parametric cases. `textplots` is pulled in to let
test failures render quick ASCII plots of diverging trajectories.
