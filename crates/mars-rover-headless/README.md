# `mars-rover-headless`

Bevy-free driver for the Mars Rover Ada control software. Fixed-step
simulation loop, no rendering, virtual-time clock, optional strict
Ada↔sim lockstep, CSV logging. For the workspace overview see the
[repository root README](../../README.md).

## When to use it

- **GNC / EKF tuning** — record a long trace with a fixed noise seed, iterate
  on Ada contracts or filter parameters, re-record, diff. Much faster than
  live-driving the GUI.
- **Determinism gate** — two runs with the same `--seed` / `--noise-seed`
  produce byte-identical CSVs (with `--lockstep`). Useful in CI.
- **Feed the GUI replay** — a headless CSV + its sidecars plays back in the
  GUI (`cargo run -p mars-rover -- --replay trace.csv`), so you can record
  on a headless box and inspect visually on a workstation.
- **Batch scenario runs** — loop over seeds, waypoint files, and GPS
  intervals without the Bevy startup cost or window-manager dependency.

## Running

```
cargo run -p mars-rover-headless --release -- \
    --seed 42 --noise-seed 7 --lockstep \
    --waypoints path.wpts \
    --duration-sim-seconds 30 \
    --log /tmp/drift.csv
```

Writes `drift.csv` + `drift.csv.meta` + `drift.csv.wpts`.

## CLI

| Flag | Description |
|---|---|
| `--seed <u64>` | Terrain seed. Default 42. |
| `--gps-interval <u32>` | GPS update interval in ms; 0 disables. Default 500. |
| `--waypoints <path>` | Waypoint file (`x y` per line; `#` comments). |
| `--text <string>` | Generate waypoints from glyph font (overrides `--waypoints`). |
| `--log <path>` | Write per-step CSV diagnostic log. |
| `--duration-sim-seconds <f64>` | Virtual-time duration. Default 30.0. |
| `--wall-clock` | Skip virtual-time mode; run at real speed. |
| `--lockstep` | Strict Ada↔sim barrier every Ada cycle (determinism). |
| `--noise-seed <u64>` | Seed for the Gaussian sensor-noise RNG. 0 = OS entropy. |
| `--slip-free` | Force ideal traction on every wheel (diagnostic). |

## Obstacles

Headless keeps the perimeter walls built by `mars_rover_core::world::World::build`
but **drops the two movable rocks** the GUI exposes. Without a renderer there
is no way to drag a rock out of the rover's path, and a rock that happens to
land on the route to the first waypoint halts the run. Filtering is done at
startup (`!is_movable`) so the terrain is otherwise identical to what the GUI
would produce with the same seed.

## Lockstep mode

`--lockstep` installs a two-phase barrier via `ada-link`'s delay hook:

1. Ada reaches `Delay_Milliseconds` → increments `done_seq`, blocks.
2. Main thread wakes on `done_seq`, advances sim + time, grants one permit.
3. Ada consumes permit, wakes, runs the next cycle.

The main thread owns `VIRTUAL_CLOCK` exclusively in this mode; Ada's delay
calls become pure rendezvous points with no clock manipulation. Without
`--lockstep`, Ada advances the virtual clock itself via fetch-add and the
main thread yields — simpler but not strictly deterministic.

A permit is granted once every `TICKS_PER_ADA_CYCLE = 4` sim steps (at 10 ms
each), so Ada perceives its expected 40 ms cycle.

## CSV format

One row per sim step (10 ms virtual). 24 columns:

```
time_ms,
truth_x, truth_y, truth_theta,
est_x, est_y, est_theta,
steer_fl, steer_fr, steer_rl, steer_rr,
power_left, power_right,
enc_fl, enc_fr, enc_rl, enc_rr,
gps_x, gps_y, gps_ts,
sonar_dist, imu_gyro_z, mast_angle,
ada_step_us
```

- `est_*` are blank until the EKF initialises on its first GPS fix.
- `gps_*` are blank on frames without a fresh GPS fix.
- `sonar_dist` is `u32::MAX` when no obstacle is in range.
- `enc_*` are cumulative truth ticks (never reset).
- `ada_step_us` is Ada's own wall-clock timestamp at the corresponding
  `Delay_Milliseconds` call (wall-clock mode only; headless runs in virtual
  time and leaves this at 0 — the replay harness's exact-DT path then kicks
  in only for GUI-recorded traces).

The `mars-rover-core::sidecar` module writes `<path>.meta`
(`seed=… gps_interval_ms=… noise_seed=… slip_free=…`) and `<path>.wpts` (one
waypoint per line) alongside the CSV so GUI replay can reconstruct the
scenario without the user remembering the flags.

## Determinism expectation

With `--seed X --noise-seed Y --lockstep`, two runs at the same
`--duration-sim-seconds` produce byte-identical CSVs:

```
cargo run -p mars-rover-headless --release -- \
    --seed 42 --noise-seed 7 --lockstep --duration-sim-seconds 5 --log a.csv
cargo run -p mars-rover-headless --release -- \
    --seed 42 --noise-seed 7 --lockstep --duration-sim-seconds 5 --log b.csv
diff a.csv b.csv   # empty
```

Without `--lockstep` this is not guaranteed — the non-lockstep mode yields
between sim steps to let Ada run, and scheduling jitter can change how many
sim steps fall between two Ada cycles.
