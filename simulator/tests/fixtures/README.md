# Replay regression fixtures

Each `*.csv` is a hand-crafted sensor trace used to drive the Ada control
software via `cargo run --bin replay`.  Each `*.golden` captures the
byte-exact stdout of running replay on its paired CSV.

The regression contract: running `replay fixture.csv` must produce stdout
that matches `fixture.golden` byte-for-byte.  Any divergence means a change
to the Ada control software, the FFI surface, or replay itself has altered
observable behaviour.

These fixtures are deliberately synthetic, not recorded from a GUI run.
A synthetic trace has three advantages over a recorded one:

- **Regeneratable from source** — no binary artefact, no display required.
- **Targeted** — each probe exercises one specific code path.
- **Minimal** — ~10 rows per probe, total suite runs in seconds.

## Probes

| File | Rows | Purpose | Expected Ada behaviour |
|---|---|---|---|
| `stationary.csv` | 50 (2 s) | All sensors zero, rover motionless, no obstacle | Enter autonomous mode; continue mast-scan sweep |
| `obstacle_close.csv` | 50 (2 s) | Sonar permanently reports obstacle at 15 cm | Autonomous avoidance eventually diverges from stationary |
| `waypoint_single.csv` + `.wpts` | 12 (480 ms) | Periodic GPS fixes + one waypoint at (0.5, 0.0) | EKF initialises from GPS; GNC commands forward motion |

Encoder-only traces without GPS or waypoints produce behaviour identical to
`stationary.csv`: autonomous mode does not read encoders, and the EKF does not
initialise without a GPS fix.  A standalone `encoders_straight` probe was
therefore redundant and has been omitted.

## CSV format

24 columns (see `simulator/src/bin/replay.rs` header comment for full spec).
Blank fields are permitted for `est_x/y/theta` (EKF not yet initialised) and
`gps_x/y/ts` (frames with no new GPS fix).  `sonar_dist = 4294967295` means
no obstacle.  `ada_step_us` must be monotonically non-decreasing; replay
uses it to identify step boundaries.

## Running the regression gate

From `simulator/`:

```
cargo build --release --bin replay
./scripts/test-replay-fixtures.sh
```

The script diffs each fixture's replay stdout against its committed golden
and exits non-zero on any drift.

## Regenerating goldens

When Ada behaviour changes intentionally:

```
./scripts/test-replay-fixtures.sh --regenerate
```

Review the diff against committed goldens (`git diff tests/fixtures/`) and
commit if the change is intentional.
