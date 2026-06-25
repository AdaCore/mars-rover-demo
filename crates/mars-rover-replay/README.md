# `mars-rover-replay`

FFI-level regression harness for the Ada control software. Reads a CSV trace
recorded by the GUI or headless driver, injects each frame's sensor values
through `mars-rover-ada-link`, gates Ada forward one control cycle at a time,
and byte-compares Ada's actuator outputs against the logged expected values.
For the workspace overview see the [repository root README](../../README.md).

## Not to be confused with

- `mars-rover`'s `--replay <csv>` flag — that is a **renderer** that
  visualises a trace in the Bevy GUI (no Ada thread). See `simulator/README.md`.
- `mars-rover-headless` — that *drives* a simulation to produce a CSV trace.

This crate consumes a trace and verifies Ada's behaviour is stable against it.

## Running

```
# Build the binary
cargo build --release -p mars-rover-replay

# Run against a single trace
cargo run --release -p mars-rover-replay -- trace.csv [tolerance]

# Run the committed fixture suite (preferred regression gate)
cd simulator && ./scripts/test-replay-fixtures.sh
```

`tolerance` is a single integer applied to both the power output (±N out of
127) and steering / mast angle (±N degrees). Default 5.

Exit code: 0 if all steps pass, 1 if any step fails.

## How it works

1. Parse the CSV (24-column schema; see `crates/mars-rover-headless/README.md`).
2. Auto-load the `<path>.wpts` sidecar if present. Without waypoints Ada falls
   into autonomous mode, which has completely different behaviour from GNC
   path-following — the mismatch would look like a regression but isn't one.
3. Enable virtual-time mode and install a delay-hook step gate on `ada-link`.
   Each Ada `Delay_Milliseconds` call becomes a barrier:

   ```
   Ada enters delay   → done_seq++, notify harness
   Ada waits           ← harness grants permit
   Ada wakes           → clock advances to logged ada_step_us, cycle runs
   ```

4. Before granting each permit, the harness:
   - Computes the window of CSV rows that cover this Ada cycle
     (first row where `ada_step_us` advances past the current).
   - Injects encoder deltas from the *first* row of the window (what Ada will
     read when it wakes).
   - Injects the IMU gyro from the *last* row of the window (what Bevy would
     have last written before Ada woke).
   - Scans the window for GPS fixes and injects the last one found.
5. After the cycle, reads Ada's actuator outputs and compares them against
   the **first row of the next window** (Bevy's PostUpdate stage in the
   original run captured outputs a frame after Ada's delay).

The expected-row offset, the IMU-last-row choice, and the GPS-window scan
are all load-bearing. Earlier iterations had them wrong and produced
apparently-failing replays; see the "Residual divergence" discussion in the
diary entries.

## Tolerance and residual divergence

Long traces (minutes) recorded from live GUI runs carry unavoidable sub-pixel
timing jitter that translates to a stable ~0.3° heading bias in the EKF. The
bias is a fixed offset — not growing drift — but it can push mode-transition
thresholds across by a single cycle (~2% of steps). Interpret the outputs as
follows:

| Pattern | Meaning |
|---|---|
| 100% pass | Perfectly deterministic trace (lockstep-recorded, or short). |
| ≤ 3% FAIL that recovers within 1–2 steps | Timing-boundary artefact. Not a regression. |
| New sustained FAILs, growing EKF δ, big pass-rate drop | Real regression. |

For tightest coverage, regress against short purpose-built traces (≤ 30 s)
where the 0.3° offset has less opportunity to shift anything.

## Fixture suite

`simulator/tests/fixtures/` contains the canonical regression scenarios.
Each `*.csv` is paired with a `*.golden` capturing the exact expected
stdout. The shell driver byte-compares and fails on any drift:

```
cd simulator
cargo build --release -p mars-rover-replay
./scripts/test-replay-fixtures.sh
```

The fixtures are deliberately synthetic (≤ 50 rows each) — targeted,
regeneratable from source, and fast enough to run in seconds. See
`simulator/tests/fixtures/README.md` for the probe list.

When Ada behaviour changes *intentionally*:

```
./scripts/test-replay-fixtures.sh --regenerate
git diff simulator/tests/fixtures/     # sanity-check the new goldens
git add simulator/tests/fixtures/*.golden
```
