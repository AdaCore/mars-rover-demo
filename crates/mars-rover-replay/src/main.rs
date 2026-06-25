//! Replay harness for the Mars Rover Ada control software.
//!
//! Reads a CSV trace recorded by the live simulator (`--log <path>`), replays
//! the sensor sequence through the Ada control thread in virtual-time mode, and
//! checks that Ada's actuator outputs match the logged values within a numeric
//! tolerance.
//!
//! # Usage
//!
//! ```text
//! cargo run --bin replay -- <trace.csv> [tolerance]
//! ```
//!
//! `tolerance` is a single integer that sets both the power tolerance (±N out of
//! 127) and the steering/mast tolerance (±N degrees).  Default: 5.
//!
//! Exit code: 0 if all steps pass, 1 if any step fails.
//!
//! # Architecture
//!
//! All FFI atomics and `#[no_mangle]` callbacks live in `mars-rover-ada-link`;
//! this binary drives them through ada-link's public API and installs a delay
//! hook that turns each Ada `Delay_Milliseconds` call into the step-gate
//! barrier below.
//!
//! ## Step-gate synchronisation
//!
//! Ada's control cycle ends with `Delay_Milliseconds(40)`.  The delay hook
//! turns each delay call into a two-phase barrier:
//!
//! 1. **Ada enters delay** → increments `done_seq`, notifies harness.
//! 2. **Ada waits** for the harness to grant a permit.
//! 3. **Harness wakes** on `done_seq`; reads Ada's outputs (stable — Ada is
//!    blocked); injects sensor values for the next step; grants the permit.
//! 4. **Ada wakes** → advances virtual clock → returns from delay → runs
//!    the next control cycle.

use std::sync::{atomic::{AtomicU64, Ordering}, Condvar, Mutex, OnceLock};

use mars_rover_ada_link as ada_link;

// ── Step gate ─────────────────────────────────────────────────────────────────

struct GateState {
    /// Permits available for Ada to consume.  Each permit lets Ada run one delay.
    permits: u32,
    /// Incremented each time Ada enters a delay (just before blocking).
    /// The harness waits on this to know Ada has written stable outputs.
    done_seq: u64,
}

/// The VIRTUAL_CLOCK value the harness wants Ada to advance to when it wakes
/// from the next delay.  The harness sets this (from the logged `ada_step_us`
/// column) before granting a permit; the delay hook stores it via
/// `ada_link::set_virtual_clock`.  Zero means "use fetch_add fallback"
/// (first step / old-format CSV).
static GATE_NEXT_CLOCK_US: AtomicU64 = AtomicU64::new(0);

/// (Mutex<GateState>, cv_ada, cv_done)
/// cv_ada:  Ada waits here for permits > 0.
/// cv_done: Harness waits here for done_seq to change.
static STEP_GATE: OnceLock<(Mutex<GateState>, Condvar, Condvar)> = OnceLock::new();

fn step_gate() -> &'static (Mutex<GateState>, Condvar, Condvar) {
    STEP_GATE.get_or_init(|| {
        (
            Mutex::new(GateState { permits: 0, done_seq: 0 }),
            Condvar::new(),
            Condvar::new(),
        )
    })
}

// ── CSV parsing ───────────────────────────────────────────────────────────────
//
// Column layout (matches logger.rs HEADER, 24 columns, 0-based):
//
//  0          time_ms
//  1–3        truth_x / truth_y / truth_theta  (ignored)
//  4–6        est_x / est_y / est_theta         (optional: blank until EKF ready)
//  7–10       steer_fl / steer_fr / steer_rl / steer_rr  (degrees, f64)
//  11–12      power_left / power_right          (i8)
//  13–16      enc_fl / enc_fr / enc_rl / enc_rr (cumulative truth ticks)
//  17–19      gps_x / gps_y / gps_ts            (optional: blank on non-fix frames)
//  20         sonar_dist                         (u32; u32::MAX = no obstacle)
//  21         imu_gyro_z                         (i16)
//  22         mast_angle                         (i8)
//  23         ada_step_us                        (u64; µs since epoch; 0 in old logs)

#[derive(Debug)]
struct CsvRow {
    time_ms: u64,
    // cols 1-3: truth x/y/theta  (ignored for comparison)
    // cols 4-6: logged EKF estimate — blank until filter initialises
    log_est_x:     Option<f64>,
    log_est_y:     Option<f64>,
    log_est_theta: Option<f64>,
    steer_fl: f64,
    steer_fr: f64,
    steer_rl: f64,
    steer_rr: f64,
    power_left: i8,
    power_right: i8,
    enc_fl: i32,
    enc_fr: i32,
    enc_rl: i32,
    enc_rr: i32,
    gps_x: Option<f64>,
    gps_y: Option<f64>,
    gps_ts: Option<u32>,
    sonar_dist: u32,
    imu_gyro_z: i16,
    mast_angle: i8,
    /// Ada's wall-clock time (µs since EPOCH) at the `Delay_Milliseconds` call that
    /// ended the step covering this Bevy frame.  Zero in logs recorded before this
    /// column was added; in that case the harness falls back to fixed 40 ms advances.
    ada_step_us: u64,
}

// ── Waypoint sidecar loading ──────────────────────────────────────────────────

/// Load waypoints from a file in "x y" (or "x,y") format, one per line.
/// Lines starting with '#' are treated as comments and skipped.
fn load_waypoints(path: &str) -> Vec<[f32; 2]> {
    let content = match std::fs::read_to_string(path) {
        Ok(s) => s,
        Err(_) => return Vec::new(),
    };
    let mut wps = Vec::new();
    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let parts: Vec<&str> = if line.contains(',') {
            line.splitn(2, ',').collect()
        } else {
            line.splitn(2, ' ').collect()
        };
        if parts.len() == 2 {
            if let (Ok(x), Ok(y)) = (
                parts[0].trim().parse::<f32>(),
                parts[1].trim().parse::<f32>(),
            ) {
                wps.push([x, y]);
            }
        }
    }
    wps
}

fn parse_opt<T: std::str::FromStr>(s: &str) -> Option<T> {
    if s.is_empty() { None } else { s.parse().ok() }
}

fn parse_csv(path: &str) -> Vec<CsvRow> {
    let content = std::fs::read_to_string(path)
        .unwrap_or_else(|e| { eprintln!("error: cannot read {path}: {e}"); std::process::exit(2); });

    let mut rows = Vec::new();
    for (lineno, line) in content.lines().enumerate() {
        if lineno == 0 { continue; } // skip header
        let c: Vec<&str> = line.splitn(25, ',').collect();
        if c.len() < 23 {
            eprintln!("warning: line {}: expected ≥23 columns, got {}; skipping", lineno + 1, c.len());
            continue;
        }
        rows.push(CsvRow {
            time_ms:       c[0].parse().unwrap_or(0),
            log_est_x:     parse_opt(c[4]),
            log_est_y:     parse_opt(c[5]),
            log_est_theta: parse_opt(c[6]),
            steer_fl:      c[7].parse().unwrap_or(0.0),
            steer_fr:      c[8].parse().unwrap_or(0.0),
            steer_rl:      c[9].parse().unwrap_or(0.0),
            steer_rr:      c[10].parse().unwrap_or(0.0),
            power_left:    c[11].parse().unwrap_or(0),
            power_right:   c[12].parse().unwrap_or(0),
            enc_fl:        c[13].parse().unwrap_or(0),
            enc_fr:        c[14].parse().unwrap_or(0),
            enc_rl:        c[15].parse().unwrap_or(0),
            enc_rr:        c[16].parse().unwrap_or(0),
            gps_x:         parse_opt(c[17]),
            gps_y:         parse_opt(c[18]),
            gps_ts:        parse_opt(c[19]),
            sonar_dist:    c[20].parse().unwrap_or(u32::MAX),
            imu_gyro_z:    c[21].parse().unwrap_or(0),
            mast_angle:    c[22].parse().unwrap_or(0),
            ada_step_us:   c.get(23).and_then(|s| s.parse().ok()).unwrap_or(0),
        });
    }
    rows
}

// ── Sensor injection ──────────────────────────────────────────────────────────

/// Push a row's non-GPS sensor values through ada-link.
///
/// * `enc_row`  — row used for encoder deltas (first row of the window, i.e.
///                the cumulative totals at the moment Ada last read encoders).
/// * `imu_row`  — row used for the gyro-Z reading.  Ada reads the IMU
///                immediately after waking from its delay, so it sees whatever
///                Bevy last wrote — the final frame of the preceding window, not
///                the first.  Pass `rows[step_end - 1]` for correctness.
/// * `enc_base` — cumulative encoder totals at Ada's previous read boundary.
///
/// GPS is handled separately by `inject_gps_window` so that the last fix
/// within the upcoming window reaches Ada before it wakes — matching the
/// original run where Bevy set GPS fixes during Ada's sleep.
///
/// NOTE: the virtual clock is NOT updated here.  The clock is advanced
/// exclusively by the delay hook so Ada's DT computation is exact.
fn inject_sensors(enc_row: &CsvRow, imu_row: &CsvRow, enc_base: &[i32; 4]) {
    ada_link::set_sonar_distance(enc_row.sonar_dist);

    // IMU: use the last frame of the window.  In the original run, Bevy updates
    // the IMU atomic every frame during Ada's sleep; Ada reads it when it wakes,
    // so it sees the final value written before it woke up.
    ada_link::set_imu_gyro_z_raw(imu_row.imu_gyro_z as i32);

    // Encoder ticks: cumulative delta from the base (Ada's last read boundary).
    let enc = [enc_row.enc_fl, enc_row.enc_fr, enc_row.enc_rl, enc_row.enc_rr];
    for (i, &curr) in enc.iter().enumerate() {
        // set_encoder_tick_delta uses `store` (not fetch_add): the harness owns
        // the encoder state between steps.
        ada_link::set_encoder_tick_delta(i, curr - enc_base[i]);
    }
}

/// Scan `rows[start..end]` and inject the last GPS fix found.
///
/// Called BEFORE granting Ada's permit so that Ada wakes to find fixes that
/// arrived during its sleep — exactly as Bevy would have set them in the
/// original run.
fn inject_gps_window(rows: &[CsvRow], start: usize, end: usize) {
    for r in &rows[start..end] {
        if let (Some(gx), Some(gy), Some(ts)) = (r.gps_x, r.gps_y, r.gps_ts) {
            const SCALE: f64 = 1_000_000.0;
            ada_link::set_gps_fix((gx * SCALE) as i32, (gy * SCALE) as i32, ts);
        }
    }
}

// ── Output comparison ─────────────────────────────────────────────────────────

struct Mismatch {
    field: &'static str,
    actual: f64,
    expected: f64,
    tolerance: f64,
}

fn check_i8(
    field: &'static str,
    actual: i8,
    expected: i8,
    tol: i8,
    out: &mut Vec<Mismatch>,
) {
    if (actual as i16 - expected as i16).unsigned_abs() > tol as u16 {
        out.push(Mismatch {
            field,
            actual: actual as f64,
            expected: expected as f64,
            tolerance: tol as f64,
        });
    }
}

fn check_f64(
    field: &'static str,
    actual: f64,
    expected: f64,
    tol: f64,
    out: &mut Vec<Mismatch>,
) {
    if (actual - expected).abs() > tol {
        out.push(Mismatch { field, actual, expected, tolerance: tol });
    }
}

/// Read Ada's current EKF estimate from the output atomics.
/// Returns `None` when the filter has not yet published an estimate.
fn ada_ekf() -> Option<(f64, f64, f64)> {
    ada_link::ekf_estimated_state_f64()
}

/// Format EKF state "(x, y, θ°)" or "—" if not yet available.
fn fmt_ekf(est: Option<(f64, f64, f64)>) -> String {
    match est {
        None => "—".to_string(),
        Some((x, y, th)) => format!("({:.3},{:.3},{:.1}°)", x, y, th.to_degrees()),
    }
}

/// If the given EKF position is within the capture radius (0.05 m) of the
/// waypoint at `wp_idx`, return `wp_idx + 1`; otherwise return `wp_idx`.
/// Mirrors the single-advance-per-cycle logic in `rover-gnc.adb`.
fn maybe_capture_wp(x: f64, y: f64, wp_idx: usize, wps: &[[f32; 2]]) -> usize {
    if wp_idx >= wps.len() { return wp_idx; }
    let wp = wps[wp_idx];
    let dist = (x - wp[0] as f64).hypot(y - wp[1] as f64);
    if dist < 0.05 { wp_idx + 1 } else { wp_idx }
}

/// Compare Ada's current output atomics against the expected row.
/// Returns a list of fields that fall outside tolerance.
fn compare_outputs(expected: &CsvRow, tol: i8) -> Vec<Mismatch> {
    let stol = tol as f64;
    let mut out = Vec::new();

    let (pl, pr) = ada_link::motor_powers();
    check_i8("power_left",  pl, expected.power_left,  tol, &mut out);
    check_i8("power_right", pr, expected.power_right, tol, &mut out);
    check_i8("mast_angle",  ada_link::mast_angle(), expected.mast_angle, tol, &mut out);

    // Steering: CSV stores degrees (f64); Ada stores integer degrees (i8).
    let steer = ada_link::wheel_angles();
    check_f64("steer_fl", steer[0] as f64, expected.steer_fl, stol, &mut out);
    check_f64("steer_fr", steer[1] as f64, expected.steer_fr, stol, &mut out);
    check_f64("steer_rl", steer[2] as f64, expected.steer_rl, stol, &mut out);
    check_f64("steer_rr", steer[3] as f64, expected.steer_rr, stol, &mut out);

    out
}

// ── Main ──────────────────────────────────────────────────────────────────────

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: replay <trace.csv> [tolerance]");
        eprintln!("  tolerance  integer, applied to power (±N/127) and degrees (±N°); default 5");
        std::process::exit(2);
    }
    let csv_path = &args[1];
    let tol: i8 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(5);

    let rows = parse_csv(csv_path);
    if rows.is_empty() {
        eprintln!("error: no data rows found in {csv_path}");
        std::process::exit(2);
    }
    println!("Loaded {} rows from {csv_path}  (tolerance ±{tol})", rows.len());

    // Auto-load the waypoint sidecar (<csvpath>.wpts) written by the logger.
    // Without it, Ada has no waypoints and falls into autonomous mode instead of
    // GNC path-following, producing entirely different behaviour.
    let wpts_path = format!("{csv_path}.wpts");
    let wpts = load_waypoints(&wpts_path);
    if !wpts.is_empty() {
        println!("Loaded {} waypoints from {wpts_path}", wpts.len());
        ada_link::set_waypoints(wpts);
    } else {
        println!("No waypoint sidecar found ({wpts_path}); Ada will use autonomous mode");
    }

    // Set initial sensor state from the first row before starting Ada so that
    // the pre-delay initialisation reads get reasonable values.
    let initial_enc = [rows[0].enc_fl, rows[0].enc_fr, rows[0].enc_rl, rows[0].enc_rr];
    inject_sensors(&rows[0], &rows[0], &[0; 4]);

    // Detect whether this trace has the ada_step_us column (non-zero in any row).
    let has_step_us = rows.iter().any(|r| r.ada_step_us > 0);
    if has_step_us {
        println!("Exact-DT replay: using logged ada_step_us timestamps.");
    } else {
        println!("Fixed-DT replay: ada_step_us column absent, using 40 ms per step.");
    }

    // Enable virtual-time mode.  Any non-zero value switches mars_rover_clock()
    // away from wall-clock.  The first meaningful clock value will be set by
    // GATE_NEXT_CLOCK_US before Ada wakes from its first delay.
    ada_link::enable_virtual_time(rows[0].time_ms * 1_000 + 1);

    // Install the step-gate delay hook BEFORE spawning the Ada thread.
    // This redirects every `Delay_Milliseconds` call into our barrier.
    ada_link::set_delay_ms_hook(|ms| {
        let (mtx, cv_ada, cv_done) = step_gate();
        // Phase 1 — announce Ada is entering a delay.
        {
            let mut g = mtx.lock().unwrap();
            g.done_seq += 1;
            cv_done.notify_all();
        }
        // Phase 2 — wait for harness to grant a permit.
        let mut g = cv_ada.wait_while(mtx.lock().unwrap(), |s| s.permits == 0).unwrap();
        g.permits -= 1;
        // Phase 3 — advance the clock (exact-DT when the harness has preloaded
        // the logged timestamp, +amount fallback otherwise).
        let target = GATE_NEXT_CLOCK_US.load(Ordering::Relaxed);
        if target > 0 {
            ada_link::set_virtual_clock(target);
        } else {
            ada_link::advance_virtual_clock_by(ms as u64 * 1_000);
        }
    });

    // Spawn Ada control thread.
    ada_link::spawn_ada_thread();

    let (mtx, cv_ada, cv_done) = step_gate();

    // Wait for Ada to reach its first delay (done_seq goes from 0 → 1).
    // This covers the initialisation path (Set_Power, initial sensor reads, etc.)
    // before the control loop enters its first Delay_Milliseconds call.
    let g = cv_done
        .wait_while(mtx.lock().unwrap(), |s| s.done_seq == 0)
        .unwrap();
    let mut current_done = g.done_seq;
    drop(g);

    // Encoder baseline: cumulative totals at the end of the last Ada step.
    // Initialised to the first row's values (pre-run ticks, usually 0).
    let mut enc_base: [i32; 4] = initial_enc;

    // row_cursor: index of the first CSV row not yet consumed by a completed step.
    let mut row_cursor: usize = 0;

    let mut pass: usize = 0;
    let mut fail: usize = 0;
    let mut step_num: usize = 0;

    // Waypoint-index tracking: simulate Ada's capture logic independently for
    // Ada's EKF state and the logged EKF state so divergence is immediately visible.
    let wps_len = ada_link::waypoints().len();
    let mut ada_wp_idx: usize = 0;
    let mut log_wp_idx: usize = 0;

    loop {
        if row_cursor >= rows.len() {
            break;
        }

        // Ada is currently blocked at a delay (waiting for a permit).

        // Pre-compute the step window boundary before releasing Ada.
        //
        // With ada_step_us: find the first row where Ada's step timestamp has
        // advanced — that row belongs to the *next* step, so everything before
        // it is the current window.
        //
        // Without ada_step_us (old logs): fall back to the virtual-clock method
        // (first row past the 40 ms mark), computed after Ada advances the clock.
        let step_end_precomputed: Option<usize> = if has_step_us {
            let cur = rows[row_cursor].ada_step_us;
            rows[row_cursor..]
                .iter()
                .position(|r| r.ada_step_us > cur)
                .map(|off| row_cursor + off)
        } else {
            None
        };

        // If we have a precomputed boundary, tell the delay hook to set
        // VIRTUAL_CLOCK to the logged timestamp of that boundary row.  Ada
        // will then see the exact DT from the original run.
        if let Some(end) = step_end_precomputed {
            GATE_NEXT_CLOCK_US.store(rows[end].ada_step_us, Ordering::Relaxed);
        } else {
            GATE_NEXT_CLOCK_US.store(0, Ordering::Relaxed); // trigger fallback
        }

        // Determine the last row in the upcoming window for sensors that Ada
        // reads after waking (IMU gyro, GPS).  In the original run, Bevy kept
        // updating those atomics during Ada's sleep; Ada saw the final value
        // written before it woke.  For new-format logs we know the exact boundary;
        // for old-format logs use a +40 ms time estimate.
        let window_last_idx = step_end_precomputed
            .filter(|&e| e > row_cursor)
            .map(|e| e - 1)
            .unwrap_or_else(|| {
                let start_ms = rows[row_cursor].time_ms;
                rows[row_cursor..]
                    .iter()
                    .rposition(|r| r.time_ms <= start_ms + 40)
                    .map(|off| row_cursor + off)
                    .unwrap_or(row_cursor)
            });

        // Inject non-GPS sensors for the step Ada is about to run.
        // - Encoder deltas:  computed from the first row of the window (enc_base).
        // - IMU gyro:        from the last row of the window (window_last_idx),
        //                    matching what Bevy last wrote before Ada woke.
        inject_sensors(&rows[row_cursor], &rows[window_last_idx], &enc_base);

        // Inject GPS: scan the upcoming window and inject the last fix found.
        //
        // In the original run, Bevy set GPS fixes during Ada's sleep (i.e. within
        // this window).  Ada read those fixes when it woke.  We must inject them
        // BEFORE granting the permit so Ada sees the same fixes at the same step.
        //
        // For new-format logs step_end_precomputed is exact; for old-format logs
        // use a +40 ms time estimate as the scan bound.
        let gps_scan_end = step_end_precomputed.unwrap_or_else(|| {
            let start_ms = rows[row_cursor].time_ms;
            rows[row_cursor..]
                .iter()
                .position(|r| r.time_ms > start_ms + 40)
                .map(|off| row_cursor + off)
                .unwrap_or(rows.len())
        });
        inject_gps_window(&rows, row_cursor, gps_scan_end);

        // Grant one permit — Ada will wake, advance VIRTUAL_CLOCK to the logged
        // timestamp (or +40 ms for old logs), run its control cycle, then block
        // again at the next delay.
        {
            let mut g = mtx.lock().unwrap();
            g.permits += 1;
            cv_ada.notify_one();
        }

        // Wait for Ada to enter its next delay (done_seq increments again).
        // At this point Ada has written its output atomics for this step.
        let g = cv_done
            .wait_while(mtx.lock().unwrap(), |s| s.done_seq == current_done)
            .unwrap();
        current_done = g.done_seq;
        drop(g);

        // Determine the step window boundary.
        let step_end = if let Some(end) = step_end_precomputed {
            end
        } else {
            // Old-format log: use the (just-advanced) virtual clock to find boundary.
            // mars_rover_clock returns µs; convert to ms.
            let vt_ms = ada_link::mars_rover_clock() / 1_000;
            rows[row_cursor..]
                .iter()
                .position(|r| r.time_ms > vt_ms)
                .map(|off| row_cursor + off)
                .unwrap_or(rows.len())
        };

        // The expected-output row is the FIRST row of the NEXT window.
        //
        // Timing: Ada runs its cycle, writes outputs, then calls Delay_Milliseconds
        // (recorded as ada_step_us = T_delay_N).  The Bevy logger captures those
        // outputs in PostUpdate frames that run AFTER T_delay_N — i.e. in the
        // next window (rows[step_end..]).  Rows inside the current window
        // (rows[row_cursor..step_end]) were logged while Ada was sleeping after
        // the *previous* delay call and reflect the *previous* cycle's outputs.
        let expected_idx = step_end.min(rows.len() - 1);
        let expected = &rows[expected_idx];

        let mismatches = compare_outputs(expected, tol);
        step_num += 1;

        // Read Ada's EKF estimate and the matching logged estimate.
        let ada_est = ada_ekf();
        let log_est: Option<(f64, f64, f64)> = match (
            expected.log_est_x, expected.log_est_y, expected.log_est_theta
        ) {
            (Some(x), Some(y), Some(th)) => Some((x, y, th)),
            _ => None,
        };

        // Waypoint index string: shows which WP each party is targeting.
        // Built BEFORE the capture update so it reflects THIS step's target.
        let wp_str = if wps_len > 0 {
            let ada_label = if ada_wp_idx < wps_len {
                format!("WP{ada_wp_idx}")
            } else {
                "done".to_string()
            };
            let log_label = if log_wp_idx < wps_len {
                format!("WP{log_wp_idx}")
            } else {
                "done".to_string()
            };
            if ada_wp_idx != log_wp_idx {
                format!("  wp ada\u{2192}{ada_label} log\u{2192}{log_label} \u{26a0}DIVERGED")
            } else {
                format!("  wp\u{2192}{ada_label}")
            }
        } else {
            String::new()
        };

        // EKF delta string: shows position and heading error relative to log.
        let ekf_delta_str = match (ada_est, log_est) {
            (Some((ax, ay, ath)), Some((lx, ly, lth))) => {
                let dth = (ath - lth).to_degrees();
                // Normalise heading delta to (−180°, 180°]
                let dth = if dth > 180.0 { dth - 360.0 }
                           else if dth <= -180.0 { dth + 360.0 }
                           else { dth };
                format!("  ekf ada{ada}  log{log}  \u{03b4}({dpos:.3}m,{dth:.1}\u{00b0})",
                    ada  = fmt_ekf(Some((ax, ay, ath))),
                    log  = fmt_ekf(Some((lx, ly, lth))),
                    dpos = (ax - lx).hypot(ay - ly),
                    dth  = dth)
            },
            _ => format!("  ekf ada{}  log{}", fmt_ekf(ada_est), fmt_ekf(log_est)),
        };

        if mismatches.is_empty() {
            pass += 1;
            let (pl, pr) = ada_link::motor_powers();
            println!(
                "PASS step {:4}  rows {:4}–{:4}  t={:6}–{:6}ms  pwr={:4}/{:4}  mast={:3}°{}{}",
                step_num,
                row_cursor, expected_idx,
                rows[row_cursor].time_ms, expected.time_ms,
                pl, pr,
                ada_link::mast_angle(),
                ekf_delta_str,
                wp_str,
            );
        } else {
            fail += 1;
            print!(
                "FAIL step {:4}  row {:4}  t={:6}ms :",
                step_num, expected_idx, expected.time_ms,
            );
            for m in &mismatches {
                print!(
                    "  {}={} exp={} (Δ={:.1}>tol={:.0})",
                    m.field, m.actual, m.expected,
                    (m.actual - m.expected).abs(),
                    m.tolerance,
                );
            }
            println!("{}{}", ekf_delta_str, wp_str);
        }

        // Advance waypoint indices if EKF position is within capture radius.
        // Done AFTER printing so the output shows the target for this step.
        {
            let wps = ada_link::waypoints();
            if !wps.is_empty() {
                if let Some((ax, ay, _)) = ada_est {
                    ada_wp_idx = maybe_capture_wp(ax, ay, ada_wp_idx, wps);
                }
                if let Some((lx, ly, _)) = log_est {
                    log_wp_idx = maybe_capture_wp(lx, ly, log_wp_idx, wps);
                }
            }
        }

        // Update encoder baseline to the START of this step window.
        enc_base = [
            rows[row_cursor].enc_fl,
            rows[row_cursor].enc_fr,
            rows[row_cursor].enc_rl,
            rows[row_cursor].enc_rr,
        ];

        // Advance the row cursor past the rows consumed by this step.
        row_cursor = step_end.max(row_cursor + 1);
    }

    println!(
        "\nReplay complete: {} PASS  {} FAIL  ({} Ada steps, {} CSV rows)",
        pass, fail, step_num, rows.len(),
    );

    std::process::exit(if fail > 0 { 1 } else { 0 });
}
