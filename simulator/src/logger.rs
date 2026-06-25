//! Per-frame CSV data logger for EKF diagnostics.
//!
//! Activated by inserting a `LogPath` resource before the app starts (i.e. when
//! `--log <path>` is passed on the command line).  When the resource is absent
//! the plugin is a no-op — no file is opened, no overhead incurred.
//!
//! # CSV columns
//!
//! | Column | Description |
//! |---|---|
//! | `time_ms`   | Bevy elapsed time in ms |
//! | `truth_x/y` | True robot position (m) |
//! | `truth_theta` | True heading (rad) |
//! | `est_x/y/theta` | EKF estimate (m / rad); blank until filter initialises |
//! | `steer_fl/fr/rl/rr` | Commanded steering angle per corner wheel (deg) |
//! | `power_left/right` | Motor power commands (-127..127) |
//! | `enc_fl/fr/rl/rr` | Cumulative truth encoder ticks since start |
//! | `gps_x/y` | Noisy GPS fix position (m); blank on frames without a new fix |
//! | `gps_ts` | GPS fix timestamp (ms); blank when no new fix this frame |
//! | `sonar_dist` | Sonar distance written to Ada this frame (cm); `4294967295` = no obstacle |
//! | `imu_gyro_z` | IMU gyro-Z written to Ada this frame (8192 LSB per rad/s) |
//! | `mast_angle` | Mast angle commanded by Ada this frame (degrees) |

use std::{
    fs::File,
    io::{BufWriter, Write},
    path::PathBuf,
};

use bevy::prelude::*;

use mars_rover_core::{domain::WheelID, sidecar};

use crate::{
    controller,
    resource::{RobotRes, SlipFreeMode, TerrainSeed},
};

/// Insert this resource before `App::run()` to enable logging.
#[derive(Resource)]
pub struct LogPath(pub PathBuf);

pub struct DataLogger;

impl Plugin for DataLogger {
    fn build(&self, app: &mut App) {
        // PostUpdate runs after all Update systems, so truth position and
        // encoder accumulators have already been advanced for this frame.
        app.add_systems(PostUpdate, log_frame);
    }
}

// Per-system persistent state (Bevy Local).
#[derive(Default)]
struct LogState {
    // None until the LogPath resource is confirmed present and the file is opened.
    writer: Option<BufWriter<File>>,
    last_gps_ts: u32,
    // True once the sidecars have been written (first frame only).
    sidecars_written: bool,
}

const HEADER: &str = "time_ms,\
truth_x,truth_y,truth_theta,\
est_x,est_y,est_theta,\
steer_fl,steer_fr,steer_rl,steer_rr,\
power_left,power_right,\
enc_fl,enc_fr,enc_rl,enc_rr,\
gps_x,gps_y,gps_ts,\
sonar_dist,imu_gyro_z,mast_angle,\
ada_step_us";

fn log_frame(
    time: Res<Time>,
    robot: Res<RobotRes>,
    log_path: Option<Res<LogPath>>,
    seed: Res<TerrainSeed>,
    slip_free: Res<SlipFreeMode>,
    mut state: Local<LogState>,
) {
    // If no --log path was given, do nothing.
    let Some(path_res) = log_path else { return };

    // Open the file on the first frame we have a path.
    if state.writer.is_none() {
        match File::create(&path_res.0) {
            Ok(f) => {
                let mut w = BufWriter::new(f);
                let _ = writeln!(w, "{HEADER}");
                state.writer = Some(w);
            }
            Err(e) => {
                eprintln!("[logger] Cannot open {:?}: {e}", path_res.0);
                return;
            }
        }
    }

    // Write `.wpts` + `.meta` sidecars once on the first frame.  The replay
    // binary reads `.wpts`; the trace-driven GUI renderer reads `.meta` to
    // reconstruct the scene seed.
    if !state.sidecars_written {
        state.sidecars_written = true;
        let wpts: Vec<(f64, f64)> = controller::waypoints()
            .iter()
            .map(|wp| (wp[0] as f64, wp[1] as f64))
            .collect();
        let meta = sidecar::TraceMeta {
            seed: seed.0,
            gps_interval_ms: controller::gps_interval_ms(),
            // The GUI doesn't currently seed gaussian() deterministically;
            // record 0 to mean "OS entropy", matching headless --noise-seed 0.
            noise_seed: 0,
            slip_free: slip_free.0,
        };
        if let Err(e) = sidecar::write_sidecars(&path_res.0, &meta, &wpts) {
            eprintln!("[logger] Cannot write sidecars for {:?}: {e}", path_res.0);
        }
    }

    // ── Collect all data before mutably borrowing the writer ──────────────────

    let time_ms = time.elapsed().as_millis();

    // Truth
    let pos = robot.position();
    let truth_x = pos.x();
    let truth_y = pos.y();
    let heading: f64 = robot.heading().into();

    // EKF estimate
    let (est_x, est_y, est_theta) = match controller::ekf_estimated_state() {
        Some((x, y, theta)) => (
            format!("{x:.6}"),
            format!("{y:.6}"),
            format!("{theta:.6}"),
        ),
        None => (String::new(), String::new(), String::new()),
    };

    // Steering angles (degrees, from robot model — the value the physics used)
    let steer = |id: WheelID| -> f64 {
        robot
            .wheel_steering_angle(id)
            .map(|a| f64::from(a).to_degrees())
            .unwrap_or(0.0)
    };
    let steer_fl = steer(WheelID::FrontLeft);
    let steer_fr = steer(WheelID::FrontRight);
    let steer_rl = steer(WheelID::RearLeft);
    let steer_rr = steer(WheelID::RearRight);

    // Motor powers
    let (power_left, power_right) = controller::motor_powers();

    // Encoder truth totals [FL, FR, RL, RR]
    let enc = controller::encoder_truth_total();

    // GPS — only populate columns when a new fix has arrived this frame.
    let (gps_x_raw, gps_y_raw, gps_ts) = controller::gps_current();
    let new_gps = gps_ts != 0 && gps_ts != state.last_gps_ts;
    let (gps_x_col, gps_y_col, gps_ts_col) = if new_gps {
        const SCALE: f64 = 1_000_000.0;
        (
            format!("{:.6}", gps_x_raw as f64 / SCALE),
            format!("{:.6}", gps_y_raw as f64 / SCALE),
            format!("{gps_ts}"),
        )
    } else {
        (String::new(), String::new(), String::new())
    };

    // Track GPS timestamp so we only mark a row as "new fix" once.
    if new_gps {
        state.last_gps_ts = gps_ts;
    }

    // Sensor / actuator columns for replay harness
    let sonar        = controller::sonar_distance();
    let gyro         = controller::imu_gyro_z();
    let mast         = controller::mast_angle();
    let ada_step_us  = controller::ada_step_us();

    // ── Write row ─────────────────────────────────────────────────────────────
    let writer = state.writer.as_mut().unwrap();
    let _ = writeln!(
        writer,
        "{time_ms},{truth_x:.6},{truth_y:.6},{heading:.6},\
         {est_x},{est_y},{est_theta},\
         {steer_fl:.3},{steer_fr:.3},{steer_rl:.3},{steer_rr:.3},\
         {power_left},{power_right},\
         {},{},{},{},\
         {gps_x_col},{gps_y_col},{gps_ts_col},\
         {sonar},{gyro},{mast},{ada_step_us}",
        enc[0], enc[1], enc[2], enc[3],
    );
}
