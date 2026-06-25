//! CSV writer for the headless driver.
//!
//! Port of `simulator/src/logger.rs` without the Bevy `Plugin` wrapper.  The
//! CSV header and column semantics are byte-identical to the GUI logger so a
//! headless CSV can be replayed through `mars-rover-replay` just like a GUI
//! CSV.

use std::{
    fs::File,
    io::{self, BufWriter, Write},
    path::Path,
};

use mars_rover_ada_link as ada_link;
use mars_rover_core::domain::{Robot, WheelID};

const HEADER: &str = "time_ms,\
truth_x,truth_y,truth_theta,\
est_x,est_y,est_theta,\
steer_fl,steer_fr,steer_rl,steer_rr,\
power_left,power_right,\
enc_fl,enc_fr,enc_rl,enc_rr,\
gps_x,gps_y,gps_ts,\
sonar_dist,imu_gyro_z,mast_angle,\
ada_step_us";

/// Owns the CSV file handle and the per-row state needed to suppress duplicate
/// GPS columns (a row only reports a GPS fix on the frame the timestamp
/// changes, matching the GUI logger's behaviour).
pub struct Logger {
    writer: BufWriter<File>,
    last_gps_ts: u32,
}

impl Logger {
    /// Open the log file and write the CSV header.
    pub fn open(path: &Path) -> io::Result<Self> {
        let mut writer = BufWriter::new(File::create(path)?);
        writeln!(writer, "{HEADER}")?;
        Ok(Self { writer, last_gps_ts: 0 })
    }

    /// Write one row.  `virtual_elapsed_us` is the caller's cumulative virtual
    /// clock time (same reference frame as the `now_ms` passed to
    /// `ada_link::tick_controller`).
    pub fn write_row(&mut self, robot: &Robot, virtual_elapsed_us: u64) {
        let time_ms = virtual_elapsed_us / 1_000;

        // Truth
        let pos = robot.position();
        let truth_x = pos.x();
        let truth_y = pos.y();
        let heading: f64 = robot.heading().into();

        // EKF estimate — blank until the filter publishes.
        let (est_x, est_y, est_theta) = match ada_link::ekf_estimated_state() {
            Some((x, y, theta)) => (
                format!("{x:.6}"),
                format!("{y:.6}"),
                format!("{theta:.6}"),
            ),
            None => (String::new(), String::new(), String::new()),
        };

        // Steering angles (degrees, from the robot model).
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
        let (power_left, power_right) = ada_link::motor_powers();

        // Encoder truth totals [FL, FR, RL, RR]
        let enc = ada_link::encoder_truth_total();

        // GPS — populate only when a new fix arrived (timestamp changed).
        let (gps_x_raw, gps_y_raw, gps_ts) = ada_link::gps_current();
        let new_gps = gps_ts != 0 && gps_ts != self.last_gps_ts;
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
        if new_gps {
            self.last_gps_ts = gps_ts;
        }

        // Sensor / actuator columns.
        let sonar = ada_link::sonar_distance();
        let gyro = ada_link::imu_gyro_z();
        let mast = ada_link::mast_angle();
        let ada_step_us = ada_link::ada_step_us();

        let _ = writeln!(
            self.writer,
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
}
