//! Bevy-side adapter for the Ada control software.  The FFI atomics and
//! `#[no_mangle]` exports live in `mars-rover-ada-link`; this module only
//! owns the Bevy `Controller` plugin and the per-frame `control` system that
//! bridges Bevy resources to ada-link's state.

use bevy::prelude::*;

use mars_rover_ada_link as ada_link;
use mars_rover_ada_link::{EncoderState, ImuState};

use crate::resource::{RobotRes, SimStep};

// Re-export the ada-link public API that GUI code (main.rs, logger.rs,
// visualizer.rs, console.rs) uses through the `controller::…` path today.
// Some items (e.g. `GncState`, `set_noise_seed`) are not yet referenced by
// the GUI binary directly — they're part of the API surface the brief says
// to preserve for downstream callers (and `set_noise_seed` will be wired to
// a future `--noise-seed` CLI flag).
#[allow(unused_imports)]
pub use mars_rover_ada_link::{
    ada_step_us, drain_new_messages, ekf_estimated_state, encoder_truth_total, gnc_state,
    gps_current, gps_interval_ms, imu_gyro_z, mast_angle, motor_powers, set_gps_interval_ms,
    set_noise_seed, set_waypoints, sonar_distance, waypoints, GncState,
};

pub struct Controller;

impl Plugin for Controller {
    fn build(&self, app: &mut App) {
        ada_link::spawn_ada_thread();
        app.add_systems(Update, control.in_set(SimStep::Control));
    }
}

fn control(
    keys: Res<ButtonInput<KeyCode>>,
    mut robot: ResMut<RobotRes>,
    time: Res<Time>,
    mut enc: Local<EncoderState>,
    mut imu: Local<ImuState>,
) {
    let mut keys_mask: u16 = 0;
    if keys.pressed(KeyCode::ArrowUp)    { keys_mask |= 0x0001; }
    if keys.pressed(KeyCode::ArrowDown)  { keys_mask |= 0x0002; }
    if keys.pressed(KeyCode::ArrowLeft)  { keys_mask |= 0x0004; }
    if keys.pressed(KeyCode::ArrowRight) { keys_mask |= 0x0008; }
    ada_link::tick_controller(
        &mut **robot,
        time.delta(),
        ada_link::now_ms(),
        keys_mask,
        &mut enc,
        &mut imu,
    );
}
