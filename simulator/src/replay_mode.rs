//! Trace-driven renderer mode for the GUI binary.
//!
//! Activated by passing `--replay <path>` on the command line.  The GUI
//! replaces its live driver (Controller + Simulator plugins + Ada thread) with
//! a `ReplayMode` plugin whose `trace_reader` system advances a `TraceSource`
//! at real-time rate and drives the renderer from the recorded values.
//!
//! No Ada thread is spawned, no physics is integrated, and the `SimInputs`
//! queue is not drained — the GUI becomes a pure consumer of recorded state.
//!
//! See `HANDOFF-PHASE-4.md` for the full design rationale.

use std::{sync::Mutex, time::Instant};

use bevy::prelude::*;

use mars_rover_ada_link as ada_link;
use mars_rover_core::domain::{Angle, Position, WheelID};

use crate::{
    replay_source::{TraceRow, TraceSource},
    resource::RobotRes,
};

/// Marks that the GUI is running a trace-driven replay rather than a live sim.
/// Other plugins (visualizer, ribbon) can read this to gate keybinds /
/// drag-drop handlers that would otherwise push `SimInput`s into a queue that
/// nothing drains.
#[derive(Resource, Debug, Default)]
pub struct ReplayActive(pub bool);

/// The pluggable row source.  Wrapped in a `Mutex` to satisfy Bevy's
/// `Resource: Sync` bound — `TraceSource: Send` is enough on the inside.
#[derive(Resource)]
struct ReplaySource(Mutex<Box<dyn TraceSource>>);

/// Playback clock.  `start_wall` is initialised on the first tick and anchors
/// the virtual row time to wall-clock elapsed.  `current` holds the last row
/// rendered so we can repeat it when playback outruns the trace.
#[derive(Resource, Default)]
struct ReplayClock {
    start_wall: Option<Instant>,
    /// Wall-clock offset added to `start_wall` when the user restarts (R key).
    /// Avoids disturbing `start_wall` itself so a later reset works idempotently.
    row_epoch_ms: u64,
    /// Requested by the R key; honoured on the next `trace_reader` tick.
    rewind_requested: bool,
    current: Option<TraceRow>,
    finished: bool,
}

pub struct ReplayMode {
    source: Mutex<Option<Box<dyn TraceSource>>>,
}

impl ReplayMode {
    pub fn new(source: Box<dyn TraceSource>) -> Self {
        Self {
            source: Mutex::new(Some(source)),
        }
    }
}

impl Plugin for ReplayMode {
    fn build(&self, app: &mut App) {
        let source = self
            .source
            .lock()
            .expect("ReplayMode source mutex poisoned")
            .take()
            .expect("ReplayMode plugin built twice");
        app.insert_resource(ReplaySource(Mutex::new(source)))
            .insert_resource(ReplayActive(true))
            .init_resource::<ReplayClock>()
            // PreUpdate so every Update-scheduled visualizer system (update_rover,
            // draw_ekf_marker, update_text) sees the freshly-populated state.
            .add_systems(PreUpdate, (handle_replay_keys, trace_reader).chain());
    }
}

/// R restarts the replay from t=0.  Handled here (rather than in the
/// visualizer's `handle_keyboard_input`) so the rewind toggles a replay-local
/// resource without touching `SimInputs`.
fn handle_replay_keys(keys: Res<ButtonInput<KeyCode>>, mut clock: ResMut<ReplayClock>) {
    if keys.just_pressed(KeyCode::KeyR) {
        clock.rewind_requested = true;
    }
}

/// Convert a CSV steering column (degrees) into an `Angle` and push it onto
/// the robot.  Ignores the `Result` because an out-of-range value in the CSV
/// would have been an out-of-range steering command during the original run
/// — it's the logger's value, and the renderer's job is to display it.
fn apply_row_to_robot(row: &TraceRow, robot: &mut mars_rover_core::domain::Robot) {
    let (tx, ty, theta) = row.truth;
    robot.set_position(Position::new(tx, ty));
    robot.set_heading(Angle::new(theta));

    // Steering columns are indexed [FL, FR, RL, RR] (the order the logger
    // writes them).  Do NOT translate via `WheelID as usize` — the enum's
    // discriminant order is FL, RL, RR, FR and has bitten this codebase
    // before (see feedback_enum_discriminant_translation.md).
    const CORNER_WHEELS: [WheelID; 4] = [
        WheelID::FrontLeft,
        WheelID::FrontRight,
        WheelID::RearLeft,
        WheelID::RearRight,
    ];
    for (idx, &wheel) in CORNER_WHEELS.iter().enumerate() {
        let _ = robot.set_wheel_steering_angle(wheel, Angle::new(row.steer[idx].to_radians()));
    }

    robot.set_distance_sensor_angle(Angle::new((row.mast_deg as f64).to_radians()));

    // Sonar: u32::MAX means "no reading this frame".  Robot stores Option<f64>
    // in metres — convert from cm.
    let sonar_m = if row.sonar_cm == u32::MAX {
        None
    } else {
        Some(row.sonar_cm as f64 / 100.0)
    };
    robot.update_distance_sensor(sonar_m);

    // EKF marker: drive via ada-link atomics so the live `draw_ekf_marker`
    // system works unchanged.
    match row.est {
        Some((x, y, t)) => ada_link::set_ekf_estimate(x, y, t),
        None => ada_link::clear_ekf_estimate(),
    }
}

fn trace_reader(
    mut robot: ResMut<RobotRes>,
    source: Res<ReplaySource>,
    mut clock: ResMut<ReplayClock>,
) {
    if clock.rewind_requested {
        if let Ok(mut s) = source.0.lock() {
            if let Err(e) = s.reset() {
                eprintln!("[replay] reset failed: {e}");
            }
        }
        clock.start_wall = Some(Instant::now());
        clock.row_epoch_ms = 0;
        clock.current = None;
        clock.finished = false;
        clock.rewind_requested = false;
    }

    if clock.finished {
        // Keep the last row pinned on screen so the user sees where the
        // trajectory ended rather than the rover snapping back to origin.
        return;
    }

    let start = *clock.start_wall.get_or_insert_with(Instant::now);
    let elapsed_ms = start.elapsed().as_millis() as u64 + clock.row_epoch_ms;

    // Advance the cursor until we catch up to `elapsed_ms`, or run out of rows.
    let mut latest = clock.current.clone();
    let mut hit_eof = false;
    loop {
        match &latest {
            Some(row) if row.time_ms > elapsed_ms => break,
            _ => {}
        }
        let Ok(mut src) = source.0.lock() else { break };
        let Some(next) = src.next_row() else {
            hit_eof = true;
            break;
        };
        drop(src);
        let next_time = next.time_ms;
        latest = Some(next);
        if next_time > elapsed_ms {
            // Rendered row is already ahead; keep it and stop draining.
            break;
        }
    }

    if let Some(row) = latest.as_ref() {
        apply_row_to_robot(row, &mut **robot);
    }
    clock.current = latest;
    if hit_eof {
        clock.finished = true;
    }
}

