//! Bevy-free physics step.  The body of the existing Bevy `simulate` system
//! lives here; the Bevy plugin in simulator/src/simulator.rs becomes a shim
//! that reads resources and calls `advance`.

use std::collections::VecDeque;
use std::sync::Mutex;
use std::time::Duration;

use crate::domain;

/// Advance robot physics by `dt`.  Validates the proposed move against the
/// environment (collision + containment); reverts to the prior state on
/// invalid move and zeros the sonar reading in that case.
///
/// `slip_free` forces traction=1.0 on every wheel; otherwise traction is
/// sampled from the terrain at each wheel position.
pub fn advance(
    robot: &mut domain::Robot,
    env: &domain::Environment,
    terrain: &domain::Terrain,
    dt: Duration,
    slip_free: bool,
) {
    // Compute per-wheel traction from the terrain slope beneath each corner wheel.
    // Order matches the velocity Jacobian: [FrontLeft, RearLeft, RearRight, FrontRight].
    // In slip-free mode all traction factors are 1.0, giving ideal no-slip kinematics.
    let slip = if slip_free {
        [1.0; 4]
    } else {
        [
            domain::WheelID::FrontLeft,
            domain::WheelID::RearLeft,
            domain::WheelID::RearRight,
            domain::WheelID::FrontRight,
        ]
        .map(|id| {
            let pos = robot.wheel_position(id);
            terrain.traction_at(pos.x(), pos.y())
        })
    };

    let updated_robot = robot.updated_position(dt, slip);
    let valid_position =
        !env.has_collision(&updated_robot) && env.contains(&updated_robot);

    if valid_position {
        *robot = updated_robot;
    }

    let distance_sensor_position = robot.distance_sensor_position();
    let distance_sensor_heading = robot.distance_sensor_heading();

    robot.update_distance_sensor(if valid_position {
        env.distance_to_next_obstacle(distance_sensor_position, distance_sensor_heading)
    } else {
        Some(0.0)
    });
}

/// A single user / control-plane event that mutates sim state.  Variants must
/// use only mars-rover-core types — no Bevy types, no ada-link types — so a
/// future network-streaming viewer can serialise these over the wire.
pub enum SimInput {
    /// Reset rover position to origin (heading preserved); R-key behaviour.
    /// `pose` carries the heading that existed when the event was queued;
    /// forwarded to `ffi_side_effects` for EKF reset at (0, 0, heading).
    Reset { pose: (f64, f64, f64) },
    /// Toggle slip-free (ideal-traction) physics.
    SetSlipFree(bool),
    /// Move rover to `to` (heading preserved by apply).  Drag-drop rover.
    MoveRover { to: domain::Position },
    /// Move movable obstacle `idx` to `to`.  Drag-drop rock.
    MoveObstacle { idx: usize, to: domain::Position },
    /// Reset obstacle `idx` to its initial position.  Not yet wired to a caller.
    ResetObstacleToDefault { idx: usize },
    /// Replace the waypoint list at runtime.  Not yet wired to a caller.
    LoadWaypoints(Vec<(f64, f64)>),
    /// Controller-only EKF reinit at the given pose.  Drag-drop rover pairs
    /// this with `MoveRover`; R-key pairs it implicitly via `Reset`.
    RequestEkfReset { pose: (f64, f64, f64) },
}

/// Thread-safe queue of pending `SimInput`s.  Push via `push`; drain via `drain`.
pub struct SimInputs(Mutex<VecDeque<SimInput>>);

impl Default for SimInputs {
    fn default() -> Self {
        Self(Mutex::new(VecDeque::new()))
    }
}

impl SimInputs {
    pub fn push(&self, input: SimInput) {
        self.0.lock().unwrap().push_back(input);
    }
    pub fn drain(&self) -> Vec<SimInput> {
        self.0.lock().unwrap().drain(..).collect()
    }
}

/// Apply one `SimInput`.  Pure-domain mutations happen here; FFI-touching
/// side effects (EKF reset, waypoint reload) are forwarded via
/// `ffi_side_effects`, which the caller wires to ada-link.
pub fn apply<F: FnOnce(&SimInput)>(
    input: &SimInput,
    robot: &mut domain::Robot,
    env: &mut domain::Environment,
    slip_free: &mut bool,
    ffi_side_effects: F,
) {
    match input {
        SimInput::Reset { .. } => {
            robot.set_position(domain::Position::new(0.0, 0.0));
            // Heading intentionally NOT changed — matches today's R-key.
        }
        SimInput::SetSlipFree(v) => {
            *slip_free = *v;
        }
        SimInput::MoveRover { to } => {
            robot.set_position(*to);
        }
        SimInput::MoveObstacle { idx, to } => {
            env.set_obstacle_position(*idx, *to);
        }
        SimInput::ResetObstacleToDefault { .. } => {
            // Not yet wired to a caller.
        }
        SimInput::LoadWaypoints(_) | SimInput::RequestEkfReset { .. } => {
            // No domain mutation; handled via ffi_side_effects.
        }
    }
    ffi_side_effects(input);
}
