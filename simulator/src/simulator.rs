//! Simulation of the robot in an environment with obstacles.
//!
//! The robot's position is updated based on the robot's state and elapsed time, taking into
//! account collisions with the environment. The measured distance of the robot's sensor is
//! updated.
//!
//! The physics body lives in `mars_rover_core::sim::advance`; this module only owns the
//! Bevy `Simulator` plugin and the per-frame `simulate` system that reads resources and
//! calls `advance`.  It also owns `apply_sim_inputs`, which drains the `SimInputsRes` queue
//! of user / control-plane events and forwards FFI-touching side effects to ada-link.

use bevy::prelude::*;

use mars_rover_core::sim::SimInput;

use crate::resource::{EnvironmentRes, RobotRes, SimInputsRes, SimStep, SlipFreeMode, TerrainRes};

pub struct Simulator;

impl Plugin for Simulator {
    fn build(&self, app: &mut App) {
        app.init_resource::<SimInputsRes>()
            .add_systems(Update, apply_sim_inputs.in_set(SimStep::ApplyInputs))
            .add_systems(Update, simulate.in_set(SimStep::Simulate));
    }
}

fn apply_sim_inputs(
    inputs: Res<SimInputsRes>,
    mut robot: ResMut<RobotRes>,
    mut environment: ResMut<EnvironmentRes>,
    mut slip_free: ResMut<SlipFreeMode>,
) {
    for input in inputs.0.drain() {
        mars_rover_core::sim::apply(
            &input,
            &mut **robot,
            &mut **environment,
            &mut slip_free.0,
            |inp| match inp {
                SimInput::Reset { pose } | SimInput::RequestEkfReset { pose } => {
                    mars_rover_ada_link::request_ekf_reset(pose.0, pose.1, pose.2);
                }
                SimInput::LoadWaypoints(wps) => {
                    let arr: Vec<[f32; 2]> =
                        wps.iter().map(|&(x, y)| [x as f32, y as f32]).collect();
                    mars_rover_ada_link::set_waypoints(arr);
                }
                _ => {}
            },
        );
    }
}

fn simulate(
    time: Res<Time>,
    mut robot: ResMut<RobotRes>,
    environment: Res<EnvironmentRes>,
    terrain: Res<TerrainRes>,
    slip_free: Res<SlipFreeMode>,
) {
    mars_rover_core::sim::advance(
        &mut **robot,
        &**environment,
        &**terrain,
        time.delta(),
        slip_free.0,
    );
}
