//! Simulation of the robot in an environment with obstacles.
//!
//! The robot's position is updated based on the robot's state and elapsed time, taking into
//! account collisions with the environment. The measured distance of the robot's sensor is
//! updated.

use bevy::prelude::*;

use crate::resource::{EnvironmentRes, RobotRes};

pub struct Simulator;

impl Plugin for Simulator {
    fn build(&self, app: &mut App) {
        app.add_systems(Update, simulate);
    }
}

fn simulate(time: Res<Time>, mut robot: ResMut<RobotRes>, environment: Res<EnvironmentRes>) {
    let updated_robot = robot.updated_position(time.delta());
    let valid_position =
        !environment.has_collision(&updated_robot) && environment.contains(&updated_robot);

    if valid_position {
        *robot = updated_robot.into();
    }

    let distance_sensor_position = robot.distance_sensor_position();
    let distance_sensor_heading = robot.distance_sensor_heading();

    robot.update_distance_sensor(if valid_position {
        environment.distance_to_next_obstacle(distance_sensor_position, distance_sensor_heading)
    } else {
        Some(0.0)
    });
}
