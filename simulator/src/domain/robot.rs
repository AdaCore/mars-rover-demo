//! Robot with four-wheel-independent steering and four-wheel-independent driving (4WIS4WID) and a
//! rotatable distance sensor.

use std::{collections::BTreeMap, f64::consts::PI, slice::Iter, time::Duration};

use nalgebra::{Matrix3x4, RowVector4, Vector3, Vector4};
use thiserror::Error;

use super::{Angle, HasCollision, Position, Shape, Velocity};

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Robot {
    position: Position,
    heading: Angle,
    wheels: BTreeMap<WheelID, Wheel>,
    distance_sensor: DistanceSensor,
    config: RobotConfig,
}

impl Robot {
    pub fn new(heading: Angle, config: RobotConfig) -> Self {
        Self {
            position: Position::default(),
            heading,
            wheels: BTreeMap::from_iter(
                WheelID::iter().map(|wheel_id| (*wheel_id, Wheel::default())),
            ),
            distance_sensor: DistanceSensor::default(),
            config,
        }
    }

    pub fn position(&self) -> Position {
        self.position
    }

    pub fn heading(&self) -> Angle {
        self.heading
    }

    pub fn wheel_steering_angle(&self, wheel_id: WheelID) -> Result<Angle, RobotError> {
        if let Some(wheel) = self.wheels.get(&wheel_id) {
            Ok(wheel.steering_angle)
        } else {
            Err(RobotError::MissingWheelID(wheel_id))
        }
    }

    pub fn wheel_rotating_angle(&self, wheel_id: WheelID) -> Result<Angle, RobotError> {
        if let Some(wheel) = self.wheels.get(&wheel_id) {
            Ok(wheel.rotating_angle)
        } else {
            Err(RobotError::MissingWheelID(wheel_id))
        }
    }

    pub fn wheel_velocity(&self, wheel_id: WheelID) -> Result<Velocity, RobotError> {
        if let Some(wheel) = self.wheels.get(&wheel_id) {
            Ok(wheel.velocity)
        } else {
            Err(RobotError::MissingWheelID(wheel_id))
        }
    }

    pub fn local_wheel_position(&self, wheel_id: WheelID) -> Position {
        Position::new(
            match wheel_id {
                WheelID::FrontLeft | WheelID::FrontRight => {
                    self.config.distance_to_front_wheel_pivot_points
                }
                WheelID::RearLeft | WheelID::RearRight => {
                    -self.config.distance_to_rear_wheel_pivot_points
                }
                WheelID::CenterLeft | WheelID::CenterRight => 0.0,
            },
            match wheel_id {
                WheelID::FrontLeft | WheelID::CenterLeft | WheelID::RearLeft => {
                    self.config.distance_to_left_wheel_pivot_points
                }
                WheelID::FrontRight | WheelID::CenterRight | WheelID::RearRight => {
                    -self.config.distance_to_right_wheel_pivot_points
                }
            },
        ) + Position::new(
            0.0,
            match wheel_id {
                WheelID::FrontLeft | WheelID::CenterLeft | WheelID::RearLeft => {
                    self.config
                        .distance_between_wheel_pivot_point_and_wheel_center
                }
                WheelID::FrontRight | WheelID::CenterRight | WheelID::RearRight => {
                    -self
                        .config
                        .distance_between_wheel_pivot_point_and_wheel_center
                }
            },
        )
        .rotate_vector(self.wheel_steering_angle(wheel_id).unwrap_or_default())
    }

    pub fn wheel_position(&self, wheel_id: WheelID) -> Position {
        self.position
            + self
                .local_wheel_position(wheel_id)
                .rotate_vector(self.heading)
    }

    pub fn distance_sensor_position(&self) -> Position {
        self.position
            + self
                .config
                .distance_sensor_position
                .rotate_vector(self.heading)
    }

    pub fn distance_sensor_heading(&self) -> Angle {
        self.heading + self.distance_sensor.angle
    }

    pub fn distance_sensor_angle(&self) -> Angle {
        self.distance_sensor.angle
    }

    pub fn distance_sensor_distance(&self) -> Option<f64> {
        self.distance_sensor.distance
    }

    pub fn config(&self) -> &RobotConfig {
        &self.config
    }

    pub fn set_position(&mut self, position: Position) {
        self.position = position;
    }

    pub fn set_heading(&mut self, heading: Angle) {
        self.heading = heading;
    }

    pub fn set_wheel_steering_angle(
        &mut self,
        wheel_id: WheelID,
        angle: Angle,
    ) -> Result<(), RobotError> {
        if let Some(ref mut wheel) = self.wheels.get_mut(&wheel_id) {
            wheel.steering_angle = angle;
            Ok(())
        } else {
            Err(RobotError::MissingWheelID(wheel_id))
        }
    }

    pub fn set_wheel_velocity(
        &mut self,
        wheel_id: WheelID,
        velocity: Velocity,
    ) -> Result<(), RobotError> {
        if let Some(ref mut wheel) = self.wheels.get_mut(&wheel_id) {
            wheel.velocity = velocity;
            Ok(())
        } else {
            Err(RobotError::MissingWheelID(wheel_id))
        }
    }

    pub fn set_distance_sensor_angle(&mut self, angle: Angle) {
        self.distance_sensor.angle = angle;
    }

    pub fn update_distance_sensor(&mut self, distance: Option<f64>) {
        self.distance_sensor.distance = distance;
    }

    pub fn updated_position(&mut self, dt: Duration) -> Robot {
        let q = self.velocity_vector();
        let linear_velocity_x = q[0];
        let linear_velocity_y = q[1];
        let angular_velocity = q[2];
        let dt = dt.as_secs_f64();

        let mut robot = self.clone();

        robot.set_position(Position::new(
            self.position.x() + linear_velocity_x * dt,
            self.position.y() + linear_velocity_y * dt,
        ));
        robot.set_heading(Angle::new(
            (Into::<f64>::into(self.heading) + angular_velocity * dt) % (2.0 * PI),
        ));

        for wheel_id in WheelID::iter() {
            if let Some(wheel) = robot.wheels.get_mut(wheel_id) {
                wheel.rotating_angle = wheel.rotating_angle
                    + Angle::new(self.v(*wheel_id) * self.config.wheel_radius.powi(-1) * dt);
            }
        }

        robot
    }

    /// Based on Lee, M. H., & Li, T. H. S. (2015). Kinematics, dynamics and control design of
    /// 4WIS4WID mobile robots. The Journal of Engineering, 2015(1), 6-16.
    fn velocity_vector(&self) -> Vector3<f64> {
        let j = Matrix3x4::from_rows(&[
            RowVector4::from_row_slice(&[
                self.c(WheelID::FrontLeft),
                self.c(WheelID::RearLeft),
                self.c(WheelID::RearRight),
                self.c(WheelID::FrontRight),
            ]),
            RowVector4::from_row_slice(&[
                self.s(WheelID::FrontLeft),
                self.s(WheelID::RearLeft),
                self.s(WheelID::RearRight),
                self.s(WheelID::FrontRight),
            ]),
            RowVector4::from_row_slice(&[
                self.w(WheelID::FrontLeft),
                self.w(WheelID::RearLeft),
                self.w(WheelID::RearRight),
                self.w(WheelID::FrontRight),
            ]),
        ]);
        let v = Vector4::from_row_slice(&[
            self.v(WheelID::FrontLeft),
            self.v(WheelID::RearLeft),
            self.v(WheelID::RearRight),
            self.v(WheelID::FrontRight),
        ]);
        j * v
    }

    fn c(&self, wheel_id: WheelID) -> f64 {
        let steering_angle: f64 = self.wheels[&wheel_id].steering_angle.into();
        let heading: f64 = self.heading.into();
        (steering_angle + heading).cos() / 4.0
    }

    fn s(&self, wheel_id: WheelID) -> f64 {
        let steering_angle: f64 = self.wheels[&wheel_id].steering_angle.into();
        let heading: f64 = self.heading.into();
        (steering_angle + heading).sin() / 4.0
    }

    fn w(&self, wheel_id: WheelID) -> f64 {
        let wheel_position = self.local_wheel_position(wheel_id);
        let (x, y) = (wheel_position.x(), wheel_position.y());
        let steering_angle: f64 = self.wheels[&wheel_id].steering_angle.into();
        (-y * steering_angle.cos() + x * steering_angle.sin()) / (4.0 * x.powi(2) + 4.0 * y.powi(2))
    }

    fn v(&self, wheel_id: WheelID) -> f64 {
        self.wheels[&wheel_id].velocity.into()
    }
}

impl HasCollision for Robot {
    fn shape(&self) -> Shape {
        Shape::Circle {
            position: self.position,
            radius: Position::default().distance(Position::new(
                f64::max(
                    self.config.distance_to_front_wheel_pivot_points,
                    self.config.distance_to_rear_wheel_pivot_points,
                ) + self.config.wheel_radius,
                f64::max(
                    self.config.distance_to_left_wheel_pivot_points,
                    self.config.distance_to_right_wheel_pivot_points,
                ) + self
                    .config
                    .distance_between_wheel_pivot_point_and_wheel_center
                    + self.config.wheel_radius,
            )),
        }
    }
}

#[derive(Error, Debug)]
pub enum RobotError {
    #[error("missing wheel id {0:?}")]
    MissingWheelID(WheelID),
    #[error("invalid wheel id {0}")]
    InvalidWheelID(usize),
}

/// All distances and positions are offsets from the center/pivot of the robot. The robot's front
/// is facing to the right along the positive x-axis.
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotConfig {
    pub distance_to_body_front: f64,
    pub distance_to_body_rear: f64,
    pub distance_to_body_left: f64,
    pub distance_to_body_right: f64,
    pub distance_to_front_wheel_pivot_points: f64,
    pub distance_to_rear_wheel_pivot_points: f64,
    pub distance_to_left_wheel_pivot_points: f64,
    pub distance_to_right_wheel_pivot_points: f64,
    pub distance_between_wheel_pivot_point_and_wheel_center: f64,
    pub wheel_radius: f64,
    pub wheel_width: f64,
    pub distance_sensor_position: Position,
    // Only required for visualisation
    pub body_height: f64,
    pub ground_clearance: f64,
    pub distance_sensor_width: f64,
    pub distance_sensor_height: f64,
    pub distance_sensor_depth: f64,
}

impl RobotConfig {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        distance_to_body_front: f64,
        distance_to_body_rear: f64,
        distance_to_body_left: f64,
        distance_to_body_right: f64,
        distance_to_front_wheel_pivot_points: f64,
        distance_to_rear_wheel_pivot_points: f64,
        distance_to_left_wheel_pivot_points: f64,
        distance_to_right_wheel_pivot_points: f64,
        distance_between_wheel_pivot_point_and_wheel_center: f64,
        wheel_radius: f64,
        wheel_width: f64,
        distance_sensor_position: Position,
        body_height: f64,
        ground_clearance: f64,
        distance_sensor_width: f64,
        distance_sensor_height: f64,
        distance_sensor_depth: f64,
    ) -> Self {
        RobotConfig {
            distance_to_body_front,
            distance_to_body_rear,
            distance_to_body_left,
            distance_to_body_right,
            distance_to_front_wheel_pivot_points,
            distance_to_rear_wheel_pivot_points,
            distance_to_left_wheel_pivot_points,
            distance_to_right_wheel_pivot_points,
            distance_between_wheel_pivot_point_and_wheel_center,
            wheel_radius,
            wheel_width,
            distance_sensor_position,
            body_height,
            ground_clearance,
            distance_sensor_width,
            distance_sensor_height,
            distance_sensor_depth,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum WheelID {
    FrontLeft,
    RearLeft,
    RearRight,
    FrontRight,
    CenterLeft,
    CenterRight,
}

impl WheelID {
    pub fn iter() -> Iter<'static, WheelID> {
        static WHEELS: [WheelID; 6] = [
            WheelID::FrontLeft,
            WheelID::RearLeft,
            WheelID::RearRight,
            WheelID::FrontRight,
            WheelID::CenterLeft,
            WheelID::CenterRight,
        ];
        WHEELS.iter()
    }
}

impl TryFrom<usize> for WheelID {
    type Error = RobotError;

    fn try_from(value: usize) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(WheelID::FrontLeft),
            1 => Ok(WheelID::RearLeft),
            2 => Ok(WheelID::RearRight),
            3 => Ok(WheelID::FrontRight),
            _ => Err(RobotError::InvalidWheelID(value)),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Wheel {
    steering_angle: Angle,
    rotating_angle: Angle,
    velocity: Velocity,
}

#[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
struct DistanceSensor {
    angle: Angle,
    distance: Option<f64>,
}

#[cfg(test)]
mod tests {
    use std::{f64::consts::PI, time::Duration};

    use approx::assert_abs_diff_eq;
    use rstest::rstest;

    use super::*;
    use crate::tests::{plot_line_chart, plot_point_chart};

    const EPSILON: f64 = 2.0 * f64::EPSILON;

    #[rstest]
    #[case::up(             0.5 * PI, 1.0, 1, ( 0.0,  1.0) )]
    #[case::down(           1.5 * PI, 1.0, 1, ( 0.0, -1.0) )]
    #[case::left(                 PI, 1.0, 1, (-1.0,  0.0) )]
    #[case::right(               0.0, 1.0, 1, ( 1.0,  0.0) )]
    #[case::double_time(         0.0, 1.0, 2, ( 2.0,  0.0) )]
    #[case::double_velocity(     0.0, 2.0, 1, ( 2.0,  0.0) )]
    fn test_robot_update_position(
        #[case] heading: f64,
        #[case] velocity: f64,
        #[case] time: u64,
        #[case] position: (f64, f64),
    ) {
        let mut robot = Robot::new(Angle::new(heading), cfg());
        set_velocity(&mut robot, Velocity::new(velocity));
        robot = robot.updated_position(Duration::from_secs(time));
        assert_abs_diff_eq!(robot.position().x(), position.0, epsilon = EPSILON);
        assert_abs_diff_eq!(robot.position().y(), position.1, epsilon = EPSILON);
        assert_abs_diff_eq!(
            Into::<f64>::into(robot.heading()),
            heading,
            epsilon = EPSILON
        );
    }

    #[rstest]
    #[case::straight(0.0)]
    #[case::left(0.25 * PI)]
    #[case::right(-0.25 * PI)]
    fn test_robot_front_wheel_steering(#[case] steering_angle: f64) {
        let mut robot = Robot::new(Angle::new(0.5 * PI), cfg());
        set_front_wheel_steering_angle(&mut robot, Angle::new(steering_angle));
        set_velocity(&mut robot, Velocity::new(0.1));

        let points = (0..=100)
            .step_by(1)
            .map(|_| {
                robot = robot.updated_position(Duration::from_millis(100));
                (robot.position.x() as f32, robot.position.y() as f32)
            })
            .collect::<Vec<_>>();

        set_snapshot_suffix!("{steering_angle}");
        insta::assert_snapshot!(plot_point_chart(&points));
    }

    #[rstest]
    #[case::straight(0.0)]
    #[case::left(0.28 * PI)]
    #[case::right(-0.28 * PI)]
    fn test_robot_all_wheel_steering(#[case] steering_angle: f64) {
        let mut robot = Robot::new(Angle::new(0.5 * PI), cfg());
        set_all_wheel_steering_angle(&mut robot, Angle::new(steering_angle));
        set_velocity(&mut robot, Velocity::new(0.1));

        let points = (0..=1000)
            .step_by(1)
            .map(|_| {
                robot = robot.updated_position(Duration::from_millis(100));
                (robot.position.x() as f32, robot.position.y() as f32)
            })
            .collect::<Vec<_>>();

        set_snapshot_suffix!("{steering_angle}");
        insta::assert_snapshot!(plot_point_chart(&points));
    }

    #[rstest]
    #[case::straight(          0.0, (            0.0,            1.0), 0.5 * PI )]
    #[case::left(         0.5 * PI, (           -1.0,            0.0), 0.5 * PI )]
    #[case::right(       -0.5 * PI, (            1.0,            0.0), 0.5 * PI )]
    #[case::half_left(   0.25 * PI, (-f64::sqrt(0.5), f64::sqrt(0.5)), 0.5 * PI )]
    #[case::half_right( -0.25 * PI, ( f64::sqrt(0.5), f64::sqrt(0.5)), 0.5 * PI )]
    fn test_robot_parallel_steering(
        #[case] steering_angle: f64,
        #[case] position: (f64, f64),
        #[case] heading: f64,
    ) {
        let mut robot = Robot::new(Angle::new(0.5 * PI), cfg());
        set_parallel_steering_angle(&mut robot, Angle::new(steering_angle));
        set_velocity(&mut robot, Velocity::new(1.0));
        robot = robot.updated_position(Duration::from_secs(1));
        assert_abs_diff_eq!(robot.position.x(), position.0, epsilon = EPSILON);
        assert_abs_diff_eq!(robot.position.y(), position.1, epsilon = EPSILON);
        assert_abs_diff_eq!(Into::<f64>::into(robot.heading), heading, epsilon = EPSILON);
    }

    #[rstest]
    #[case(Angle::new(0.0), Position::new(1.2, 2.1))]
    #[case(Angle::new(0.5 * PI), Position::new(0.9, 2.2))]
    #[case(Angle::new(PI), Position::new(0.8, 1.9))]
    #[case(Angle::new(1.5 * PI), Position::new(1.1, 1.8))]
    fn test_robot_distance_sensor_position(#[case] angle: Angle, #[case] position: Position) {
        let config = RobotConfig::new(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            Position::new(0.2, 0.1),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        );
        let mut robot = Robot::new(angle, config);
        robot.set_position(Position::new(1.0, 2.0));
        assert_abs_diff_eq!(robot.distance_sensor_position(), position);
    }

    #[test]
    fn test_robot_distance_sensor_heading() {
        let mut robot = Robot::new(Angle::new(0.25 * PI), cfg());
        robot.set_distance_sensor_angle(Angle::new(0.25 * PI));
        assert_abs_diff_eq!(robot.distance_sensor_heading(), Angle::new(0.5 * PI));
    }

    #[test]
    fn test_robot_distance_sensor_distance() {
        let mut robot = Robot::new(Angle::new(0.25 * PI), cfg());
        robot.update_distance_sensor(Some(1.2));
        assert_abs_diff_eq!(robot.distance_sensor_distance().unwrap(), 1.2);
    }

    #[test]
    fn test_robot_positions() {
        let mut robot = Robot::new(Angle::new(0.25 * PI), cfg());
        robot.set_position(Position::new(-0.2, -0.4));
        set_all_wheel_steering_angle(&mut robot, Angle::new(0.25 * PI));

        let mut object_points = vec![];
        object_points.push(vec![
            robot.distance_sensor_position().into(),
            robot.distance_sensor_position().into(),
        ]);
        object_points.push(
            [
                Corner::FrontLeft,
                Corner::RearLeft,
                Corner::RearRight,
                Corner::FrontRight,
                Corner::FrontLeft,
            ]
            .into_iter()
            .map(|c| body_corner(&robot, c).into())
            .collect(),
        );
        for wheel_id in [
            WheelID::FrontLeft,
            WheelID::RearLeft,
            WheelID::RearRight,
            WheelID::FrontRight,
        ] {
            object_points.push(
                [
                    Corner::FrontLeft,
                    Corner::RearLeft,
                    Corner::RearRight,
                    Corner::FrontRight,
                    Corner::FrontLeft,
                ]
                .into_iter()
                .map(|c| wheel_corner(&robot, wheel_id, c).into())
                .collect(),
            );
        }

        insta::assert_snapshot!(plot_line_chart(&object_points));
    }

    fn wheel_corner(robot: &Robot, wheel_id: WheelID, corner: Corner) -> Position {
        robot.position
            + (robot.local_wheel_position(wheel_id)
                + Position::new(
                    match corner {
                        Corner::FrontLeft | Corner::FrontRight => robot.config.wheel_radius,
                        Corner::RearLeft | Corner::RearRight => -robot.config.wheel_radius,
                    },
                    match corner {
                        Corner::FrontLeft | Corner::RearLeft => robot.config.wheel_width / 2.0,
                        Corner::RearRight | Corner::FrontRight => -robot.config.wheel_width / 2.0,
                    },
                )
                .rotate_vector(robot.wheels[&wheel_id].steering_angle))
            .rotate_vector(robot.heading)
    }

    fn body_corner(robot: &Robot, corner: Corner) -> Position {
        robot.position
            + Position::new(
                match corner {
                    Corner::FrontLeft | Corner::FrontRight => robot.config.distance_to_body_front,
                    Corner::RearLeft | Corner::RearRight => -robot.config.distance_to_body_rear,
                },
                match corner {
                    Corner::FrontLeft | Corner::RearLeft => robot.config.distance_to_body_left,
                    Corner::RearRight | Corner::FrontRight => -robot.config.distance_to_body_right,
                },
            )
            .rotate_vector(robot.heading)
    }

    #[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
    enum Corner {
        FrontLeft,
        RearLeft,
        RearRight,
        FrontRight,
    }

    /// Steer front wheels and keep rear wheels fixed in neutral position.
    fn set_front_wheel_steering_angle(robot: &mut Robot, angle: Angle) {
        let _ = robot.set_wheel_steering_angle(WheelID::FrontLeft, angle);
        let _ = robot.set_wheel_steering_angle(WheelID::RearLeft, Angle::new(0.0));
        let _ = robot.set_wheel_steering_angle(WheelID::RearRight, Angle::new(0.0));
        let _ = robot.set_wheel_steering_angle(WheelID::FrontRight, angle);
    }

    /// Steer front and rear wheels in opposite directions. This enables tighter turns.
    fn set_all_wheel_steering_angle(robot: &mut Robot, angle: Angle) {
        let _ = robot.set_wheel_steering_angle(WheelID::FrontLeft, angle);
        let _ = robot.set_wheel_steering_angle(WheelID::RearLeft, -angle);
        let _ = robot.set_wheel_steering_angle(WheelID::RearRight, -angle);
        let _ = robot.set_wheel_steering_angle(WheelID::FrontRight, angle);
    }

    /// Steer front and rear wheels in the same direction. This enables movement without changing
    /// orientation.
    fn set_parallel_steering_angle(robot: &mut Robot, angle: Angle) {
        let _ = robot.set_wheel_steering_angle(WheelID::FrontLeft, angle);
        let _ = robot.set_wheel_steering_angle(WheelID::RearLeft, angle);
        let _ = robot.set_wheel_steering_angle(WheelID::RearRight, angle);
        let _ = robot.set_wheel_steering_angle(WheelID::FrontRight, angle);
    }

    fn set_velocity(robot: &mut Robot, velocity: Velocity) {
        let _ = robot.set_wheel_velocity(WheelID::FrontLeft, velocity);
        let _ = robot.set_wheel_velocity(WheelID::RearLeft, velocity);
        let _ = robot.set_wheel_velocity(WheelID::RearRight, velocity);
        let _ = robot.set_wheel_velocity(WheelID::FrontRight, velocity);
        let _ = robot.set_wheel_velocity(WheelID::CenterLeft, velocity);
        let _ = robot.set_wheel_velocity(WheelID::CenterRight, velocity);
    }

    fn cfg() -> RobotConfig {
        let a = 0.2;
        let b = 0.25;
        RobotConfig::new(
            a - 0.05,
            a - 0.05,
            b - 0.05,
            b - 0.05,
            a,
            a,
            b,
            b,
            0.0,
            0.05,
            0.05,
            Position::new(0.08, 0.0),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
    }
}
