//! Controller steering the robot.
//!
//! The robot is controlled by either the Ada-based controller or the keyboard. The Ada control software is used by default and takes control if no key is pressed on the keyboard for a few seconds.

use bevy::prelude::*;

use std::{
    sync::atomic::{AtomicI8, AtomicU16, AtomicU32, Ordering},
    thread,
    time::{Duration, Instant},
};

use once_cell::sync::Lazy;

use crate::{
    domain::{Angle, Velocity, WheelID},
    resource::RobotRes,
};

static EPOCH: Lazy<Instant> = Lazy::new(Instant::now);
static DISTANCE_SENSOR_DISTANCE: AtomicU32 = AtomicU32::new(0);
static DISTANCE_SENSOR_ANGLE: AtomicI8 = AtomicI8::new(0);
static WHEEL_ANGLE: [AtomicI8; 4] = [
    AtomicI8::new(0),
    AtomicI8::new(0),
    AtomicI8::new(0),
    AtomicI8::new(0),
];
static LEFT_WHEEL_POWER: AtomicI8 = AtomicI8::new(0);
static RIGHT_WHEEL_POWER: AtomicI8 = AtomicI8::new(0);
static CONTROLLER_BUTTONS_STATE: AtomicU16 = AtomicU16::new(0);

// Import of externally defined functions

extern "C" {
    fn Mars_Roverinit();
    fn Mars_Roverfinal();
    fn mars_rover_demo_task();
}

// Export of externally required functions

#[no_mangle]
pub extern "C" fn mars_rover_clock() -> u64 {
    (Instant::now() - *EPOCH).as_micros() as u64
}

#[no_mangle]
pub extern "C" fn mars_rover_delay_microseconds(us: u16) {
    thread::sleep(Duration::from_micros(us as u64));
}

#[no_mangle]
pub extern "C" fn mars_rover_delay_milliseconds(ms: u16) {
    thread::sleep(Duration::from_millis(ms as u64));
}

#[no_mangle]
pub extern "C" fn mars_rover_sonar_distance() -> u32 {
    DISTANCE_SENSOR_DISTANCE.load(Ordering::Relaxed)
}

#[no_mangle]
pub extern "C" fn mars_rover_set_mast_angle(a: i8) {
    DISTANCE_SENSOR_ANGLE.store(a, Ordering::Relaxed)
}

#[no_mangle]
pub extern "C" fn mars_rover_set_wheel_angle(wheel: u8, side: u8, a: i8) {
    let idx = match (wheel, side) {
        (0, 0) => WheelID::FrontLeft as usize,
        (1, 0) => WheelID::RearLeft as usize,
        (1, 1) => WheelID::RearRight as usize,
        (0, 1) => WheelID::FrontRight as usize,
        _ => usize::MAX,
    };
    if idx < usize::MAX {
        WHEEL_ANGLE[idx].store(a, Ordering::Relaxed);
    }
}

#[no_mangle]
pub extern "C" fn mars_rover_set_power(side: u8, power: i8) {
    if side == 0 {
        LEFT_WHEEL_POWER.store(power, Ordering::Relaxed);
    } else if side == 1 {
        RIGHT_WHEEL_POWER.store(power, Ordering::Relaxed);
    }
}

#[no_mangle]
pub extern "C" fn mars_rover_controller_state() -> u16 {
    CONTROLLER_BUTTONS_STATE.load(Ordering::Relaxed)
}

pub struct Controller;

fn spawn() {
    let _ = thread::spawn(move || unsafe {
        Mars_Roverinit();
        mars_rover_demo_task();
        Mars_Roverfinal();
    });
}

fn control(keys: Res<ButtonInput<KeyCode>>, mut robot: ResMut<RobotRes>) {
    const POWER_TO_VELOCITY_FACTOR: f64 = 1.0 / 350.0;

    for (i, angle) in WHEEL_ANGLE.iter().enumerate() {
        if let Ok(wheel_id) = WheelID::try_from(i) {
            let _ = robot.set_wheel_steering_angle(
                wheel_id,
                Angle::from_deg(angle.load(Ordering::Relaxed).into()),
            );
        };
    }

    let left_wheel_velocity =
        Velocity::new(LEFT_WHEEL_POWER.load(Ordering::Relaxed) as f64 * POWER_TO_VELOCITY_FACTOR);
    let _ = robot.set_wheel_velocity(WheelID::FrontLeft, left_wheel_velocity);
    let _ = robot.set_wheel_velocity(WheelID::CenterLeft, left_wheel_velocity);
    let _ = robot.set_wheel_velocity(WheelID::RearLeft, left_wheel_velocity);

    let right_wheel_velocity =
        Velocity::new(RIGHT_WHEEL_POWER.load(Ordering::Relaxed) as f64 * POWER_TO_VELOCITY_FACTOR);
    let _ = robot.set_wheel_velocity(WheelID::FrontRight, right_wheel_velocity);
    let _ = robot.set_wheel_velocity(WheelID::CenterRight, right_wheel_velocity);
    let _ = robot.set_wheel_velocity(WheelID::RearRight, right_wheel_velocity);

    DISTANCE_SENSOR_DISTANCE.store(
        if let Some(distance) = robot.distance_sensor_distance() {
            (distance * 100.0) as u32
        } else {
            u32::MAX
        },
        Ordering::Relaxed,
    );

    robot.set_distance_sensor_angle(-Angle::from_deg(
        DISTANCE_SENSOR_ANGLE.load(Ordering::Relaxed).into(),
    ));

    let mut raw_keys_state: u16 = 0;
    if keys.pressed(KeyCode::ArrowUp) {
        raw_keys_state |= 0x0001;
    }
    if keys.pressed(KeyCode::ArrowDown) {
        raw_keys_state |= 0x0002;
    }
    if keys.pressed(KeyCode::ArrowLeft) {
        raw_keys_state |= 0x0004;
    }
    if keys.pressed(KeyCode::ArrowRight) {
        raw_keys_state |= 0x0008;
    }

    CONTROLLER_BUTTONS_STATE.store(raw_keys_state, Ordering::Relaxed);
}

impl Plugin for Controller {
    fn build(&self, app: &mut App) {
        spawn();

        app.add_systems(Update, control);
    }
}
