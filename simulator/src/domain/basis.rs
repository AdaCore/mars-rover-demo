//! Basic building blocks.

use std::{
    f64::consts::PI,
    ops::{Add, Neg},
};

#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct Position {
    x: f64,
    y: f64,
}

impl Position {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn x(&self) -> f64 {
        self.x
    }

    pub fn y(&self) -> f64 {
        self.y
    }

    pub fn distance(&self, position: Self) -> f64 {
        ((self.x - position.x).powi(2) + (self.y - position.y).powi(2)).sqrt()
    }

    pub fn rotate_vector(&self, angle: Angle) -> Position {
        Position::new(
            self.x * angle.0.cos() - self.y * angle.0.sin(),
            self.x * angle.0.sin() + self.y * angle.0.cos(),
        )
    }
}

impl From<Position> for (f32, f32) {
    fn from(value: Position) -> Self {
        (value.x as f32, value.y as f32)
    }
}

impl From<Position> for (f64, f64) {
    fn from(value: Position) -> Self {
        (value.x, value.y)
    }
}

impl Add for Position {
    type Output = Position;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct Angle(f64);

impl Angle {
    pub fn new(radians: f64) -> Self {
        Self(radians)
    }

    pub fn from_deg(degree: f64) -> Self {
        Self(degree * PI / 180.0)
    }

    pub fn to_deg(self) -> f64 {
        (self.0 * (180.0 / PI) + 360.0) % 360.0
    }
}

impl Neg for Angle {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Angle(-self.0)
    }
}

impl Add for Angle {
    type Output = Angle;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl From<Angle> for f64 {
    fn from(value: Angle) -> Self {
        value.0
    }
}

impl From<Angle> for f32 {
    fn from(value: Angle) -> Self {
        value.0 as f32
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct Velocity(f64);

impl Velocity {
    pub fn new(velocity: f64) -> Self {
        Self(velocity)
    }
}

impl Add for Velocity {
    type Output = Velocity;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl From<Velocity> for f64 {
    fn from(value: Velocity) -> Self {
        value.0
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use approx::{assert_abs_diff_eq, AbsDiffEq};
    use rstest::rstest;

    use super::*;

    #[test]
    fn test_position() {
        let position = Position::new(1.0, 2.0);
        assert_abs_diff_eq!(position.x(), 1.0);
        assert_abs_diff_eq!(position.y(), 2.0);
    }

    #[rstest]
    #[case(Angle::new(0.0), 0.0)]
    #[case(Angle::new(0.5 * PI), 90.0)]
    #[case(Angle::new(1.0 * PI), 180.0)]
    #[case(Angle::new(1.5 * PI), 270.0)]
    #[case(Angle::new(2.0 * PI), 0.0)]
    fn test_angle_to_deg(#[case] angle: Angle, #[case] expected: f64) {
        assert_abs_diff_eq!(angle.to_deg(), expected);
    }

    impl AbsDiffEq for Position {
        type Epsilon = f64;

        fn default_epsilon() -> f64 {
            f64::EPSILON
        }

        fn abs_diff_eq(&self, other: &Self, epsilon: f64) -> bool {
            f64::abs_diff_eq(&self.x, &other.x, epsilon)
                && f64::abs_diff_eq(&self.y, &other.y, epsilon)
        }
    }

    impl AbsDiffEq for Angle {
        type Epsilon = f64;

        fn default_epsilon() -> f64 {
            f64::EPSILON
        }

        fn abs_diff_eq(&self, other: &Self, epsilon: f64) -> bool {
            f64::abs_diff_eq(&self.0, &other.0, epsilon)
        }
    }
}
