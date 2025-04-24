//! Collision detection based on basic shapes.

use super::Position;

pub trait HasCollision {
    fn has_collision(&self, other: &dyn HasCollision) -> bool {
        self.shape().has_intersection(&other.shape())
    }

    fn shape(&self) -> Shape;
}

pub enum Shape {
    Rectangle {
        position: Position,
        x_length: f64,
        y_length: f64,
    },
    Circle {
        position: Position,
        radius: f64,
    },
}

impl Shape {
    fn has_intersection(&self, other: &Shape) -> bool {
        match (self, other) {
            (
                Shape::Circle { position, radius },
                Shape::Circle {
                    position: other_position,
                    radius: other_radius,
                },
            ) => position.distance(*other_position) < radius + other_radius,
            (
                Shape::Circle { position, radius },
                Shape::Rectangle {
                    position: other_position,
                    x_length: other_x_length,
                    y_length: other_y_length,
                },
            )
            | (
                Shape::Rectangle {
                    position: other_position,
                    x_length: other_x_length,
                    y_length: other_y_length,
                },
                Shape::Circle { position, radius },
            ) => {
                let x_min = other_position.x() - other_x_length / 2.0;
                let x_max = other_position.x() + other_x_length / 2.0;
                let y_min = other_position.y() - other_y_length / 2.0;
                let y_max = other_position.y() + other_y_length / 2.0;
                [
                    Position::new(x_min, y_min),
                    Position::new(x_max, y_min),
                    Position::new(x_min, y_max),
                    Position::new(x_max, y_max),
                ]
                .iter()
                .any(|p| position.distance(*p) < *radius)
            }
            (
                Shape::Rectangle {
                    position,
                    x_length,
                    y_length,
                },
                Shape::Rectangle {
                    position: other_position,
                    x_length: other_x_length,
                    y_length: other_y_length,
                },
            ) => {
                let x_min = position.x() - x_length / 2.0;
                let x_max = position.x() + x_length / 2.0;
                let y_min = position.y() - y_length / 2.0;
                let y_max = position.y() + y_length / 2.0;
                let other_x_min = other_position.x() - other_x_length / 2.0;
                let other_x_max = other_position.x() + other_x_length / 2.0;
                let other_y_min = other_position.y() - other_y_length / 2.0;
                let other_y_max = other_position.y() + other_y_length / 2.0;
                [
                    Position::new(x_min, y_min),
                    Position::new(x_max, y_min),
                    Position::new(x_min, y_max),
                    Position::new(x_max, y_max),
                ]
                .iter()
                .any(|p| {
                    p.x() + f64::EPSILON >= other_x_min
                        && p.x() - f64::EPSILON <= other_x_max
                        && p.y() + f64::EPSILON >= other_y_min
                        && p.y() - f64::EPSILON <= other_y_max
                })
            }
        }
    }
}
