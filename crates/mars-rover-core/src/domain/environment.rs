//! Environment with obstacles.

use super::{Angle, HasCollision, Position, Shape};

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Environment {
    obstacles: Vec<Obstacle>,
}

impl Environment {
    pub fn new(obstacles: Vec<Obstacle>) -> Self {
        Self { obstacles }
    }

    pub fn obstacles(&self) -> &[Obstacle] {
        &self.obstacles
    }

    pub fn x_min(&self) -> f64 {
        self.obstacles
            .iter()
            .map(|o| o.bottom_left_corner().x())
            .min_by(|a, b| a.total_cmp(b))
            .unwrap_or_default()
    }

    pub fn x_max(&self) -> f64 {
        self.obstacles
            .iter()
            .map(|o| o.top_right_corner().x())
            .max_by(|a, b| a.total_cmp(b))
            .unwrap_or_default()
    }

    pub fn y_min(&self) -> f64 {
        self.obstacles
            .iter()
            .map(|o| o.bottom_left_corner().y())
            .min_by(|a, b| a.total_cmp(b))
            .unwrap_or_default()
    }

    pub fn y_max(&self) -> f64 {
        self.obstacles
            .iter()
            .map(|o| o.top_right_corner().y())
            .max_by(|a, b| a.total_cmp(b))
            .unwrap_or_default()
    }

    pub fn distance_to_next_obstacle(&self, position: Position, angle: Angle) -> Option<f64> {
        self.obstacles
            .iter()
            .flat_map(|o| {
                o.edges()
                    .iter()
                    .filter_map(|e| {
                        e.intersect_with_ray(position, angle)
                            .map(|i| position.distance(i))
                    })
                    .collect::<Vec<_>>()
            })
            .min_by(|a, b| a.total_cmp(b))
    }

    pub fn has_collision(&self, object: &dyn HasCollision) -> bool {
        self.obstacles().iter().any(|o| o.has_collision(object))
    }

    pub fn contains(&self, object: &dyn HasCollision) -> bool {
        match object.shape() {
            Shape::Circle { position, radius } => {
                position.x() - radius >= self.x_min()
                    && position.x() + radius <= self.x_max()
                    && position.y() - radius >= self.y_min()
                    && position.y() + radius <= self.y_max()
            }
            Shape::Rectangle {
                position,
                x_length,
                y_length,
            } => {
                position.x() - x_length / 2.0 >= self.x_min()
                    && position.x() + x_length / 2.0 <= self.x_max()
                    && position.y() - y_length / 2.0 >= self.y_min()
                    && position.y() + y_length / 2.0 <= self.y_max()
            }
        }
    }

    pub fn set_obstacle_position(&mut self, idx: usize, position: Position) {
        if let Some(obstacle) = self.obstacles.get_mut(idx) {
            obstacle.set_position(position)
        }
    }
}

#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct Obstacle {
    position: Position,
    x_length: f64,
    y_length: f64,
    movability: Movability,
}

impl Obstacle {
    pub fn new(position: Position, length: f64) -> Self {
        Self {
            position,
            x_length: length,
            y_length: length,
            movability: Movability::default(),
        }
    }

    pub fn with_movability(&self, movability: Movability) -> Self {
        Self {
            movability,
            ..*self
        }
    }

    pub fn position(&self) -> Position {
        self.position
    }

    pub fn set_position(&mut self, position: Position) {
        self.position = position;
    }

    pub fn x_length(&self) -> f64 {
        self.x_length
    }

    pub fn y_length(&self) -> f64 {
        self.y_length
    }

    pub fn is_movable(&self) -> bool {
        self.movability == Movability::Movable
    }

    pub fn bottom_left_corner(&self) -> Position {
        Position::new(
            self.position.x() - self.x_length / 2.0,
            self.position.y() - self.y_length / 2.0,
        )
    }

    pub fn bottom_right_corner(&self) -> Position {
        Position::new(
            self.position.x() + self.x_length / 2.0,
            self.position.y() - self.y_length / 2.0,
        )
    }

    pub fn top_left_corner(&self) -> Position {
        Position::new(
            self.position.x() - self.x_length / 2.0,
            self.position.y() + self.y_length / 2.0,
        )
    }

    pub fn top_right_corner(&self) -> Position {
        Position::new(
            self.position.x() + self.x_length / 2.0,
            self.position.y() + self.y_length / 2.0,
        )
    }

    fn edges(&self) -> Vec<LineSegment> {
        vec![
            self.bottom_edge(),
            self.right_edge(),
            self.top_edge(),
            self.left_edge(),
        ]
    }

    fn bottom_edge(&self) -> LineSegment {
        LineSegment::new(self.bottom_left_corner(), self.bottom_right_corner())
    }

    fn right_edge(&self) -> LineSegment {
        LineSegment::new(self.bottom_right_corner(), self.top_right_corner())
    }

    fn top_edge(&self) -> LineSegment {
        LineSegment::new(self.top_right_corner(), self.top_left_corner())
    }

    fn left_edge(&self) -> LineSegment {
        LineSegment::new(self.top_left_corner(), self.bottom_left_corner())
    }

    pub fn contains(&self, position: Position) -> bool {
        let x_min = self.position.x() - self.x_length / 2.0;
        let x_max = self.position.x() + self.x_length / 2.0;
        let y_min = self.position.y() - self.y_length / 2.0;
        let y_max = self.position.y() + self.y_length / 2.0;
        position.x() + f64::EPSILON >= x_min
            && position.x() - f64::EPSILON <= x_max
            && position.y() + f64::EPSILON >= y_min
            && position.y() - f64::EPSILON <= y_max
    }
}

impl HasCollision for Obstacle {
    fn shape(&self) -> Shape {
        Shape::Rectangle {
            position: self.position,
            x_length: self.x_length,
            y_length: self.y_length,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, PartialOrd)]
pub enum Movability {
    #[default]
    Unmovable,
    Movable,
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
struct LineSegment {
    p1: Position,
    p2: Position,
}

impl LineSegment {
    pub fn new(p1: Position, p2: Position) -> Self {
        Self { p1, p2 }
    }

    /// Check if a ray defined by a position and an angle intersects the line segment
    pub fn intersect_with_ray(&self, ray_origin: Position, angle: Angle) -> Option<Position> {
        // Compute the line's equation in the form of y = mx + b
        let m = (self.p2.y() - self.p1.y()) / (self.p2.x() - self.p1.x());
        let b = self.p1.y() - m * self.p1.x();

        // Compute the ray's slope from the angle
        let m_ray = Into::<f64>::into(angle).tan();

        // Check if the ray and line are parallel
        let dm = m_ray - m;

        if dm.abs() < f64::EPSILON {
            // Ray and line are parallel
            if (m * ray_origin.x() + b - ray_origin.y()).abs() < f64::EPSILON
                && self.contains(ray_origin)
            {
                // Ray origin is on line segment
                return Some(ray_origin);
            } else {
                return None;
            }
        }

        let intersection = if m.is_infinite() {
            // Line is vertical
            let b_ray = ray_origin.y() - m_ray * ray_origin.x();
            let x = self.p1.x();
            let y = m_ray * x + b_ray;
            Position::new(x, y)
        } else {
            let x = (m_ray * ray_origin.x() - ray_origin.y() + b) / dm;
            let y = m * x + b;
            Position::new(x, y)
        };

        // Check if the intersection point lies outside the line segment
        if !self.contains(intersection) {
            return None;
        }

        // Check if the intersection point lies in front of the ray
        let ray_dx = Into::<f64>::into(angle).cos();
        let ray_dy = Into::<f64>::into(angle).sin();

        let dx = intersection.x() - ray_origin.x();
        let dy = intersection.y() - ray_origin.y();

        let dot_product = dx * ray_dx + dy * ray_dy;

        if dot_product >= 0.0 {
            // Intersection is in front of the ray
            Some(intersection)
        } else {
            // Intersection is behind the ray
            None
        }
    }

    pub fn contains(&self, position: Position) -> bool {
        position.x() + f64::EPSILON >= self.p1.x().min(self.p2.x())
            && position.x() - f64::EPSILON <= self.p1.x().max(self.p2.x())
            && position.y() + f64::EPSILON >= self.p1.y().min(self.p2.y())
            && position.y() - f64::EPSILON <= self.p1.y().max(self.p2.y())
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use approx::{assert_abs_diff_eq, AbsDiffEq};
    use pretty_assertions::assert_eq;
    use rstest::rstest;

    use super::super::{Robot, RobotConfig};
    use super::*;
    use crate::tests::plot_line_chart;

    const EPSILON: f64 = 2.0 * f64::EPSILON;

    #[test]
    fn test_environment_distance_to_next_obstacle() {
        let environment = Environment::new(vec![obstacle_from_corners(
            Position::new(1.0, -1.0),
            Position::new(1.0, 1.0),
        )]);
        assert_abs_diff_eq!(
            environment
                .distance_to_next_obstacle(Position::new(0.0, 0.0), Angle::new(0.0))
                .unwrap(),
            1.0
        );
    }

    #[rstest]
    #[case(Position::new(1.0, 1.0))]
    #[case(Position::new(-1.0, 1.0))]
    #[case(Position::new(1.0, -1.0))]
    #[case(Position::new(-1.0, -1.0))]
    fn test_environment_has_collision(#[case] position: Position) {
        let mut robot = Robot::new(Angle::new(0.25 * PI), cfg());
        robot.set_position(position);
        let environment = Environment::new(vec![obstacle_from_corners(
            Position::new(-1.0, -1.0),
            Position::new(1.0, 1.0),
        )]);
        assert!(environment.has_collision(&robot));
    }

    #[test]
    fn test_obstacle() {
        let obstacles = vec![
            obstacle_from_corners(Position::new(-0.9, -0.9), Position::new(-0.6, -0.6)),
            obstacle_from_corners(Position::new(1.0, 1.0), Position::new(0.7, 0.8)),
            obstacle_from_corners(Position::new(-1.0, 0.1), Position::new(1.0, 0.1)),
            obstacle_from_corners(Position::new(0.2, -0.9), Position::new(0.2, 0.9)),
            obstacle_from_corners(Position::new(-0.3, 0.3), Position::new(-0.3, 0.3)),
        ];

        let mut obstacle_points = vec![];
        for o in obstacles {
            obstacle_points.push(vec![
                o.bottom_left_corner().into(),
                o.bottom_right_corner().into(),
                o.top_right_corner().into(),
                o.top_left_corner().into(),
                o.bottom_left_corner().into(),
            ]);
        }

        insta::assert_snapshot!(plot_line_chart(&obstacle_points));
    }

    #[test]
    fn test_obstacle_edges() {
        let obstacle = obstacle_from_corners(Position::new(0.1, 0.2), Position::new(0.3, 0.4));

        for (edge, expected) in std::iter::zip(
            obstacle.edges(),
            vec![
                LineSegment::new(Position::new(0.1, 0.2), Position::new(0.3, 0.2)),
                LineSegment::new(Position::new(0.3, 0.2), Position::new(0.3, 0.4)),
                LineSegment::new(Position::new(0.3, 0.4), Position::new(0.1, 0.4)),
                LineSegment::new(Position::new(0.1, 0.4), Position::new(0.1, 0.2)),
            ],
        ) {
            assert_abs_diff_eq!(edge, expected);
        }
    }

    #[rstest]
    #[case::intersection_in_front_of_ray(
        LineSegment::new(Position::new(0.0, 0.0), Position::new(2.0, 2.0)),
        Position::new(1.0, 0.0),
        Angle::new(0.75 * PI),
        Some(Position::new(0.5, 0.5))
    )]
    #[case::intersection_behind_ray(
        LineSegment::new(Position::new(0.0, 0.0), Position::new(2.0, 2.0)),
        Position::new(0.0, 1.0),
        Angle::new(0.75 * PI),
        None
    )]
    #[case::intersection_not_on_line_segment(
        LineSegment::new(Position::new(0.0, 0.0), Position::new(2.0, 2.0)),
        Position::new(1.0, -2.0),
        Angle::new(0.75 * PI),
        None
    )]
    #[case::ray_origin_on_line_segment(
        LineSegment::new(Position::new(0.0, 0.0), Position::new(2.0, 2.0)),
        Position::new(0.5, 0.5),
        Angle::new(0.5 * PI),
        Some(Position::new(0.5, 0.5))
    )]
    #[case::ray_origin_not_on_line_segment(
        LineSegment::new(Position::new(0.0, 0.0), Position::new(2.0, 2.0)),
        Position::new(-0.5, -0.5),
        Angle::new(0.25 * PI),
        None
    )]
    #[case::identical(
        LineSegment::new(Position::new(0.0, 0.0), Position::new(2.0, 2.0)),
        Position::new(1.0, 1.0),
        Angle::new(0.25 * PI),
        Some(Position::new(1.0, 1.0))
    )]
    #[case::parallel(
        LineSegment::new(Position::new(0.0, 0.0), Position::new(2.0, 2.0)),
        Position::new(0.0, 1.0),
        Angle::new(0.25 * PI),
        None
    )]
    #[case::vertical_line_segment_right(
        LineSegment::new(Position::new(1.0, 1.0), Position::new(1.0, -1.0)),
        Position::new(0.0, 1.0),
        Angle::new(0.0),
        Some(Position::new(1.0, 1.0))
    )]
    #[case::vertical_line_segment_left(
        LineSegment::new(Position::new(-1.0, 1.0), Position::new(-1.0, -1.0)),
        Position::new(-0.8, 0.0),
        Angle::new(3.0/4.0 * PI),
        Some(Position::new(-1.0, 0.2))
    )]
    #[case::vertical_line_segment_left_corner(
        LineSegment::new(Position::new(-1.0, 1.0), Position::new(-1.0, -1.0)),
        Position::new(0.0, 0.0),
        Angle::new(3.0/4.0 * PI),
        Some(Position::new(-1.0, 1.0))
    )]
    #[case::vertical_line_segment_left_behind(
        LineSegment::new(Position::new(-1.0, 1.0), Position::new(-1.0, -1.0)),
        Position::new(-2.0, 0.0),
        Angle::new(3.0/4.0 * PI),
        None
    )]
    #[case::vertical_ray(
        LineSegment::new(Position::new(-2.0, 1.0), Position::new(2.0, 1.0)),
        Position::new(1.0, 0.0),
        Angle::new(0.5 * PI),
        Some(Position::new(1.0, 1.0))
    )]
    fn test_line_segment_intersect_with_ray(
        #[case] line: LineSegment,
        #[case] position: Position,
        #[case] angle: Angle,
        #[case] intersection: Option<Position>,
    ) {
        let result = line.intersect_with_ray(position, angle);
        if let (Some(r), Some(i)) = (result, intersection) {
            assert_abs_diff_eq!(r, i, epsilon = EPSILON);
        } else {
            assert_eq!(result, intersection);
        }
    }

    pub fn obstacle_from_corners(c_1: Position, c_2: Position) -> Obstacle {
        let x_min = c_1.x().min(c_2.x());
        let x_max = c_1.x().max(c_2.x());
        let y_min = c_1.y().min(c_2.y());
        let y_max = c_1.y().max(c_2.y());
        let x_length = x_max - x_min;
        let y_length = y_max - y_min;
        let position = Position::new(x_min + x_length / 2.0, y_min + y_length / 2.0);
        Obstacle {
            position,
            x_length,
            y_length,
            movability: Movability::default(),
        }
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

    impl AbsDiffEq for LineSegment {
        type Epsilon = f64;

        fn default_epsilon() -> f64 {
            f64::EPSILON
        }

        fn abs_diff_eq(&self, other: &Self, epsilon: f64) -> bool {
            Position::abs_diff_eq(&self.p1, &other.p1, epsilon)
                && Position::abs_diff_eq(&self.p2, &other.p2, epsilon)
        }
    }
}
