//! The domain module encapsulates the core business logic. It defines the `Robot` and
//! `Environment` entities, along with the rules governing their interactions.
//!
//! By minimizing hard dependencies, this module ensures the business logic remains adaptable and
//! independent of specific implementation details.

mod basis;
mod collision;
mod environment;
mod robot;

pub use basis::{Angle, Position, Velocity};
pub use collision::{HasCollision, Shape};
pub use environment::{Environment, Movability, Obstacle};
pub use robot::{Robot, RobotConfig, WheelID};
