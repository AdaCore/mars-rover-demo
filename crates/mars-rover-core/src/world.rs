//! Constructs the authoritative simulation state (`Robot`, `Environment`,
//! `Terrain`) at startup.
//!
//! This module is Bevy-free.  Both the GUI and (future) headless binaries
//! build the world here; the resources are then wrapped by the GUI's Bevy
//! `Resource` types in `resource.rs`.

use rand::{
    distr::{Distribution, Uniform},
    SeedableRng,
};
use rand_chacha::ChaCha8Rng;

use crate::domain::{
    Angle, Environment, Movability, Obstacle, Position, Robot, RobotConfig, Terrain,
};

/// Seed for deterministic obstacle layout (and rendering decorations —
/// pebbles, dust — that reuse the same stream in the visualizer).
pub const RNG_SEED: u64 = 19878367467712;

/// Width of the traversable world in metres.  Increase here (and adjust
/// `X_MAX` in `build_environment`) to grow the world.
pub const TERRAIN_WORLD_WIDTH: f64 = 10.0;
/// Height of the traversable world in metres.  See `TERRAIN_WORLD_WIDTH`.
pub const TERRAIN_WORLD_HEIGHT: f64 = 6.0;

const TERRAIN_CELL_SIZE: f64 = 0.05;
const TERRAIN_NOISE_SCALE: f64 = 2.0;
const TERRAIN_AMPLITUDE: f64 = 0.015;
const TERRAIN_T_MIN: f64 = 0.3;
const TERRAIN_MAX_SLOPE: f64 = 0.04;

/// The authoritative simulation state at startup.  All three fields are
/// plain domain types — no Bevy resource wrappers — so this struct can be
/// constructed from a headless driver as well.
pub struct World {
    pub robot: Robot,
    pub environment: Environment,
    pub terrain: Terrain,
}

impl World {
    /// Build the world.  `terrain_seed` drives the terrain heightmap;
    /// obstacle layout and visual decorations are driven by the fixed
    /// `RNG_SEED` above.
    pub fn build(terrain_seed: u64) -> Self {
        Self {
            robot: build_robot(),
            environment: build_environment(),
            terrain: build_terrain(terrain_seed),
        }
    }
}

fn build_robot() -> Robot {
    Robot::new(
        Angle::new(0.0),
        RobotConfig::new(
            0.066,
            0.049,
            0.0385,
            0.0385,
            0.08,
            0.075,
            0.0565,
            0.0565,
            0.028,
            0.0225,
            0.017,
            Position::new(0.0375, -0.0125),
            0.018,
            0.045,
            0.027,
            0.018,
            0.007,
        ),
    )
}

fn build_environment() -> Environment {
    const X_MAX: i32 = 49;
    const Y_MAX: i32 = 29;
    const LENGTH: f64 = 0.1;
    const HALF_LENGTH: f64 = 0.05;

    let mut rng = ChaCha8Rng::seed_from_u64(RNG_SEED);
    let position_offset = Uniform::try_from(-HALF_LENGTH..=HALF_LENGTH).unwrap();
    let length_offset = Uniform::try_from(0.0..=LENGTH).unwrap();

    let mut obstacles = vec![];

    for x in -X_MAX + 1..=X_MAX - 1 {
        for y in [
            (-Y_MAX - 1) as f64 / 10.0,
            -Y_MAX as f64 / 10.0,
            Y_MAX as f64 / 10.0,
            (Y_MAX + 1) as f64 / 10.0,
        ] {
            obstacles.push(Obstacle::new(
                Position::new(
                    x as f64 / 10.0 + position_offset.sample(&mut rng),
                    y + position_offset.sample(&mut rng),
                ),
                LENGTH + length_offset.sample(&mut rng),
            ));
        }
    }

    for x in [
        (-X_MAX - 1) as f64 / 10.0,
        -X_MAX as f64 / 10.0,
        X_MAX as f64 / 10.0,
        (X_MAX + 1) as f64 / 10.0,
    ] {
        for y in -Y_MAX + 1..=Y_MAX - 1 {
            obstacles.push(Obstacle::new(
                Position::new(
                    x + position_offset.sample(&mut rng),
                    y as f64 / 10.0 + position_offset.sample(&mut rng),
                ),
                LENGTH + length_offset.sample(&mut rng),
            ));
        }
    }

    for (x, y) in [(-0.8, -0.25), (0.8, 0.25)] {
        obstacles
            .push(Obstacle::new(Position::new(x, y), 0.1).with_movability(Movability::Movable));
    }

    Environment::new(obstacles)
}

fn build_terrain(seed: u64) -> Terrain {
    Terrain::new(
        seed,
        TERRAIN_WORLD_WIDTH,
        TERRAIN_WORLD_HEIGHT,
        TERRAIN_CELL_SIZE,
        TERRAIN_NOISE_SCALE,
        TERRAIN_AMPLITUDE,
        TERRAIN_T_MIN,
        TERRAIN_MAX_SLOPE,
    )
}
