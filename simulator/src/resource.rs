//! The resource module encapsulates domain entities for use with Bevy.

use std::ops::{Deref, DerefMut};

use bevy::ecs::{schedule::SystemSet, system::Resource};

use mars_rover_core::{domain, sim};

#[derive(Resource)]
pub struct RobotRes(domain::Robot);

impl Deref for RobotRes {
    type Target = domain::Robot;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for RobotRes {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl From<domain::Robot> for RobotRes {
    fn from(value: domain::Robot) -> Self {
        Self(value)
    }
}

#[derive(Resource)]
pub struct EnvironmentRes(domain::Environment);

impl Deref for EnvironmentRes {
    type Target = domain::Environment;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for EnvironmentRes {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl From<domain::Environment> for EnvironmentRes {
    fn from(value: domain::Environment) -> Self {
        Self(value)
    }
}

#[derive(Resource)]
pub struct TerrainRes(domain::Terrain);

impl Deref for TerrainRes {
    type Target = domain::Terrain;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl From<domain::Terrain> for TerrainRes {
    fn from(value: domain::Terrain) -> Self {
        Self(value)
    }
}

/// Carries the RNG seed chosen at startup so plugins can use it.
#[derive(Resource)]
pub struct TerrainSeed(pub u64);

/// When enabled, per-wheel slip is forced to zero (traction = 1.0 for all wheels).
/// Toggle with the `F` key.  Use this to verify EKF correctness against ideal kinematics.
#[derive(Resource, Default)]
pub struct SlipFreeMode(pub bool);

/// Waypoints for the rover to follow, in world-space (x, y) metres.
/// Populated from `--waypoints <file>` or `--text <string>` at startup.
/// Empty when neither argument was given.  Read-only after insertion.
#[derive(Resource, Default)]
pub struct WaypointsRes(pub Vec<(f64, f64)>);

/// Controls whether the waypoint path is rendered in the viewport.
/// Toggle with the `P` key.  Off by default.
#[derive(Resource, Default)]
pub struct ShowWaypointPath(pub bool);

/// Bevy wrapper around core's `SimInputs` queue.  Systems push via `.0.push(...)`;
/// the `apply_sim_inputs` system drains each frame.
#[derive(Resource, Default)]
pub struct SimInputsRes(pub sim::SimInputs);

/// Ordering for the per-frame sim step.  `apply_sim_inputs` runs first so
/// user/control-plane events (drag-drop, R-key, F-key) affect the same frame's
/// physics tick; `simulate` advances physics; `control` then runs the Ada
/// controller with the already-updated robot state.
#[derive(SystemSet, Debug, Hash, PartialEq, Eq, Clone)]
pub enum SimStep {
    ApplyInputs,
    Simulate,
    Control,
}
