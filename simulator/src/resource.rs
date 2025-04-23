//! The resource module encapsulates domain entities for use with Bevy.

use std::ops::{Deref, DerefMut};

use bevy::ecs::system::Resource;

use crate::domain;

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
