use bevy::prelude::*;

#[cfg(test)]
#[macro_use]
mod tests;

mod controller;
mod domain;
mod resource;
mod simulator;
mod visualizer;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(controller::Controller)
        .add_plugins(visualizer::Visualizer)
        .add_plugins(simulator::Simulator)
        .run();
}
