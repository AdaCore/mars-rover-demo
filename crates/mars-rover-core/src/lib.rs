//! Bevy-free core of the Mars Rover simulator.
//!
//! Owns the Ada static-library link (`build.rs` invokes `alr build` and emits
//! the `-lMars_Rover -lgnat` directives), and exposes the pure-Rust domain
//! model, world factory, and glyph-to-waypoint helper that every driver
//! (GUI, headless, replay) shares.

pub mod domain;
pub mod glyph;
pub mod sidecar;
pub mod sim;
pub mod world;

#[cfg(test)]
pub mod tests;
