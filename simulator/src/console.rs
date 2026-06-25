//! Quake-style sliding console showing the rover's action log.
//!
//! Press `C` to toggle. The panel slides up from the bottom-left, occupying ~33% of the window
//! width. Messages are colour-coded: green = moving forward, red = obstacle/turning around,
//! orange = scanning or manoeuvring. Older entries are shown in grey.

use std::collections::VecDeque;

use bevy::prelude::*;

use crate::{controller, controller::drain_new_messages, resource::{RobotRes, SlipFreeMode}};

const CONSOLE_HEIGHT: f32 = 200.0;
const MAX_LOG_LINES: usize = 50;
const VISIBLE_LINES: usize = 8;
const ANIMATION_SPEED: f32 = 10.0;

#[derive(Resource, Default)]
pub struct ConsoleState {
    pub visible: bool,
    /// Animation progress: 0.0 = fully hidden (off-screen below), 1.0 = fully shown.
    pub animation: f32,
    pub log: VecDeque<(String, Color)>,
}

#[derive(Component)]
struct ConsolePanel;

#[derive(Component)]
struct ConsoleText;

#[derive(Component)]
struct GncStatusText;

fn classify_color(msg: &str) -> Color {
    if msg.starts_with("Going forward")
        || msg.starts_with("Remote: Forward")
        || msg.starts_with("Remote: Fwd")
    {
        Color::rgb(0.2, 0.85, 0.2) // green — moving forward
    } else if msg.starts_with("Turning around") {
        Color::rgb(0.9, 0.2, 0.2) // red — obstacle blocked
    } else {
        Color::rgb(0.9, 0.55, 0.1) // orange — scanning / manoeuvring
    }
}

fn setup_console(mut commands: Commands) {
    commands
        .spawn((
            NodeBundle {
                style: Style {
                    position_type: PositionType::Absolute,
                    bottom: Val::Px(-CONSOLE_HEIGHT),
                    left: Val::Px(0.0),
                    width: Val::Percent(33.0),
                    height: Val::Px(CONSOLE_HEIGHT),
                    padding: UiRect::all(Val::Px(8.0)),
                    flex_direction: FlexDirection::Column,
                    justify_content: JustifyContent::FlexEnd,
                    overflow: Overflow::clip(),
                    ..default()
                },
                background_color: BackgroundColor(Color::rgba(0.05, 0.05, 0.05, 0.75)),
                z_index: ZIndex::Global(100),
                ..default()
            },
            ConsolePanel,
        ))
        .with_children(|parent| {
            parent.spawn((
                TextBundle {
                    text: Text::from_sections(vec![]),
                    ..default()
                },
                ConsoleText,
            ));

            // Thin separator between the log and the GNC status line.
            parent.spawn(NodeBundle {
                style: Style {
                    width: Val::Percent(100.0),
                    height: Val::Px(1.0),
                    margin: UiRect::vertical(Val::Px(4.0)),
                    ..default()
                },
                background_color: BackgroundColor(Color::rgba(0.4, 0.4, 0.4, 0.6)),
                ..default()
            });

            // GNC live status: truth vs Ada sensor readings, updated every frame.
            parent.spawn((
                TextBundle {
                    text: Text::from_sections(vec![
                        TextSection::new("", TextStyle {
                            font_size: 13.0,
                            color: Color::rgba(0.5, 0.85, 1.0, 0.9),
                            ..default()
                        }),
                    ]),
                    ..default()
                },
                GncStatusText,
            ));
        });
}

fn toggle_console(keys: Res<ButtonInput<KeyCode>>, mut state: ResMut<ConsoleState>) {
    if keys.just_pressed(KeyCode::KeyC) {
        state.visible = !state.visible;
    }
}

fn animate_console(
    time: Res<Time>,
    mut state: ResMut<ConsoleState>,
    mut panels: Query<&mut Style, With<ConsolePanel>>,
) {
    let target = if state.visible { 1.0_f32 } else { 0.0 };
    let diff = target - state.animation;
    if diff.abs() > 0.001 {
        let step = diff.signum() * (time.delta_seconds() * ANIMATION_SPEED).min(diff.abs());
        state.animation += step;
    } else {
        state.animation = target;
    }

    // animation=0 → bottom=-CONSOLE_HEIGHT (hidden), animation=1 → bottom=0 (fully visible)
    let bottom_px = (state.animation - 1.0) * CONSOLE_HEIGHT;
    for mut style in panels.iter_mut() {
        style.bottom = Val::Px(bottom_px);
    }
}

fn poll_and_update_console(
    mut state: ResMut<ConsoleState>,
    mut texts: Query<&mut Text, With<ConsoleText>>,
) {
    let new_messages = drain_new_messages();
    if new_messages.is_empty() {
        return;
    }

    for msg in new_messages {
        let color = classify_color(&msg);
        state.log.push_back((msg, color));
        while state.log.len() > MAX_LOG_LINES {
            state.log.pop_front();
        }
    }

    let grey = Color::rgba(0.5, 0.5, 0.5, 0.8);
    let log_len = state.log.len();
    let start = log_len.saturating_sub(VISIBLE_LINES);

    let sections: Vec<TextSection> = state
        .log
        .iter()
        .skip(start)
        .enumerate()
        .map(|(i, (msg, action_color))| {
            let is_newest = (start + i) == log_len - 1;
            TextSection {
                value: format!("{}\n", msg),
                style: TextStyle {
                    font_size: 14.0,
                    color: if is_newest { *action_color } else { grey },
                    ..default()
                },
            }
        })
        .collect();

    for mut text in texts.iter_mut() {
        text.sections = sections.clone();
    }
}

fn update_gnc_status(
    mut texts: Query<&mut Text, With<GncStatusText>>,
    robot: Res<RobotRes>,
    slip_free: Res<SlipFreeMode>,
) {
    let mut text = texts.single_mut();

    let pos         = robot.position();
    // Normalise truth heading to (−π, π] to match Ada's EKF theta convention.
    // Rust's `%` can produce negative heading (CW turns from near 0), so we
    // must handle the full range, not just (π, 2π).
    let heading_raw = f64::from(robot.heading()) as f32;
    let tau = 2.0 * std::f32::consts::PI;
    let heading = {
        let h = ((heading_raw % tau) + tau) % tau; // map to [0, 2π)
        if h > std::f32::consts::PI { h - tau } else { h }
    };
    let t       = controller::encoder_truth_total();
    let gnc     = controller::gnc_state();
    let ekf     = controller::ekf_estimated_state();

    // Encoder rows (6-char columns, +sign + up to 5 digits).
    let enc_truth = format!("{:+6} {:+6} {:+6} {:+6}", t[0], t[1], t[2], t[3]);
    let enc_ada = match &gnc {
        Some(s) => format!("{:+6} {:+6} {:+6} {:+6}", s.enc[0], s.enc[1], s.enc[2], s.enc[3]),
        None    => "   ---    ---    ---    ---".to_string(),
    };

    // GPS rows.
    let gps_ada = match &gnc {
        Some(s) => format!("{:+7.2},{:+7.2}", s.est_x, s.est_y),
        None    => "    ---,    ---".to_string(),
    };

    // EKF rows (position + heading in degrees).
    let ekf_est = match ekf {
        Some((ex, ey, et)) => format!(
            "{:+7.2},{:+7.2}  {:+6.1} deg",
            ex, ey, et.to_degrees()
        ),
        None => "    ---,    ---     --- deg".to_string(),
    };

    let slip_line = if slip_free.0 { "\n[F] SLIP-FREE" } else { "" };
    text.sections[0].value = format!(
"                FL     FR     RL     RR
ENC  truth  {enc_truth}
       ada  {enc_ada}

                  x,      y
GPS  truth  {:+7.2},{:+7.2}
       ada  {gps_ada}

                  x,      y,  theta
EKF  true   {:+7.2},{:+7.2}  {:+6.1} deg
      est   {ekf_est}{slip_line}",
        pos.x() as f32, pos.y() as f32,
        pos.x() as f32, pos.y() as f32, heading.to_degrees(),
    );
}

pub struct ConsolePlugin;

impl Plugin for ConsolePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<ConsoleState>()
            .add_systems(Startup, setup_console)
            .add_systems(
                Update,
                (toggle_console, animate_console, poll_and_update_console, update_gnc_status),
            );
    }
}
