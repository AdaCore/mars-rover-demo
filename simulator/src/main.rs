use bevy::prelude::*;

use mars_rover_core::{glyph, sidecar, world};
use resource::{
    EnvironmentRes, RobotRes, SimInputsRes, SimStep, TerrainRes, TerrainSeed, WaypointsRes,
};

mod console;
mod controller;
mod logger;
mod replay_mode;
mod replay_source;
mod resource;
mod ribbon;
mod simulator;
mod visualizer;

struct Args {
    seed: u64,
    gps_interval_ms: u32,
    log_path: Option<std::path::PathBuf>,
    waypoint_path: Option<std::path::PathBuf>,
    text: Option<String>,
    /// When set, the GUI runs as a trace-driven renderer over the given CSV
    /// instead of spawning the Ada controller + live physics.  The sidecars
    /// `<path>.meta` (terrain seed) and `<path>.wpts` (waypoint path) are
    /// auto-loaded.  Distinct from the `mars-rover-replay` binary, which is
    /// an FFI-level regression harness for the Ada control software.
    replay_path: Option<std::path::PathBuf>,
}

fn parse_args() -> Args {
    const DEFAULT_SEED: u64 = 42;
    const DEFAULT_GPS_INTERVAL_MS: u32 = 500;

    let args: Vec<String> = std::env::args().collect();
    let prog = args.first().map(String::as_str).unwrap_or("mars-rover");

    let mut seed = DEFAULT_SEED;
    let mut gps_interval_ms = DEFAULT_GPS_INTERVAL_MS;
    let mut log_path = None;
    let mut waypoint_path = None;
    let mut text = None;
    let mut replay_path: Option<std::path::PathBuf> = None;
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "-h" | "--help" => {
                println!(
                    "Usage: {prog} [OPTIONS]\n\
                     \n\
                     Options:\n\
                     \x20 --seed <u64>          Terrain generation seed [default: {DEFAULT_SEED}]\n\
                     \x20 --gps-interval <u32>  GPS update interval in ms; 0 = disabled [default: {DEFAULT_GPS_INTERVAL_MS}]\n\
                     \x20 --log <path>          Write per-frame CSV diagnostic log to <path>\n\
                     \x20 --waypoints <path>    Waypoint file (x y per line); omit for no path\n\
                     \x20 --text <string>       Generate rover waypoints that trace the given text\n\
                     \x20 --replay <path>       Render a recorded trace instead of running a live\n\
                     \x20                       sim.  Reads <path>.meta for the terrain seed and\n\
                     \x20                       <path>.wpts for the waypoint path; no Ada thread\n\
                     \x20                       is spawned, no physics is stepped.  (Distinct from\n\
                     \x20                       the `mars-rover-replay` binary, which is an\n\
                     \x20                       FFI-level regression harness.)\n\
                     \x20 -h, --help            Print this help message\n\
                     \n\
                     In-simulator controls:\n\
                     \x20 Arrow keys            Manual wheel override (disabled in --replay)\n\
                     \x20 R                     Reset rover position / restart replay\n\
                     \x20 T                     Toggle status text\n\
                     \x20 F                     Toggle slip-free mode (disabled in --replay)\n\
                     \x20 P                     Toggle waypoint path overlay\n\
                     \x20 C                     Toggle console\n\
                     \x20 L                     Toggle TRON ribbon trail\n\
                     \x20 Drag                  Move rover / obstacles (disabled in --replay)\n\
                     \x20 Scroll                Zoom camera\n"
                );
                std::process::exit(0);
            }
            "--seed" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    if let Ok(v) = val.parse::<u64>() {
                        seed = v;
                    }
                }
            }
            "--gps-interval" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    if let Ok(v) = val.parse::<u32>() {
                        gps_interval_ms = v;
                    }
                }
            }
            "--log" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    log_path = Some(std::path::PathBuf::from(val));
                }
            }
            "--waypoints" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    waypoint_path = Some(std::path::PathBuf::from(val));
                }
            }
            "--text" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    text = Some(val.clone());
                }
            }
            "--replay" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    replay_path = Some(std::path::PathBuf::from(val));
                }
            }
            _ => {}
        }
        i += 1;
    }

    Args { seed, gps_interval_ms, log_path, waypoint_path, text, replay_path }
}

fn load_waypoints(path: &std::path::Path) -> Vec<(f64, f64)> {
    let content = match std::fs::read_to_string(path) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("warning: could not read waypoints file {:?}: {}", path, e);
            return Vec::new();
        }
    };
    let mut wps = Vec::new();
    for line in content.lines() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let parts: Vec<&str> = if line.contains(',') {
            line.splitn(2, ',').collect()
        } else {
            line.splitn(2, ' ').collect()
        };
        if parts.len() == 2 {
            if let (Ok(x), Ok(y)) = (
                parts[0].trim().parse::<f64>(),
                parts[1].trim().parse::<f64>(),
            ) {
                wps.push((x, y));
            }
        }
    }
    wps
}

fn main() {
    let args = parse_args();

    if let Some(replay_path) = args.replay_path.clone() {
        run_replay(args, replay_path);
        return;
    }

    run_live(args);
}

fn run_live(args: Args) {
    controller::set_gps_interval_ms(args.gps_interval_ms);

    // --text takes priority over --waypoints when both are given.
    let waypoints: Vec<(f64, f64)> = if let Some(t) = args.text.as_deref() {
        glyph::text_to_waypoints(
            t,
            world::TERRAIN_WORLD_WIDTH,
            world::TERRAIN_WORLD_HEIGHT,
        )
    } else if let Some(p) = args.waypoint_path.as_deref() {
        load_waypoints(p)
    } else {
        vec![]
    };

    // Expose waypoints to the Ada control software via the FFI.
    controller::set_waypoints(waypoints.iter().map(|&(x, y)| [x as f32, y as f32]).collect());

    // Construct the simulation state before any plugin runs, so Simulator
    // and Controller no longer implicitly depend on Visualizer's Startup.
    let world = world::World::build(args.seed);

    let mut app = App::new();
    app.insert_resource(TerrainSeed(args.seed))
        .insert_resource(WaypointsRes(waypoints))
        .insert_resource(RobotRes::from(world.robot))
        .insert_resource(EnvironmentRes::from(world.environment))
        .insert_resource(TerrainRes::from(world.terrain))
        .insert_resource(replay_mode::ReplayActive(false))
        .add_plugins(DefaultPlugins)
        .add_plugins(controller::Controller)
        .add_plugins(visualizer::Visualizer)
        .add_plugins(ribbon::RibbonPlugin)
        .add_plugins(console::ConsolePlugin)
        .add_plugins(simulator::Simulator)
        .add_plugins(logger::DataLogger);
    if let Some(path) = args.log_path {
        app.insert_resource(logger::LogPath(path));
    }
    // Enforce per-frame sim ordering across plugin boundaries:
    // apply_sim_inputs → simulate → control.
    app.configure_sets(
        Update,
        (SimStep::ApplyInputs, SimStep::Simulate, SimStep::Control).chain(),
    );
    app.run();
}

fn run_replay(args: Args, replay_path: std::path::PathBuf) {
    // Open the trace first — no point building a window just to fail on a
    // missing file.
    let source = match replay_source::FileTraceSource::open(&replay_path) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("error: cannot open replay trace {:?}: {e}", replay_path);
            std::process::exit(2);
        }
    };

    // Prefer the `.meta` sidecar's seed so terrain matches the recording.
    // Fall back to --seed when the sidecar is missing (pre-Phase-4 traces,
    // hand-crafted test inputs).
    let (seed, fell_back) = match sidecar::read_meta(&replay_path) {
        Some(m) => (m.seed, false),
        None => (args.seed, true),
    };
    if fell_back {
        eprintln!(
            "[replay] no metadata sidecar next to {:?}; using --seed {} (terrain may not match recording)",
            replay_path, seed,
        );
    }

    // Waypoints come from the `.wpts` sidecar the logger writes.  CLI
    // --waypoints / --text are ignored in replay mode: the rover's trajectory
    // is driven entirely from the CSV, so the overlay must reflect what the
    // recording's Ada thread was following, not what this invocation was told.
    let waypoints = sidecar::read_waypoints(&replay_path);

    let world = world::World::build(seed);

    let mut app = App::new();
    app.insert_resource(TerrainSeed(seed))
        .insert_resource(WaypointsRes(waypoints))
        .insert_resource(RobotRes::from(world.robot))
        .insert_resource(EnvironmentRes::from(world.environment))
        .insert_resource(TerrainRes::from(world.terrain))
        // Visualizer still reads SimInputsRes from its drag-drop handlers, but
        // nothing drains it in replay mode.  The handlers themselves check
        // `ReplayActive` and short-circuit before pushing; the resource only
        // needs to exist so the system signature matches.
        .init_resource::<SimInputsRes>()
        .add_plugins(DefaultPlugins)
        // Visualizer, Ribbon, Console still render.  Console will stay blank
        // (nothing pushes to CONSOLE_LOG); that's fine.
        .add_plugins(visualizer::Visualizer)
        .add_plugins(ribbon::RibbonPlugin)
        .add_plugins(console::ConsolePlugin)
        .add_plugins(replay_mode::ReplayMode::new(Box::new(source)));
    // SimStep sets are referenced by the Visualizer's chain config, so keep
    // them configured even though no system is scheduled against Simulate /
    // Control in replay mode.
    app.configure_sets(
        Update,
        (SimStep::ApplyInputs, SimStep::Simulate, SimStep::Control).chain(),
    );
    app.run();
}
