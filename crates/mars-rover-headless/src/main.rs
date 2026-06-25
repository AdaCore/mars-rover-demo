//! Bevy-free headless driver for the Mars Rover Ada control software.
//!
//! Drives a fixed-step simulation loop without rendering.  Used for:
//! - CI-like regression scenarios (deterministic CSV recordings).
//! - Batch EKF tuning (identical sensor noise seed → byte-identical logs).
//! - Future network-streaming viewer experimentation.
//!
//! # Obstacles
//!
//! Headless keeps the perimeter walls but **drops all movable rocks**.
//! Without a renderer there's no way to drag a rock out of the rover's
//! path, and a rock that happens to lie on the route to the first waypoint
//! halts the run.  The terrain heightmap is still built from `--seed`.
//!
//! # CLI
//!
//! ```text
//! --seed <u64>                 Terrain seed [default: 42]
//! --gps-interval <u32>         GPS update interval in ms; 0 = disabled [default: 500]
//! --waypoints <path>           Waypoint file (x y per line)
//! --text <string>              Generate rover waypoints that trace the given text
//! --log <path>                 Write per-step CSV diagnostic log
//! --duration-sim-seconds <f64> Virtual-time duration to run [default: 30.0]
//! --wall-clock                 Run in wall-clock mode (skips virtual-time enable)
//! --lockstep                   Enforce strict Ada ↔ sim barrier per Ada cycle (determinism test)
//! --noise-seed <u64>           Seed for the gaussian() RNG; 0 = OS entropy [default: 0]
//! --slip-free                  Force ideal traction on every wheel
//! ```

mod logger;

use std::{
    sync::{Condvar, Mutex, OnceLock},
    time::Duration,
};

use mars_rover_ada_link as ada_link;
use mars_rover_ada_link::{EncoderState, ImuState};
use mars_rover_core::{domain::Environment, glyph, sidecar, sim, world};

const DT: Duration = Duration::from_millis(10);
const DT_US: u64 = 10_000;

/// Main advances the virtual clock by `DT_US` every iteration, and Ada runs
/// one control cycle per Ada-delay (~40 ms).  Granting a permit once every
/// four DT iterations therefore lets Ada perceive its natural rate while
/// keeping VIRTUAL_CLOCK owned solely by the main thread.
const TICKS_PER_ADA_CYCLE: u32 = 4;

// ── Lockstep step-gate ───────────────────────────────────────────────────────
// Only used when --lockstep is passed.  Main thread is sole owner of
// VIRTUAL_CLOCK in this mode; Ada's delay calls become pure 2-phase barriers
// (announce-done + wait-for-permit) with no clock manipulation.

struct GateState {
    permits: u32,
    done_seq: u64,
}

static STEP_GATE: OnceLock<(Mutex<GateState>, Condvar, Condvar)> = OnceLock::new();

fn gate_init() {
    let _ = STEP_GATE.set((
        Mutex::new(GateState { permits: 0, done_seq: 0 }),
        Condvar::new(),
        Condvar::new(),
    ));
}

fn gate() -> &'static (Mutex<GateState>, Condvar, Condvar) {
    STEP_GATE.get().expect("gate_init not called")
}

/// Install an ada-link delay-ms hook that implements the 2-phase barrier:
/// Phase 1 announces Ada has reached a delay (main thread wakes); Phase 2
/// waits for the main thread to grant a permit.  Does NOT advance
/// VIRTUAL_CLOCK — the main thread is sole clock owner in lockstep mode.
fn install_lockstep_hook() {
    ada_link::set_delay_ms_hook(|_ms| {
        let (mtx, cv_ada, cv_done) = gate();
        {
            let mut g = mtx.lock().unwrap();
            g.done_seq += 1;
            cv_done.notify_all();
        }
        let mut g = cv_ada
            .wait_while(mtx.lock().unwrap(), |s| s.permits == 0)
            .unwrap();
        g.permits -= 1;
    });
}

/// Wait until Ada has entered at least one more delay since `last_seen`.
/// Returns the new `done_seq` value.
fn wait_for_ada_delay(last_seen: u64) -> u64 {
    let (mtx, _, cv_done) = gate();
    let g = cv_done
        .wait_while(mtx.lock().unwrap(), |s| s.done_seq == last_seen)
        .unwrap();
    g.done_seq
}

/// Grant one permit — allow Ada to wake from its next delay.
fn grant_ada_permit() {
    let (mtx, cv_ada, _) = gate();
    let mut g = mtx.lock().unwrap();
    g.permits += 1;
    cv_ada.notify_one();
}

// ── CLI parsing (hand-rolled, parallel to simulator/src/main.rs) ─────────────

struct Args {
    seed: u64,
    gps_interval_ms: u32,
    log_path: Option<std::path::PathBuf>,
    waypoint_path: Option<std::path::PathBuf>,
    text: Option<String>,
    duration_s: f64,
    wall_clock: bool,
    lockstep: bool,
    noise_seed: u64,
    slip_free: bool,
}

fn parse_args() -> Args {
    const DEFAULT_SEED: u64 = 42;
    const DEFAULT_GPS_INTERVAL_MS: u32 = 500;
    const DEFAULT_DURATION_S: f64 = 30.0;

    let args: Vec<String> = std::env::args().collect();
    let prog = args.first().map(String::as_str).unwrap_or("mars-rover-headless");

    let mut out = Args {
        seed: DEFAULT_SEED,
        gps_interval_ms: DEFAULT_GPS_INTERVAL_MS,
        log_path: None,
        waypoint_path: None,
        text: None,
        duration_s: DEFAULT_DURATION_S,
        wall_clock: false,
        lockstep: false,
        noise_seed: 0,
        slip_free: false,
    };

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "-h" | "--help" => {
                println!(
                    "Usage: {prog} [OPTIONS]\n\
                     \n\
                     Options:\n\
                     \x20 --seed <u64>                 Terrain generation seed [default: {DEFAULT_SEED}]\n\
                     \x20 --gps-interval <u32>         GPS update interval in ms; 0 = disabled [default: {DEFAULT_GPS_INTERVAL_MS}]\n\
                     \x20 --log <path>                 Write per-step CSV diagnostic log to <path>\n\
                     \x20 --waypoints <path>           Waypoint file (x y per line); omit for no path\n\
                     \x20 --text <string>              Generate rover waypoints that trace the given text\n\
                     \x20 --duration-sim-seconds <f64> Virtual-time duration to run [default: {DEFAULT_DURATION_S}]\n\
                     \x20 --wall-clock                 Run in wall-clock mode (no virtual time)\n\
                     \x20 --lockstep                   Enforce strict Ada ↔ sim barrier per Ada cycle\n\
                     \x20 --noise-seed <u64>           Seed for gaussian() RNG; 0 = OS entropy [default: 0]\n\
                     \x20 --slip-free                  Force ideal traction on every wheel\n\
                     \x20 -h, --help                   Print this help message\n"
                );
                std::process::exit(0);
            }
            "--seed" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    if let Ok(v) = val.parse::<u64>() { out.seed = v; }
                }
            }
            "--gps-interval" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    if let Ok(v) = val.parse::<u32>() { out.gps_interval_ms = v; }
                }
            }
            "--log" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    out.log_path = Some(std::path::PathBuf::from(val));
                }
            }
            "--waypoints" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    out.waypoint_path = Some(std::path::PathBuf::from(val));
                }
            }
            "--text" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    out.text = Some(val.clone());
                }
            }
            "--duration-sim-seconds" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    if let Ok(v) = val.parse::<f64>() { out.duration_s = v; }
                }
            }
            "--wall-clock" => out.wall_clock = true,
            "--lockstep" => out.lockstep = true,
            "--noise-seed" => {
                i += 1;
                if let Some(val) = args.get(i) {
                    if let Ok(v) = val.parse::<u64>() { out.noise_seed = v; }
                }
            }
            "--slip-free" => out.slip_free = true,
            _ => {}
        }
        i += 1;
    }

    out
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

// ── Main ─────────────────────────────────────────────────────────────────────

fn main() {
    let args = parse_args();

    // Waypoint resolution: --text overrides --waypoints.
    let waypoints: Vec<(f64, f64)> = if let Some(t) = args.text.as_deref() {
        glyph::text_to_waypoints(t, world::TERRAIN_WORLD_WIDTH, world::TERRAIN_WORLD_HEIGHT)
    } else if let Some(p) = args.waypoint_path.as_deref() {
        load_waypoints(p)
    } else {
        Vec::new()
    };

    // Configure ada-link BEFORE spawning the Ada thread.
    ada_link::set_gps_interval_ms(args.gps_interval_ms);
    ada_link::set_noise_seed(args.noise_seed);
    ada_link::set_waypoints(
        waypoints.iter().map(|&(x, y)| [x as f32, y as f32]).collect(),
    );

    if !args.wall_clock {
        // Non-zero value flips `mars_rover_clock` + `now_ms` to virtual mode.
        ada_link::enable_virtual_time(1);
    }

    if args.lockstep {
        gate_init();
        install_lockstep_hook();
    }

    ada_link::spawn_ada_thread();

    // Build the sim world.  Headless keeps perimeter walls but drops
    // movable rocks — see the crate-level "Obstacles" note for rationale.
    let mut world = world::World::build(args.seed);
    world.environment = Environment::new(
        world
            .environment
            .obstacles()
            .iter()
            .filter(|o| !o.is_movable())
            .cloned()
            .collect(),
    );
    let mut enc = EncoderState::default();
    let mut imu = ImuState::default();
    let mut slip_free = args.slip_free;
    // Present but always empty in headless; wired for consistency with the GUI
    // control-plane so a future network viewer can push SimInputs here.
    let sim_inputs = sim::SimInputs::default();

    // Optional CSV logger.
    let mut log = match args.log_path.as_deref() {
        Some(p) => match logger::Logger::open(p) {
            Ok(l) => {
                let meta = sidecar::TraceMeta {
                    seed: args.seed,
                    gps_interval_ms: args.gps_interval_ms,
                    noise_seed: args.noise_seed,
                    slip_free: args.slip_free,
                };
                if let Err(e) = sidecar::write_sidecars(p, &meta, &waypoints) {
                    eprintln!("[logger] Cannot write sidecars for {:?}: {e}", p);
                }
                Some(l)
            }
            Err(e) => {
                eprintln!("[logger] Cannot open {:?}: {e}", p);
                None
            }
        },
        None => None,
    };

    let duration_us = (args.duration_s * 1_000_000.0) as u64;
    let mut virtual_elapsed_us: u64 = 0;
    let mut ada_done_seq: u64 = 0;
    let mut ticks_since_grant: u32 = 0;

    // In lockstep mode, wait for Ada to reach its first delay (init path:
    // Set_Power, initial sensor reads, etc.) before the main loop starts.
    if args.lockstep {
        ada_done_seq = wait_for_ada_delay(0);
    }

    // ── Main fixed-step loop ─────────────────────────────────────────────────
    loop {
        // Drain any SimInputs pushed by a future control plane.
        for input in sim_inputs.drain() {
            sim::apply(
                &input,
                &mut world.robot,
                &mut world.environment,
                &mut slip_free,
                |inp| match inp {
                    sim::SimInput::Reset { pose } | sim::SimInput::RequestEkfReset { pose } => {
                        ada_link::request_ekf_reset(pose.0, pose.1, pose.2);
                    }
                    sim::SimInput::LoadWaypoints(wps) => {
                        let arr: Vec<[f32; 2]> =
                            wps.iter().map(|&(x, y)| [x as f32, y as f32]).collect();
                        ada_link::set_waypoints(arr);
                    }
                    _ => {}
                },
            );
        }

        sim::advance(
            &mut world.robot,
            &world.environment,
            &world.terrain,
            DT,
            slip_free,
        );

        let now_ms = (virtual_elapsed_us / 1_000) as u32;
        ada_link::tick_controller(
            &mut world.robot,
            DT,
            now_ms,
            0, // keys_mask — autonomous-mode takeover from Ada
            &mut enc,
            &mut imu,
        );

        if let Some(l) = log.as_mut() {
            l.write_row(&world.robot, virtual_elapsed_us);
        }

        // Flush Ada-side console log to stderr for visibility.
        for msg in ada_link::drain_new_messages() {
            eprintln!("[ada] {msg}");
        }

        virtual_elapsed_us += DT_US;
        ada_link::advance_virtual_clock_by(DT_US);
        ticks_since_grant += 1;

        if args.lockstep {
            // Grant Ada a permit once every TICKS_PER_ADA_CYCLE DT steps so
            // Ada's virtual-time perception matches its expected 40 ms cycle.
            if ticks_since_grant >= TICKS_PER_ADA_CYCLE {
                grant_ada_permit();
                ada_done_seq = wait_for_ada_delay(ada_done_seq);
                ticks_since_grant = 0;
            }
        } else {
            // Non-lockstep: Ada's thread advances itself via the default
            // virtual-time fetch_add + yield_now path in ada-link.  Yield here
            // so Ada gets a chance to run between sim steps.
            std::thread::yield_now();
        }

        if virtual_elapsed_us >= duration_us {
            break;
        }
    }
}
