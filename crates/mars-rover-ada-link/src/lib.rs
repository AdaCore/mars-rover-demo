//! Static-library linkage and Rust-side FFI surface for the Ada control
//! software (`libMars_Rover.a`).
//!
//! This crate owns:
//!
//! * The `build.rs` that builds the Ada static library and emits the
//!   `-L`/`-l` rustc-link directives (via the `links = "Mars_Rover"` key).
//! * All `#[no_mangle]` extern "C" functions that Ada calls back into —
//!   clock, sonar, encoders, IMU, GPS, EKF bridge, waypoints, etc.
//! * The backing atomic statics and the thread-local RNG used by the
//!   sensor-noise model.
//! * Public Rust accessor functions that other crates (the Bevy GUI, the
//!   replay harness, the future headless driver) use to read/write those
//!   atomics.
//!
//! The atomics themselves are private; callers go through the getter/setter
//! functions at the bottom of this file.

use std::{
    cell::RefCell,
    collections::VecDeque,
    sync::{
        atomic::{AtomicBool, AtomicI8, AtomicI32, AtomicU16, AtomicU32, AtomicU64, Ordering},
        Mutex, OnceLock,
    },
    thread,
    time::{Duration, Instant},
};

use once_cell::sync::Lazy;
use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;

// ── Timekeeping ──────────────────────────────────────────────────────────────

static EPOCH: Lazy<Instant> = Lazy::new(Instant::now);

/// Virtual clock time in microseconds. Zero means wall-clock mode (normal GUI
/// operation).  When non-zero, `mars_rover_clock()` returns this value and
/// the delay functions advance it instead of sleeping.
static VIRTUAL_CLOCK: AtomicU64 = AtomicU64::new(0);

/// Wall-clock time (µs since EPOCH) recorded at the most recent
/// `Delay_Milliseconds` call, just before Ada actually sleeps.  Zero until
/// Ada reaches its first delay.  Written by Ada's background thread; read by
/// the Bevy logger.  Only updated in wall-clock mode so that replay does not
/// overwrite it.
static ADA_STEP_US: AtomicU64 = AtomicU64::new(0);

// ── Sensor atomics (Rust→Ada) ────────────────────────────────────────────────

static DISTANCE_SENSOR_DISTANCE: AtomicU32 = AtomicU32::new(0);
static DISTANCE_SENSOR_ANGLE: AtomicI8 = AtomicI8::new(0);

static WHEEL_ANGLE: [AtomicI8; 4] = [
    AtomicI8::new(0),
    AtomicI8::new(0),
    AtomicI8::new(0),
    AtomicI8::new(0),
];
static LEFT_WHEEL_POWER: AtomicI8 = AtomicI8::new(0);
static RIGHT_WHEEL_POWER: AtomicI8 = AtomicI8::new(0);
static CONTROLLER_BUTTONS_STATE: AtomicU16 = AtomicU16::new(0);
static CONSOLE_LOG: Lazy<Mutex<VecDeque<String>>> = Lazy::new(|| Mutex::new(VecDeque::new()));

// Encoder delta accumulators (Rust→Ada).
// Indexed by corner wheel: 0=FrontLeft, 1=FrontRight, 2=RearLeft, 3=RearRight.
// `mars_rover_encoder_ticks()` atomically swaps the value to 0 on read.
static ENCODER_TICKS: [AtomicI32; 4] = [
    AtomicI32::new(0), AtomicI32::new(0),
    AtomicI32::new(0), AtomicI32::new(0),
];

// Cumulative truth encoder ticks from physics (never reset), parallel to ENCODER_TICKS.
static ENCODER_TRUTH_TOTAL: [AtomicI32; 4] = [
    AtomicI32::new(0), AtomicI32::new(0),
    AtomicI32::new(0), AtomicI32::new(0),
];

// GPS fix (Rust→Ada). Coordinates scaled ×1_000_000 (µm precision in world metres).
// Timestamp is milliseconds since EPOCH; Ada detects freshness by comparing timestamps.
static GPS_X:         AtomicI32 = AtomicI32::new(0);
static GPS_Y:         AtomicI32 = AtomicI32::new(0);
static GPS_TIMESTAMP: AtomicU32 = AtomicU32::new(0);
/// Tracks when the most recent fix was emitted; compared against the emission
/// interval by the GUI `control()` shim to decide whether to emit a new fix.
static GPS_LAST_FIX:  AtomicU32 = AtomicU32::new(0);

/// GPS emission interval in milliseconds.  0 = disabled.  Set at startup from CLI.
static GPS_INTERVAL_MS: AtomicU32 = AtomicU32::new(500);

// IMU gyroscope angular rate, Rust→Ada.
// Updated every control() frame: (true_yaw_rate + Gaussian noise) × 8192 LSB/(rad/s).
// Scale matches a MEMS ±250°/s FSR: ±4.0 rad/s range, ~0.007°/s resolution.
static IMU_GYRO_Z: AtomicI32 = AtomicI32::new(0);

// ── Ada→Rust GNC state (pushed by Ada via mars_rover_report_gnc_state) ───────
// GNC_VALID is false until Ada has called Poll at least once.
static GNC_VALID: AtomicBool = AtomicBool::new(false);
static GNC_EST_X: AtomicI32  = AtomicI32::new(0); // Ada's position estimate X (×1_000_000)
static GNC_EST_Y: AtomicI32  = AtomicI32::new(0); // Ada's position estimate Y (×1_000_000)
static GNC_ENC: [AtomicI32; 4] = [
    AtomicI32::new(0), AtomicI32::new(0),
    AtomicI32::new(0), AtomicI32::new(0),
];

// ── EKF estimated state (pushed by Ada via mars_rover_set_estimated_position) ─
// EKF_VALID is false until the filter has initialised and run at least once.
static EKF_VALID:     AtomicBool = AtomicBool::new(false);
static EKF_EST_X:     AtomicI32  = AtomicI32::new(0); // estimated X (×1_000_000 m)
static EKF_EST_Y:     AtomicI32  = AtomicI32::new(0); // estimated Y (×1_000_000 m)
static EKF_EST_THETA: AtomicI32  = AtomicI32::new(0); // estimated θ (×1_000_000 rad)

// ── EKF reset request (Rust→Ada) ─────────────────────────────────────────────
static EKF_RESET_REQUESTED: AtomicBool = AtomicBool::new(false);
static EKF_RESET_X:         AtomicI32  = AtomicI32::new(0);
static EKF_RESET_Y:         AtomicI32  = AtomicI32::new(0);
static EKF_RESET_THETA:     AtomicI32  = AtomicI32::new(0);

// Waypoints loaded from --waypoints file (Rust→Ada, constant after startup).
static WAYPOINTS: OnceLock<Vec<[f32; 2]>> = OnceLock::new();

/// Seed for the gaussian RNG.  `0` (the default) means "seed from OS entropy on
/// first use", preserving today's stochastic GUI behaviour.  Any non-zero value
/// means the per-thread `ChaCha8Rng` is seeded deterministically via
/// `seed_from_u64`, which is what the headless determinism test requires.
///
/// The CLI is expected to call `set_noise_seed` exactly once at startup,
/// before any call to `gaussian()`.
static NOISE_SEED: AtomicU64 = AtomicU64::new(0);

thread_local! {
    static NOISE_RNG: RefCell<Option<ChaCha8Rng>> = const { RefCell::new(None) };
}

/// Per-system local state for the encoder: previous `rotating_angle` per
/// corner wheel (radians) and a sub-tooth carry so fractional rotations
/// accumulate rather than being lost to truncation.
///
/// Made `pub` so `tick_controller` (a future headless entry point) can take
/// it by `&mut` — the Bevy GUI stores it in `Local<_>`.
#[derive(Default)]
pub struct EncoderState {
    pub prev_angle: [f64; 4],
    pub carry: [f64; 4],
}

/// Per-system local state for the IMU.  Kept separate from `EncoderState`
/// so additional IMU channels (accelerometers, temperature) can be added
/// here without touching encoder logic.  `pub` for the same reason.
#[derive(Default)]
pub struct ImuState {
    pub prev_heading: f64, // radians, in [0, 2π); updated each frame
}

/// Hook that, when installed, replaces the default body of
/// `mars_rover_delay_milliseconds`.  Intended for the replay harness's
/// step-gate synchronisation — production callers (GUI, headless) should
/// leave this unset and rely on the default (virtual-time `fetch_add` or
/// wall-clock `sleep`).
///
/// Set at most once per process; subsequent `set_delay_ms_hook` calls are
/// silently ignored.
static DELAY_MS_HOOK: OnceLock<Box<dyn Fn(u16) + Send + Sync>> = OnceLock::new();

/// Install a hook that replaces the default body of
/// `mars_rover_delay_milliseconds`.  See `DELAY_MS_HOOK`.  Returns without
/// error if a hook was already installed.
pub fn set_delay_ms_hook<F: Fn(u16) + Send + Sync + 'static>(f: F) {
    let _ = DELAY_MS_HOOK.set(Box::new(f));
}

// ── Ada library imports ──────────────────────────────────────────────────────

extern "C" {
    fn Mars_Roverinit();
    fn Mars_Roverfinal();
    fn mars_rover_demo_task();
}

/// Spawn a background thread running the Ada control task.  The thread owns
/// the Ada runtime lifecycle: init, run, final.  Call exactly once at
/// startup.
pub fn spawn_ada_thread() {
    let _ = std::thread::spawn(move || unsafe {
        Mars_Roverinit();
        mars_rover_demo_task();
        Mars_Roverfinal();
    });
}

// ── FFI exports (Ada → Rust) ─────────────────────────────────────────────────

#[no_mangle]
pub extern "C" fn mars_rover_clock() -> u64 {
    let vt = VIRTUAL_CLOCK.load(Ordering::Relaxed);
    if vt != 0 {
        vt
    } else {
        (Instant::now() - *EPOCH).as_micros() as u64
    }
}

#[no_mangle]
pub extern "C" fn mars_rover_delay_microseconds(us: u16) {
    if VIRTUAL_CLOCK.load(Ordering::Relaxed) != 0 {
        VIRTUAL_CLOCK.fetch_add(us as u64, Ordering::Relaxed);
    } else {
        thread::sleep(Duration::from_micros(us as u64));
    }
}

#[no_mangle]
pub extern "C" fn mars_rover_delay_milliseconds(ms: u16) {
    // A caller-installed hook (e.g. replay's step-gate) overrides the
    // default body entirely.
    if let Some(h) = DELAY_MS_HOOK.get() {
        h(ms);
        return;
    }
    if VIRTUAL_CLOCK.load(Ordering::Relaxed) != 0 {
        VIRTUAL_CLOCK.fetch_add(ms as u64 * 1_000, Ordering::Relaxed);
        // Yield so the main thread gets a chance to run; without this, Ada's
        // thread can busy-spin past the main thread under virtual time.
        std::thread::yield_now();
    } else {
        // Record Ada's wall-clock time at this step boundary so the logger
        // can capture it for deterministic replay.  Do this before sleeping
        // so the next Bevy frame sees the updated value.
        ADA_STEP_US.store(
            (Instant::now() - *EPOCH).as_micros() as u64,
            Ordering::Relaxed,
        );
        thread::sleep(Duration::from_millis(ms as u64));
    }
}

/// Set the virtual clock to `micros` microseconds, enabling virtual-time mode.
/// Pass 0 to revert to wall-clock mode (normal simulator operation).
#[no_mangle]
pub extern "C" fn mars_rover_set_clock(micros: u64) {
    VIRTUAL_CLOCK.store(micros, Ordering::Relaxed);
}

#[no_mangle]
pub extern "C" fn mars_rover_sonar_distance() -> u32 {
    DISTANCE_SENSOR_DISTANCE.load(Ordering::Relaxed)
}

#[no_mangle]
pub extern "C" fn mars_rover_set_mast_angle(a: i8) {
    DISTANCE_SENSOR_ANGLE.store(a, Ordering::Relaxed)
}

#[no_mangle]
pub extern "C" fn mars_rover_set_wheel_angle(wheel: u8, side: u8, a: i8) {
    // Mapping: (wheel, side) → corner index [FL, FR, RL, RR].
    // This matches WheelID's corner indices (0..=3) used throughout the
    // Rust side; we hard-code here to avoid pulling in mars-rover-core.
    let idx = match (wheel, side) {
        (0, 0) => 0, // FrontLeft
        (0, 1) => 1, // FrontRight
        (1, 0) => 2, // RearLeft
        (1, 1) => 3, // RearRight
        _ => return,
    };
    WHEEL_ANGLE[idx].store(a, Ordering::Relaxed);
}

#[no_mangle]
pub extern "C" fn mars_rover_set_power(side: u8, power: i8) {
    if side == 0 {
        LEFT_WHEEL_POWER.store(power, Ordering::Relaxed);
    } else if side == 1 {
        RIGHT_WHEEL_POWER.store(power, Ordering::Relaxed);
    }
}

#[no_mangle]
pub extern "C" fn mars_rover_controller_state() -> u16 {
    CONTROLLER_BUTTONS_STATE.load(Ordering::Relaxed)
}

#[no_mangle]
pub extern "C" fn mars_rover_set_display_info(ptr: *const u8, len: i32) {
    // Safety: Ada passes Str'Address + Str'Length of a valid Ada string (ASCII subset of UTF-8).
    let s = unsafe {
        std::str::from_utf8_unchecked(std::slice::from_raw_parts(ptr, len as usize)).to_owned()
    };
    if let Ok(mut log) = CONSOLE_LOG.lock() {
        log.push_back(s);
        while log.len() > 50 {
            log.pop_front();
        }
    }
}

/// Returns the encoder tick delta accumulated since the last call for the given corner wheel
/// (0=FrontLeft, 1=FrontRight, 2=RearLeft, 3=RearRight) and resets the counter to zero.
/// Clamped to i16 range; in practice the per-cycle delta is never more than a few ticks.
#[no_mangle]
pub extern "C" fn mars_rover_encoder_ticks(wheel: u8) -> i16 {
    let idx = wheel as usize;
    if idx < 4 {
        ENCODER_TICKS[idx]
            .swap(0, Ordering::Relaxed)
            .clamp(i16::MIN as i32, i16::MAX as i32) as i16
    } else {
        0
    }
}

#[no_mangle]
pub extern "C" fn mars_rover_gps_x() -> i32 {
    GPS_X.load(Ordering::Relaxed)
}

#[no_mangle]
pub extern "C" fn mars_rover_gps_y() -> i32 {
    GPS_Y.load(Ordering::Relaxed)
}

/// Milliseconds since simulator start at the time of the last GPS fix.
/// Ada compares this against the previously seen value to detect a fresh fix.
#[no_mangle]
pub extern "C" fn mars_rover_gps_timestamp() -> u32 {
    GPS_TIMESTAMP.load(Ordering::Relaxed)
}

/// IMU gyroscope Z-axis angular rate, scaled ×8192 LSB/(rad/s).
/// Returns the most recent instantaneous yaw rate (plus Gaussian noise).
/// No reset on read — Ada always gets the latest value.
/// Clamped to i16 range; in practice the rover's ±4.0 rad/s max fits in ±32767.
#[no_mangle]
pub extern "C" fn mars_rover_imu_gyro_z() -> i16 {
    IMU_GYRO_Z
        .load(Ordering::Relaxed)
        .clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

/// Number of waypoints loaded at startup.  Returns 0 if no file was provided.
#[no_mangle]
pub extern "C" fn mars_rover_waypoint_count() -> u32 {
    WAYPOINTS.get().map(|v| v.len() as u32).unwrap_or(0)
}

/// X coordinate of waypoint `idx`, scaled ×1_000_000.  Returns 0 for out-of-bounds index.
#[no_mangle]
pub extern "C" fn mars_rover_waypoint_x(idx: u32) -> i32 {
    WAYPOINTS
        .get()
        .and_then(|v| v.get(idx as usize))
        .map(|wp| (wp[0] * 1_000_000.0) as i32)
        .unwrap_or(0)
}

/// Y coordinate of waypoint `idx`, scaled ×1_000_000.  Returns 0 for out-of-bounds index.
#[no_mangle]
pub extern "C" fn mars_rover_waypoint_y(idx: u32) -> i32 {
    WAYPOINTS
        .get()
        .and_then(|v| v.get(idx as usize))
        .map(|wp| (wp[1] * 1_000_000.0) as i32)
        .unwrap_or(0)
}

/// Called by Ada (`Rover_HAL.Set_Estimated_Position`) to push the EKF
/// estimated position and heading.  All values are scaled ×1_000_000.
#[no_mangle]
pub extern "C" fn mars_rover_set_estimated_position(x: i32, y: i32, theta: i32) {
    EKF_EST_X.store(x, Ordering::Relaxed);
    EKF_EST_Y.store(y, Ordering::Relaxed);
    EKF_EST_THETA.store(theta, Ordering::Relaxed);
    EKF_VALID.store(true, Ordering::Relaxed);
}

/// Called by Ada (`Rover_HAL.EKF_Reset_Pending`) each Poll cycle.
/// Returns 1 once if a reset was requested, then 0 (consume-on-read).
#[no_mangle]
pub extern "C" fn mars_rover_ekf_reset_pending() -> i32 {
    // Acquire pairs with the Release store in `request_ekf_reset`,
    // guaranteeing the X/Y/Theta values written before the flag are visible
    // to the caller before it reads the getter functions below.
    if EKF_RESET_REQUESTED.swap(false, Ordering::Acquire) { 1 } else { 0 }
}

/// Reset pose: X position in metres (×1_000_000).
#[no_mangle]
pub extern "C" fn mars_rover_ekf_reset_x() -> i32 {
    EKF_RESET_X.load(Ordering::Relaxed)
}

/// Reset pose: Y position in metres (×1_000_000).
#[no_mangle]
pub extern "C" fn mars_rover_ekf_reset_y() -> i32 {
    EKF_RESET_Y.load(Ordering::Relaxed)
}

/// Reset pose: heading in radians (×1_000_000).
#[no_mangle]
pub extern "C" fn mars_rover_ekf_reset_theta() -> i32 {
    EKF_RESET_THETA.load(Ordering::Relaxed)
}

/// Called by Ada (`Rover.GNC.Poll`) to push its current state estimate.
/// `est_x/y` are the sensed GPS position scaled ×1_000_000.
/// `enc_*` are cumulative encoder ticks since start.
#[no_mangle]
pub extern "C" fn mars_rover_report_gnc_state(
    est_x: i32, est_y: i32,
    enc_fl: i32, enc_fr: i32, enc_rl: i32, enc_rr: i32,
) {
    GNC_EST_X.store(est_x, Ordering::Relaxed);
    GNC_EST_Y.store(est_y, Ordering::Relaxed);
    GNC_ENC[0].store(enc_fl, Ordering::Relaxed);
    GNC_ENC[1].store(enc_fr, Ordering::Relaxed);
    GNC_ENC[2].store(enc_rl, Ordering::Relaxed);
    GNC_ENC[3].store(enc_rr, Ordering::Relaxed);
    GNC_VALID.store(true, Ordering::Relaxed);
}

// ── Rust-side public API ─────────────────────────────────────────────────────

/// Set the seed used to initialise the per-thread gaussian RNG on its first
/// use.  Call exactly once at startup (before the first `gaussian()` call on
/// any thread).  Passing `0` reverts to OS-entropy seeding.
pub fn set_noise_seed(seed: u64) {
    NOISE_SEED.store(seed, Ordering::Relaxed);
}

/// Box-Muller transform: returns a Gaussian-distributed sample with the given σ.
///
/// Uses a per-thread `ChaCha8Rng`, lazily initialised on first use.  If
/// `NOISE_SEED` is non-zero at first use, the RNG is seeded deterministically
/// via `ChaCha8Rng::seed_from_u64`; otherwise it is seeded from OS entropy.
pub fn gaussian(sigma: f64) -> f64 {
    use std::f64::consts::PI;
    NOISE_RNG.with(|cell| {
        let mut slot = cell.borrow_mut();
        let rng = slot.get_or_insert_with(|| {
            let seed = NOISE_SEED.load(Ordering::Relaxed);
            if seed == 0 {
                ChaCha8Rng::from_os_rng()
            } else {
                ChaCha8Rng::seed_from_u64(seed)
            }
        });
        let u1: f64 = rng.random::<f64>().max(f64::EPSILON); // avoid ln(0)
        let u2: f64 = rng.random::<f64>();
        (-2.0_f64 * u1.ln()).sqrt() * (2.0 * PI * u2).cos() * sigma
    })
}

/// Set the GPS update interval (ms).  Call before the Bevy app starts.
/// Pass 0 to disable GPS entirely.
pub fn set_gps_interval_ms(ms: u32) {
    GPS_INTERVAL_MS.store(ms, Ordering::Relaxed);
}

/// Read the GPS emission interval currently in force.
pub fn gps_interval_ms() -> u32 {
    GPS_INTERVAL_MS.load(Ordering::Relaxed)
}

/// Load waypoints.  Call exactly once before `App::run()`.
pub fn set_waypoints(wps: Vec<[f32; 2]>) {
    let _ = WAYPOINTS.set(wps);
}

/// Returns the waypoints set at startup (empty slice if none were loaded).
pub fn waypoints() -> &'static [[f32; 2]] {
    WAYPOINTS.get().map(|v| v.as_slice()).unwrap_or(&[])
}

/// Write the EKF estimated-state atomics directly, bypassing Ada.  Used by
/// the trace-driven GUI renderer to push a logged estimate into the same
/// atomics the live controller updates, so the existing `draw_ekf_marker`
/// system works unchanged during replay.
///
/// Does NOT set `EKF_RESET_REQUESTED`; the normal live path (Ada writing via
/// `mars_rover_set_estimated_position`) is undisturbed.
pub fn set_ekf_estimate(x: f64, y: f64, theta: f64) {
    const SCALE: f64 = 1_000_000.0;
    EKF_EST_X.store((x * SCALE) as i32, Ordering::Relaxed);
    EKF_EST_Y.store((y * SCALE) as i32, Ordering::Relaxed);
    EKF_EST_THETA.store((theta * SCALE) as i32, Ordering::Relaxed);
    EKF_VALID.store(true, Ordering::Relaxed);
}

/// Clear the EKF_VALID flag so the renderer hides the marker (e.g. when a
/// replay is rewound past the point where the logged filter had converged).
pub fn clear_ekf_estimate() {
    EKF_VALID.store(false, Ordering::Relaxed);
}

/// Signal Ada to re-initialise the EKF on its next Poll cycle.
/// Position and heading are stored so Ada can skip the GPS-wait path and
/// initialise immediately at the correct pose.
/// Also clears EKF_VALID so the yellow marker disappears immediately.
pub fn request_ekf_reset(x: f64, y: f64, theta: f64) {
    const SCALE: f64 = 1_000_000.0;
    EKF_RESET_X.store((x * SCALE) as i32, Ordering::Relaxed);
    EKF_RESET_Y.store((y * SCALE) as i32, Ordering::Relaxed);
    EKF_RESET_THETA.store((theta * SCALE) as i32, Ordering::Relaxed);
    // Release ensures the X/Y/Theta stores above are visible to any thread
    // that observes this flag via an Acquire load.
    EKF_RESET_REQUESTED.store(true, Ordering::Release);
    EKF_VALID.store(false, Ordering::Relaxed);
}

/// Drain all pending console messages written by Ada via
/// `mars_rover_set_display_info`.
pub fn drain_new_messages() -> Vec<String> {
    CONSOLE_LOG
        .lock()
        .map(|mut q| q.drain(..).collect())
        .unwrap_or_default()
}

/// Ada's current state estimate pushed via `mars_rover_report_gnc_state`.
pub struct GncState {
    pub est_x: f32,
    pub est_y: f32,
    /// Cumulative encoder ticks per corner wheel: [FL, FR, RL, RR].
    pub enc: [i32; 4],
}

/// Returns Ada's latest state estimate, or `None` until Ada has called Poll
/// at least once.
pub fn gnc_state() -> Option<GncState> {
    if !GNC_VALID.load(Ordering::Relaxed) {
        return None;
    }
    const SCALE: f32 = 1_000_000.0;
    Some(GncState {
        est_x: GNC_EST_X.load(Ordering::Relaxed) as f32 / SCALE,
        est_y: GNC_EST_Y.load(Ordering::Relaxed) as f32 / SCALE,
        enc: [
            GNC_ENC[0].load(Ordering::Relaxed),
            GNC_ENC[1].load(Ordering::Relaxed),
            GNC_ENC[2].load(Ordering::Relaxed),
            GNC_ENC[3].load(Ordering::Relaxed),
        ],
    })
}

/// EKF estimated state (f32 precision).  Returns `None` until the filter has
/// produced its first estimate.
pub fn ekf_estimated_state() -> Option<(f32, f32, f32)> {
    if !EKF_VALID.load(Ordering::Relaxed) {
        return None;
    }
    const SCALE: f32 = 1_000_000.0;
    Some((
        EKF_EST_X.load(Ordering::Relaxed) as f32 / SCALE,
        EKF_EST_Y.load(Ordering::Relaxed) as f32 / SCALE,
        EKF_EST_THETA.load(Ordering::Relaxed) as f32 / SCALE,
    ))
}

/// EKF estimated state at f64 precision — used by replay for tighter
/// comparisons with the logged values, which are also f64.
pub fn ekf_estimated_state_f64() -> Option<(f64, f64, f64)> {
    if !EKF_VALID.load(Ordering::Relaxed) {
        return None;
    }
    const SCALE: f64 = 1_000_000.0;
    Some((
        EKF_EST_X.load(Ordering::Relaxed) as f64 / SCALE,
        EKF_EST_Y.load(Ordering::Relaxed) as f64 / SCALE,
        EKF_EST_THETA.load(Ordering::Relaxed) as f64 / SCALE,
    ))
}

/// Cumulative truth encoder ticks from physics per corner wheel: [FL, FR, RL, RR].
pub fn encoder_truth_total() -> [i32; 4] {
    [
        ENCODER_TRUTH_TOTAL[0].load(Ordering::Relaxed),
        ENCODER_TRUTH_TOTAL[1].load(Ordering::Relaxed),
        ENCODER_TRUTH_TOTAL[2].load(Ordering::Relaxed),
        ENCODER_TRUTH_TOTAL[3].load(Ordering::Relaxed),
    ]
}

/// Current motor power commands: (left, right), range -127..127.
pub fn motor_powers() -> (i8, i8) {
    (
        LEFT_WHEEL_POWER.load(Ordering::Relaxed),
        RIGHT_WHEEL_POWER.load(Ordering::Relaxed),
    )
}

/// Current GPS fix as stored in atomics: (x ×1_000_000, y ×1_000_000, timestamp_ms).
pub fn gps_current() -> (i32, i32, u32) {
    (
        GPS_X.load(Ordering::Relaxed),
        GPS_Y.load(Ordering::Relaxed),
        GPS_TIMESTAMP.load(Ordering::Relaxed),
    )
}

/// Timestamp (ms since EPOCH) of the most recent GPS fix emission.
pub fn gps_last_fix_ms() -> u32 {
    GPS_LAST_FIX.load(Ordering::Relaxed)
}

/// Record that a GPS fix was emitted at `ts_ms`.  The caller is expected to
/// have already written the fix via `set_gps_fix`.
pub fn set_gps_last_fix_ms(ts_ms: u32) {
    GPS_LAST_FIX.store(ts_ms, Ordering::Relaxed);
}

/// Sonar distance last written to Ada (centimetres); `u32::MAX` when no obstacle in range.
pub fn sonar_distance() -> u32 {
    DISTANCE_SENSOR_DISTANCE.load(Ordering::Relaxed)
}

/// IMU gyro-Z reading last written to Ada (8192 LSB per rad/s), clamped to i16 range.
pub fn imu_gyro_z() -> i16 {
    IMU_GYRO_Z
        .load(Ordering::Relaxed)
        .clamp(i16::MIN as i32, i16::MAX as i32) as i16
}

/// Mast angle last commanded by Ada (degrees).
pub fn mast_angle() -> i8 {
    DISTANCE_SENSOR_ANGLE.load(Ordering::Relaxed)
}

/// Steering angle last commanded by Ada for every corner wheel (degrees).
/// Indexed by corner: [FL, FR, RL, RR].
pub fn wheel_angles() -> [i8; 4] {
    [
        WHEEL_ANGLE[0].load(Ordering::Relaxed),
        WHEEL_ANGLE[1].load(Ordering::Relaxed),
        WHEEL_ANGLE[2].load(Ordering::Relaxed),
        WHEEL_ANGLE[3].load(Ordering::Relaxed),
    ]
}

/// Ada's wall-clock time (µs since EPOCH) at its most recent
/// `Delay_Milliseconds` call.  Zero until Ada has completed its first control
/// cycle.  Log this value as `ada_step_us` to enable exact-DT replay.
pub fn ada_step_us() -> u64 {
    ADA_STEP_US.load(Ordering::Relaxed)
}

/// Milliseconds since ada-link's EPOCH (the `Instant` captured when the
/// shared clock was first accessed).  GUI callers pass this into `set_gps_fix`
/// and check it against `gps_last_fix_ms()` to schedule emissions, so that
/// GPS timestamps share the same reference frame as `mars_rover_clock`.
///
/// Mirrors `mars_rover_clock`'s dual-mode logic: when `VIRTUAL_CLOCK` is
/// non-zero (headless / virtual-time mode), returns the virtual clock
/// converted to milliseconds; otherwise returns wall-clock milliseconds since
/// `EPOCH`.  This keeps GPS timestamps consistent with Ada's view of time
/// across both GUI and headless modes.
pub fn now_ms() -> u32 {
    let vt = VIRTUAL_CLOCK.load(Ordering::Relaxed);
    if vt != 0 {
        (vt / 1_000) as u32
    } else {
        (Instant::now() - *EPOCH).as_millis() as u32
    }
}

// ── Rust-side setters (sensor push) ──────────────────────────────────────────

/// Push the sonar distance (cm) Ada should see on its next read.
/// Use `u32::MAX` to signal "no obstacle in range".
pub fn set_sonar_distance(cm: u32) {
    DISTANCE_SENSOR_DISTANCE.store(cm, Ordering::Relaxed);
}

/// Push the raw IMU gyro-Z reading (pre-clamp, scaled ×8192 LSB/(rad/s)).
pub fn set_imu_gyro_z_raw(lsb: i32) {
    IMU_GYRO_Z.store(lsb, Ordering::Relaxed);
}

/// Overwrite the encoder-tick delta for the given corner wheel.  The next
/// `mars_rover_encoder_ticks(wheel)` call will return this value (and reset
/// the counter to zero).  Replay uses this because it owns the encoder state
/// between steps; live simulation uses `accumulate_encoder_tick` instead.
pub fn set_encoder_tick_delta(wheel: usize, delta: i32) {
    if wheel < 4 {
        ENCODER_TICKS[wheel].store(delta, Ordering::Relaxed);
    }
}

/// Add `delta` ticks to both the per-cycle encoder counter (consumed by Ada)
/// and the cumulative truth total (read by the logger).  Called from the live
/// simulator every frame.
pub fn accumulate_encoder_tick(wheel: usize, delta: i32) {
    if wheel < 4 {
        ENCODER_TICKS[wheel].fetch_add(delta, Ordering::Relaxed);
        ENCODER_TRUTH_TOTAL[wheel].fetch_add(delta, Ordering::Relaxed);
    }
}

/// Push a GPS fix.  Coordinates are pre-scaled by 1_000_000 (µm precision);
/// `ts_ms` is milliseconds since EPOCH.
pub fn set_gps_fix(x_scaled: i32, y_scaled: i32, ts_ms: u32) {
    GPS_X.store(x_scaled, Ordering::Relaxed);
    GPS_Y.store(y_scaled, Ordering::Relaxed);
    GPS_TIMESTAMP.store(ts_ms, Ordering::Relaxed);
}

/// Push the raw controller-button mask that Ada reads via
/// `mars_rover_controller_state`.
pub fn set_controller_buttons(mask: u16) {
    CONTROLLER_BUTTONS_STATE.store(mask, Ordering::Relaxed);
}

// ── Virtual-time clock controls ──────────────────────────────────────────────

/// Enable virtual-time mode and initialise the clock to `initial_us`.  Pass
/// any non-zero value; subsequent calls to `set_virtual_clock` /
/// `advance_virtual_clock_by` then control it.
pub fn enable_virtual_time(initial_us: u64) {
    VIRTUAL_CLOCK.store(initial_us, Ordering::Relaxed);
}

/// Overwrite the virtual clock with `micros`.
pub fn set_virtual_clock(micros: u64) {
    VIRTUAL_CLOCK.store(micros, Ordering::Relaxed);
}

/// Advance the virtual clock by `dt_us` microseconds.
pub fn advance_virtual_clock_by(dt_us: u64) {
    VIRTUAL_CLOCK.fetch_add(dt_us, Ordering::Relaxed);
}

// ── Per-frame controller bridge ──────────────────────────────────────────────

/// Bevy-free controller step.  Runs the per-frame bridge between the live
/// robot state and the FFI atomics that Ada reads/writes.
///
/// `now_ms` is the caller's clock (GUI: `now_ms()`; headless: the virtual
/// clock converted to ms).  `keys_mask` is the 4-bit keyboard state
/// (bit 0 = up, bit 1 = down, bit 2 = left, bit 3 = right); pass 0 in
/// headless.  `enc` and `imu` are per-caller state owned by the caller
/// (`Local<_>` in Bevy; stack-owned in headless).
pub fn tick_controller(
    robot: &mut mars_rover_core::domain::Robot,
    dt: std::time::Duration,
    now_ms: u32,
    keys_mask: u16,
    enc: &mut EncoderState,
    imu: &mut ImuState,
) {
    use mars_rover_core::domain::{Angle, Velocity, WheelID};

    const POWER_TO_VELOCITY_FACTOR: f64 = 1.0 / 350.0;

    // FFI corner index convention: 0=FrontLeft, 1=FrontRight, 2=RearLeft, 3=RearRight.
    // This is NOT the same as `WheelID::try_from(i)` (whose enum discriminants run
    // FL, RL, RR, FR); iterate explicitly so the mapping is invariant to future
    // reorderings of `WheelID`'s variants.  Matches `wheel_angles`,
    // `mars_rover_encoder_ticks`, and `mars_rover_set_wheel_angle`.
    const CORNER_WHEELS: [WheelID; 4] = [
        WheelID::FrontLeft, WheelID::FrontRight,
        WheelID::RearLeft,  WheelID::RearRight,
    ];

    let steer = wheel_angles();
    for (i, &wheel_id) in CORNER_WHEELS.iter().enumerate() {
        let _ = robot.set_wheel_steering_angle(wheel_id, Angle::from_deg(steer[i].into()));
    }

    let (left_power, right_power) = motor_powers();

    let left_wheel_velocity = Velocity::new(left_power as f64 * POWER_TO_VELOCITY_FACTOR);
    let _ = robot.set_wheel_velocity(WheelID::FrontLeft, left_wheel_velocity);
    let _ = robot.set_wheel_velocity(WheelID::CenterLeft, left_wheel_velocity);
    let _ = robot.set_wheel_velocity(WheelID::RearLeft, left_wheel_velocity);

    let right_wheel_velocity = Velocity::new(right_power as f64 * POWER_TO_VELOCITY_FACTOR);
    let _ = robot.set_wheel_velocity(WheelID::FrontRight, right_wheel_velocity);
    let _ = robot.set_wheel_velocity(WheelID::CenterRight, right_wheel_velocity);
    let _ = robot.set_wheel_velocity(WheelID::RearRight, right_wheel_velocity);

    set_sonar_distance(if let Some(distance) = robot.distance_sensor_distance() {
        (distance * 100.0) as u32
    } else {
        u32::MAX
    });

    robot.set_distance_sensor_angle(-Angle::from_deg(mast_angle().into()));

    set_controller_buttons(keys_mask);

    // --- Encoders ---
    // Compute the angular delta (in teeth) for each corner wheel since the last frame
    // and accumulate through ada-link.  Raw `rotating_angle` (radians) is used — not
    // the slip-scaled position — which is the correct encoder model (wheel rotation,
    // not ground displacement).  `CORNER_WHEELS` is shared with the steering loop above.
    const TEETH_PER_REV: f64 = 48.0;
    use std::f64::consts::PI;

    for (i, &wheel_id) in CORNER_WHEELS.iter().enumerate() {
        if let Ok(angle) = robot.wheel_rotating_angle(wheel_id) {
            let current = f64::from(angle);
            let delta_rad = current - enc.prev_angle[i];
            let accumulated = (delta_rad / (2.0 * PI)) * TEETH_PER_REV + enc.carry[i];
            let delta_ticks = accumulated.trunc() as i32;
            enc.carry[i] = accumulated - delta_ticks as f64;
            accumulate_encoder_tick(i, delta_ticks);
            enc.prev_angle[i] = current;
        }
    }

    // --- IMU gyroscope ---
    // Compute the true yaw rate from the change in robot.heading since the last frame,
    // add Gaussian noise, and store scaled to IMU_GYRO_Z.
    // Scale: 8192 LSB/(rad/s) — matching a MEMS ±250°/s FSR (±4.0 rad/s, ~0.007°/s resolution).
    const GYRO_SCALE: f64 = 8192.0; // LSB / (rad/s)
    const GYRO_SIGMA: f64 = 0.05;   // rad/s (1σ), ~2.9°/s — realistic MEMS noise floor

    let dt_secs = dt.as_secs_f64();
    if dt_secs > 1.0e-6 {
        let current_heading = f64::from(robot.heading());
        let raw_delta = current_heading - imu.prev_heading;
        // Wrap to (−π, π] to handle the 0/2π boundary cleanly.
        let heading_delta = (raw_delta + PI).rem_euclid(2.0 * PI) - PI;
        let omega_true = heading_delta / dt_secs;
        let omega_noisy = omega_true + gaussian(GYRO_SIGMA);
        set_imu_gyro_z_raw(
            (omega_noisy * GYRO_SCALE).clamp(i32::MIN as f64, i32::MAX as f64) as i32,
        );
        imu.prev_heading = current_heading;
    }

    // --- GPS ---
    // Emit a new noisy fix at a configurable rate (default 500 ms → 2 Hz).
    // Set to 0 via --gps-interval to disable GPS entirely.
    const GPS_SIGMA: f64 = 0.1;         // metres
    const GPS_SCALE: f64 = 1_000_000.0; // scale factor: store as µm integer

    let gps_interval = gps_interval_ms();
    if gps_interval > 0
        && now_ms.wrapping_sub(gps_last_fix_ms()) >= gps_interval
    {
        let pos = robot.position();
        let gx = ((pos.x() + gaussian(GPS_SIGMA)) * GPS_SCALE) as i32;
        let gy = ((pos.y() + gaussian(GPS_SIGMA)) * GPS_SCALE) as i32;
        set_gps_fix(gx, gy, now_ms);
        set_gps_last_fix_ms(now_ms);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    // --- gaussian() ---

    #[test]
    fn test_gaussian_mean_near_zero() {
        // Over 10 000 samples the sample mean should be within ±0.05 of zero
        // (standard error ≈ σ/√N = 1/100 = 0.01, so this is a ~5σ bound).
        let sigma = 1.0_f64;
        let n = 10_000;
        let mean = (0..n).map(|_| gaussian(sigma)).sum::<f64>() / n as f64;
        assert_abs_diff_eq!(mean, 0.0, epsilon = 0.05);
    }

    #[test]
    fn test_gaussian_stddev_matches_sigma() {
        // Sample std dev should be within 5 % of σ — a very safe bound for N = 10 000.
        let sigma = 1.0_f64;
        let n = 10_000;
        let samples: Vec<f64> = (0..n).map(|_| gaussian(sigma)).collect();
        let mean = samples.iter().sum::<f64>() / n as f64;
        let variance = samples.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64;
        let stddev = variance.sqrt();
        assert_abs_diff_eq!(stddev, sigma, epsilon = 0.05 * sigma);
    }

    // --- mars_rover_encoder_ticks() ---

    /// Verify that calling mars_rover_encoder_ticks atomically returns the accumulated value
    /// and resets the counter to zero so the next call returns 0.
    #[test]
    fn test_encoder_ticks_swap_resets_to_zero() {
        ENCODER_TICKS[2].store(17, Ordering::Relaxed);
        assert_eq!(mars_rover_encoder_ticks(2), 17);
        assert_eq!(mars_rover_encoder_ticks(2), 0); // counter was reset on the first read
    }

    /// Values outside i16 range are clamped, not truncated.
    #[test]
    fn test_encoder_ticks_clamped_to_i16() {
        ENCODER_TICKS[3].store(i16::MAX as i32 + 100, Ordering::Relaxed);
        assert_eq!(mars_rover_encoder_ticks(3), i16::MAX);

        ENCODER_TICKS[3].store(i16::MIN as i32 - 100, Ordering::Relaxed);
        assert_eq!(mars_rover_encoder_ticks(3), i16::MIN);
    }

    /// An out-of-range wheel index returns 0 without panicking.
    #[test]
    fn test_encoder_ticks_invalid_wheel_returns_zero() {
        assert_eq!(mars_rover_encoder_ticks(4), 0);
        assert_eq!(mars_rover_encoder_ticks(255), 0);
    }

    // --- encoder delta arithmetic ---

    /// Verify the radians-to-teeth conversion that control() applies each frame.
    /// One full revolution (2π rad) should produce exactly 48 teeth.
    /// Half a revolution should produce 24 teeth.
    /// One tooth's worth of rotation (2π/48 rad) should produce 1 tooth.
    #[test]
    fn test_delta_to_teeth_conversion() {
        let teeth_from_delta = |delta_rad: f64| -> i32 {
            ((delta_rad / (2.0 * std::f64::consts::PI)) * 48.0).round() as i32
        };

        assert_eq!(teeth_from_delta(2.0 * std::f64::consts::PI), 48);
        assert_eq!(teeth_from_delta(std::f64::consts::PI), 24);
        assert_eq!(teeth_from_delta(2.0 * std::f64::consts::PI / 48.0), 1);
        assert_eq!(teeth_from_delta(-(2.0 * std::f64::consts::PI / 48.0)), -1);
        assert_eq!(teeth_from_delta(0.0), 0);
    }
}
