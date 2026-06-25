# `mars-rover-ada-link`

Owns the Rust ↔ Ada boundary for the desktop simulator workspace.
For the workspace overview see the [repository root README](../../README.md).

This crate is the only place that:

- invokes `alr build` in `../../simulator_interface/` and emits the rustc-link
  directives that pull in `libMars_Rover.a` + `libgnat`,
- defines the `#[no_mangle] extern "C"` callbacks Ada invokes (clock, sonar,
  encoders, IMU, GPS, EKF bridge, waypoints, …),
- owns the atomics those callbacks read and write.

Every other crate in the workspace that drives Ada (the Bevy GUI, the
headless driver, the replay harness) depends on this crate and goes through
its public Rust API. The atomics themselves are private; callers use the
accessor functions exposed at the bottom of `lib.rs`.

## Why `mars-rover-core` does not depend on this crate

`mars-rover-core` is a pure-Rust crate that builds and tests without the Ada
static library in scope. Keeping the link direction one-way (`ada-link`
depends on `core`, not the reverse) means `cargo test -p mars-rover-core`
runs on any box regardless of whether Alire / GNAT Pro / the Pico BSP are
available.

## FFI surface

See `src/lib.rs` for the canonical list. Two broad categories:

**Rust → Ada** (sensor atomics Ada reads):

| Signal | FFI function | Encoding |
|---|---|---|
| Sonar distance | `mars_rover_sonar_distance()` | `u32`, centimetres |
| Encoder ticks | `mars_rover_encoder_ticks(wheel)` | `i16` delta, reset-on-read |
| GPS position | `mars_rover_gps_x/y/timestamp()` | `i32` ×1 000 000 µm; `u32` ms |
| IMU gyro Z | `mars_rover_imu_gyro_z()` | `i16`, ×8192 LSB/(rad/s) |
| Remote control | `mars_rover_controller_state()` | `u16` bitfield |
| EKF reset flag | `mars_rover_ekf_reset_pending()` | consume-on-read bool |
| Waypoints | `mars_rover_waypoint_count/x/y()` | `u32`; `i32` ×1 000 000 |
| Clock | `mars_rover_clock()` | `u64` microseconds since epoch |

**Ada → Rust** (actuator atomics Ada writes):

| Signal | FFI function | Encoding |
|---|---|---|
| Wheel steering | `mars_rover_set_wheel_angle(wheel, side, angle)` | `i8`, degrees |
| Motor power | `mars_rover_set_power(side, power)` | `i8`, −127…127 |
| Mast/sonar angle | `mars_rover_set_mast_angle(angle)` | `i8`, degrees |
| EKF estimate | `mars_rover_set_estimated_position(x, y, θ)` | `i32` ×1 000 000 |
| GNC state report | `mars_rover_report_gnc_state(...)` | cumulative totals, for console |
| Display text | `mars_rover_set_display_info(ptr, len)` | 18-char ASCII |

All coordinate scalings (`×1_000_000` for world metres, `×8192` for gyro
rad/s) are matched in the Ada HAL spec (`src/rover_hal.ads`). Changing a
scale here requires a matching change there.

## Virtual time and determinism

`mars-rover-ada-link` also owns the virtual-clock machinery that makes
headless / replay runs deterministic and faster-than-real-time:

- `enable_virtual_time(initial_us)` — flip `mars_rover_clock()` from the
  wall-clock `EPOCH` to a `VIRTUAL_CLOCK` atomic.
- `set_virtual_clock(us)` / `advance_virtual_clock_by(us)` — harness-controlled
  time.
- `set_delay_ms_hook(|ms| …)` — redirect every `Delay_Milliseconds` call into
  a closure. The replay harness uses this to implement a step-gate barrier;
  the headless lockstep mode uses a variant that advances the clock itself.

`ADA_STEP_US` records Ada's own wall-clock timestamp at each `Delay_Milliseconds`
call (in wall-clock mode only), which is what the CSV logger writes into the
`ada_step_us` column. The replay harness uses that column to reproduce Ada's
DT exactly, which is what keeps the EKF's long-trace divergence bounded.

## Spawning the Ada thread

`spawn_ada_thread()` starts the Ada control loop on a dedicated thread and
returns immediately. Configure `ada-link` (`set_gps_interval_ms`,
`set_noise_seed`, `set_waypoints`, `enable_virtual_time`, `set_delay_ms_hook`)
*before* calling this — the hook and virtual-time settings must be live by
the time the Ada side starts issuing calls.
