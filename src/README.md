# `src/` — Ada / SPARK control software

This is the Mars Rover control software. The same sources compile against two
HAL backends: `binding/` for the desktop simulator (FFI to Rust) and `device/`
for the RP2040 (real motors, sonar, gamepad, display). For the workspace
overview see the [repository root README](../README.md).

## Package tour

### `Rover` (`rover.ads`)

Top-level package. Defines the safety constant `Safety_Distance` and the ghost
function `Cannot_Crash`:

```ada
function Cannot_Crash return Boolean is
  (if Rover_HAL.Get_Sonar_Distance < Safety_Distance
     and then Rover_HAL.Get_Turn = Rover_HAL.Straight
   then
     Rover_HAL.Get_Power (Rover_HAL.Left)  <= 0 and then
     Rover_HAL.Get_Power (Rover_HAL.Right) <= 0);
```

This is the demo's headline safety property: *when the sonar sees an obstacle
closer than the safety distance and the rover is pointed straight at it, the
motors may not be commanded forward*. The autonomous and remote-controlled
modes discharge this contract at proof level 1. The GNC path-follower is the
next mode that needs to.

### `Rover_HAL` (`rover_hal.ads`)

Abstract hardware interface. Two bodies:

- `src/binding/rover_hal.adb` — calls `mars_rover_*` functions exported by
  `mars-rover-ada-link`. Used by the desktop simulator, headless driver, and
  replay harness.
- `src/device/rover_hal.adb` — drives real peripherals via the Pico BSP
  (motors over I²C/PCA9685, sonar on GPIO, gamepad over I²C, SSD1306 display).

Surface split by concern:

| Group | Kinds |
|---|---|
| World geometry | `World_X`, `World_Y`, `Heading` subtypes; `Terrain_Half_Width/Height`, `GPS_Position_Margin` |
| Timer | `Clock`, `Delay_Microseconds`, `Delay_Milliseconds` |
| Sonar | `Sonar_Distance` (side-effecting volatile function, latch at top of cycle) |
| Encoders | `Read_Encoder_Ticks` (delta-on-read), `EKF_Reset_Pending` |
| GPS | `GPS_Fix` (idempotent; freshness via `Timestamp`) |
| IMU | `Read_IMU_Gyro_Z` — `Integer_16`, `×8192` LSB/(rad/s) |
| Waypoints | `Waypoint_Count`, `Get_Waypoint` |
| Mast | `Set_Mast_Angle` |
| Remote | `Update` returns a `Buttons_State` |
| Motors / steering | `Set_Wheel_Angle`, `Set_Turn`, `Set_Power` |
| Estimate feedback | `Set_Estimated_Position`, `Report_GNC_State` |
| Display | `Set_Display_Info` |

`Encoder_Ticks`, `GPS_Fix_Type`, `Gyro_Rate_Raw`, and friends follow the MCU
idiom: integer counts or fixed-point scales, not Floats. Conversion to SI
units happens inside `Rover.Estimation`. Constants that must agree with Rust
(e.g. `Gyro_Rate_Scale = 8192.0`, `Metres_Per_Tick`, terrain half-width) are
called out with "if this changes in Rust, update here" comments.

### `Rover.Estimation` (`rover-estimation.ads/adb`)

4-state Extended Kalman Filter: state vector `(X, Y, Theta, S_Odometry)`.

- `Init` — set state to a known pose; unit-ish diagonal covariance.
- `Predict` — kinematic process model; heading update from gyro; translational
  update from encoder ticks scaled by `S_Odometry` (the learned slip factor).
- `Update` — GPS correction step with full symmetric-K Joseph-form covariance
  update. Measurement is `(X, Y)` only; heading and scale are corrected
  indirectly through covariance coupling.

The package is pure computation: no abstract state, no HAL calls. The caller
(`Rover.GNC`) owns the `State` and `Covariance` and passes them in. All
physical constants (wheel radius, teeth/rev, body geometry) and derived
bounds live in the spec so they can be referenced from contracts and from
GNC loop invariants.

**Proof state**: Gold. All runtime-error-freedom checks discharge at level 2.
Some postconditions are proved, but they're insufficient to prove Silver in GNC.
The Post clauses that require Positive-Semi-Definiteness reasoning
(`P(1,1)`/`P(2,2)` non-negativity after `Predict`) are enforced at runtime via
the `Estimator_Assumption_Violation` exception rather than proved
structurally. See `proof-status.md` for the full audit trail.

Children:

- `Rover.Estimation.Common_Lemmas` — utility lemmas shared by both sides.
- `Rover.Estimation.Predict_Lemmas` — FP / FPFt bound lemmas used by `Predict`.
- `Rover.Estimation.Update_Lemmas` — scale-chain + Joseph-form lemmas used by `Update`.

### `Rover.GNC` (`rover-gnc.ads/adb`)

Guidance, Navigation, Control. Two entry points:

- `Poll (Steering)` — one cycle of the EKF: read encoders + GPS + IMU, call
  `Predict`, run `Update` when a fresh GPS fix is seen, push the estimate to
  the visualiser via `Set_Estimated_Position`. Called from every control mode
  so the EKF runs continuously.
- `Follow_Path` — drive through the waypoints loaded at the HAL. Loops:
  `Poll`, read target waypoint, call `Rover.Path_Following.Compute` for
  desired steering + power, sonar-preempt, `Set_Power` / `Set_Wheel_Angle`.
  Returns when waypoints are exhausted.

Both procedures declare `Exceptional_Cases => (Estimator_Assumption_Violation
=> True)` — if the EKF invariants are violated at runtime, the exception
propagates up to `Rover.Tasks`, which is expected to halt the rover.

**Proof state**: Silver. Runtime checks discharged at level 2 with a small
set of runtime guards (`In_Position_Envelope`, `All_P_In_Range`,
`Predict_Diag_OK`, `Update_Diag_OK`) that raise
`Estimator_Assumption_Violation` on violation. See `proof-status.md`.

### `Rover.Path_Following` (`rover-path_following.ads/adb`)

Direct-aim waypoint follower. Pure computation (`Global => null`). Given the
current estimate and the target waypoint, returns symmetric 4WIS steering
angles (front wheels toward the target, rear wheels counter-steering) clamped
to ±40°, plus a motor power. The main limitation is documented in-line: no
cross-track correction, so lateral drift is not actively fought — the rover
re-aims at every cycle.

### `Rover.Mast_Control` (`rover-mast_control.ads/adb`)

Rotatable distance sensor, −60° to +60°. Used by the autonomous and GNC
modes to scan ahead. Independent of the main control flow.

### `Rover.Autonomous` (`rover-autonomous.adb`)

Pre-GNC control mode. Mast-scans for obstacles; if the front is clear, drive
forward; otherwise turn toward the widest gap. Discharges `Cannot_Crash` at
proof level 1.

### `Rover.Remote_Controlled` (`rover-remote_controlled.adb`)

Gamepad-driven manual mode. Auto-returns to autonomous after ~10 s of
inactivity. Also discharges `Cannot_Crash`.

### `Rover.Tasks` (`rover-tasks.ads/adb`)

Top-level dispatcher. Decides which mode to run each cycle: path-following if
waypoints are loaded, otherwise remote-controlled when a gamepad button is
pressed, otherwise autonomous.

## HAL backends

### `binding/` (desktop)

One file: `rover_hal.adb`. Every subprogram calls a `mars_rover_*` function
implemented on the Rust side in `mars-rover-ada-link`. Types that cross the
FFI are scaled integers (`×1_000_000` for world coordinates, `×8192` for gyro
rate); the binding converts to `Float` on the way in and out.

### `device/` (RP2040)

Hardware HAL using the Pico BSP. Per-peripheral children:

| Package | Hardware |
|---|---|
| `Rover_HAL.Motors` | PCA9685 PWM driver over I²C |
| `Rover_HAL.PCA9685` | I²C driver for the PCA9685 |
| `Rover_HAL.I2C` | low-level I²C bring-up |
| `Rover_HAL.Sonar` | HC-SR04 ping-echo on GPIO |
| `Rover_HAL.Remote` | I²C gamepad |
| `Rover_HAL.Screen` | SSD1306 OLED |
| `Rover_HAL.Gui` | framebuffer primitives for the screen |

On hardware the GPS / IMU / waypoints paths are stubs (return 0 / empty).
Adding a real sensor is a matter of filling in the corresponding subprogram
bodies here.

## Building

### Desktop (for the simulator workspace)

```
cd ../simulator_interface
alr build
```

Triggered automatically by `mars-rover-ada-link/build.rs`.

### RP2040 firmware

```
# At repo root
alr build
```

Uses the root `mars_rover.gpr`. Output at `bin/mars_rover`.

## Proofs

```
alr exec -- gnatprove -P mars_rover.gpr
```

The project sets `--no-subprojects --level=1` in its `Prove` package. The
`/gnatprove` skill is the preferred way to work on proofs interactively:
level-2 campaigns per subprogram, tee-and-grep discipline on the output.

- `proof-status.md` — current status by unit and subprogram.
- `campaign-journal.md` — running log of proof campaigns.
