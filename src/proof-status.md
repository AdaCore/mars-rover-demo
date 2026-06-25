# Proof Status: Rover.GNC — Silver proof of Poll and Follow_Path
<!-- Reflect the top-level goal given. Items in the list below are moved from
     Not Started to In Progress to Reviewed and finally to Proved and Finalized. -->

Goal: achieve Silver (AoRTE) proof for `Rover.GNC.Poll` and `Rover.GNC.Follow_Path`
on the simulator_interface GPR at `--level=2`.

Baseline (015de03, body SPARK_Mode => Off): 0 unproved at level=2; GNC +
Path_Following skipped. Rover.Estimation is Gold-proved.

**FINAL STATE**: whole-program (simulator_interface GPR) level=2 proves 783/783
checks with zero medium and zero high messages (down from 805 after the helper
consolidation described below).  Silver achieved.

**Post-review refactor** (user-driven):
- Added `function To_Rad_S (Raw : Gyro_Rate_Raw) return Gyro_Rate_Rad_S` to
  `Rover_HAL` (expression function in the spec); replaces the Float'Max/Min
  clamp on the raw gyro divide.  The clamp was a solver-budget workaround, not
  a correctness necessity (the type design already makes `Gyro_Rate_Raw /
  Gyro_Rate_Scale ⊆ ±4.0` exact); encapsulating into a function with the
  return subtype establishes the bound once at the declaration.
- Added `function To_Scaled_Int32 (V : Console_Coord) return Integer_32` to
  `Rover.GNC` (body-local expression function) for the `Integer_32 (Fix.*
  * Scale)` conversions — same pattern: encapsulate the range check at the
  declaration, free for callers.

Known limitation given by user: the EKF state-evolution invariant required to
establish Predict/Update preconditions *inductively* inside the loops is not
provable at Silver. Mitigation: guard each unprovable precondition clause with
runtime `if not <expr_fn_for_clause> then raise
Rover.Estimation.Estimator_Assumption_Violation; end if;` (split as independent
statements, not combined with `or else` — see diary 2026-04-23).

## Structural (main-agent) changes done

- `rover-gnc.ads`:
  - `Global` for Poll: added `Rover_HAL.IMU_State` (Input).
  - `Global` for Follow_Path: added `IMU_State` (Input); moved `Distance_State`
    from Input to In_Out (Sonar_Distance has Side_Effects); added
    `Rover_HAL.Power_State` (In_Out).
  - Both retain `Exceptional_Cases => (Estimator_Assumption_Violation => True)`.
- `rover-gnc.adb`:
  - Flipped body to `SPARK_Mode` with `Refined_State => (GNC_State => (...10 constituents))`.
  - Removed `Ada.Text_IO` and the diagnostic `Report_EKF_Bounds` (uses Text_IO,
    not in SPARK; purely diagnostic, superseded by the exception mechanism).
  - Latched volatile-function calls out of interfering contexts:
    `Read_IMU_Gyro_Z`, `EKF_Reset_Pending`, `Sonar_Distance` now each go through
    a local temporary.
  - Typed `Reset_X/Y/Th` as `World_X/World_Y/Heading` (they are the return
    types of `EKF_Reset_X/Y/Theta`).
  - Introduced `subtype Predict_Count_Type is Natural range 0 .. N - 1` and
    restructured the Predict→Update branch so that `Predict_Count + 1` never
    exits the subtype (if-equal-to-N-1 fires Update and resets).
  - Introduced `Tick_Total` subtype (`Integer_32` with a `Max_Encoder_Delta`
    margin reserved) and converted `Encoder_Total` to a saturating add with an
    explicit `Ticks`-clamp to the `Encoder_Delta` envelope.
  - Added expression-function helpers (`All_P_In_Range`,
    `In_Position_Envelope`, `Predict_Diag_OK`, `Update_Diag_OK`) used by the
    runtime-guard `if` statements at each Predict/Update call site (split into
    independent statements).

## Current gnatprove assessment (level=2, `-u rover-gnc.adb`)

18 medium checks remain after the structural changes.  Each must be discharged
by its owning subprogram's tactical loop:

| Location | Kind | Owner |
|----------|------|-------|
| `rover-gnc.adb:216` Predict `Gyro_Z` range | input type bound | Poll |
| `rover-gnc.ads:10` Est_P / Est_State constituents, init on exceptional exit | flow | Poll |
| `rover-gnc.adb:395` Predict `Gyro_Z` range | input type bound | Follow_Path |
| `rover-gnc.adb:276` To_HAL `Integer_8 (Clamped)` range × 4 call sites | range | To_HAL (nested) |
| `rover-gnc.adb:444` Get_Waypoint Pre | loop invariant | Follow_Path |
| `rover-gnc.adb:447` `Dx*Dx + Dy*Dy` float overflow × 3 | FP overflow | Follow_Path |
| `rover-gnc.adb:448` `Current_Waypoint_Idx + 1` overflow | integer overflow | Follow_Path |
| `rover-gnc.ads:32` Est_P / Est_State, init on exceptional exit | flow | Follow_Path |

## Proved and Finalized
<!-- Before marking an item complete here, follow the Widen Scope step
     (Strategic Loop Step 5) in workflow.md in the /gnatprove Skill. -->

- [x] Rover.Estimation (level=2, mode=all) — Gold at commit 015de03
- [x] Rover.Autonomous (level=2, mode=all)
- [x] Rover.Tasks (level=2, mode=all)
- [x] Rover.GNC (level=2, mode=all) — unit proves at `-u rover-gnc.adb -f`
      level=2 with 154 checks; whole-program re-verify passes with 805 checks.
  - [x] Rover.GNC.Poll (54 checks at `--limit-subp`)
  - [x] Rover.GNC.Follow_Path (94 checks at `--limit-subp`)
  - [x] Rover.GNC.To_HAL (nested; proved inlined)

## Reviewed

- [x] Rover.GNC.Poll — subagent edits reviewed; no antipatterns.  Float'Max/'Min
      clamp style and local-handler-with-reraise are reused in Follow_Path.
- [x] Rover.GNC.Follow_Path — subagent edits reviewed; no antipatterns.
      `Bounded_Deg` local subtype inside To_HAL is load-bearing (Discovered
      Obligation noted).

## In Progress
<!-- A subagent executes the Tactical Loop for the subprogram below. -->

(none — campaign complete)

## Done (archived)

- [x] Rover.GNC.Poll (level=2, mode=all) — all 54 checks proved at
      `--limit-subp=rover-gnc.ads:10 --level=2`
  - [x] Predict call `Gyro_Z` range check
        — clamped `Gyro_Z` with `Float'Max/'Min` around
        `Rover_HAL.Gyro_Max_Rate_Rad_S` after the raw-divide.
  - [x] Est_State / Est_P initialization on exceptional exit (spec line 10)
        — wrapped the Predict call in a local handler that resets
        `Est_State`/`Est_P` to flow-safe defaults, then re-raises.  No
        handler needed around `Update` (no `Exceptional_Cases` on it).
- [x] Rover.GNC.Follow_Path (level=2, mode=all) — all 94 checks proved at
      `--limit-subp=rover-gnc.ads:29 --level=2`
  - [x] Predict call `Gyro_Z` range check — same `Float'Max/'Min` clamp
        around `Gyro_Max_Rate_Rad_S` as Poll.
  - [x] To_HAL conversion proofs (4 call sites) — tightened To_HAL's
        parameter type from `Float` to `Rover.Estimation.Steering_Angle_Rad`
        (±Max_Steering_Angle_Rad ≈ ±0.6981 rad) AND introduced local
        `subtype Bounded_Deg is Float range -40.0 .. 40.0` for Clamped so
        the Integer_8 conversion discharges without solver timeout.
  - [x] Get_Waypoint precondition — added
        `pragma Loop_Invariant (Current_Waypoint_Idx < Waypoint_Count);`
        at the top of the outer loop.  Establishes from the
        `Waypoint_Count = 0 → return` guard; preserved because every path
        that increments the index exits the loop when it reaches
        Waypoint_Count.
  - [x] `Dx*Dx + Dy*Dy` float overflow — runtime-guarded
        `In_Position_Envelope` check immediately before the waypoint
        distance computation; violation raises
        `Estimator_Assumption_Violation`.  Same carve-out style as Poll.
  - [x] `Current_Waypoint_Idx + 1` overflow — follows from the loop
        invariant above: `Current_Waypoint_Idx < Waypoint_Count` and
        `Waypoint_Count : Natural`, so `+1` fits in Natural/Integer_32.
  - [x] Est_State / Est_P initialization on exceptional exit (spec line 29)
        — wrapped the Predict call in a local handler that resets
        `Est_State`/`Est_P` to flow-safe defaults, then re-raises.  Same
        pattern as Poll.  No handler needed around Update.

## Not Started

(none — both Poll and Follow_Path are proved)

## Discovered Obligations

- [x] Tightened `To_HAL (Rad : Float)` parameter to
      `Rover.Estimation.Steering_Angle_Rad` (±Max_Steering_Angle_Rad).
      Callers in Follow_Path already pass values from
      `Rover.Estimation.Corner_Steering`, which is that subtype.
      Additionally, introduced a local `subtype Bounded_Deg is Float range
      -40.0 .. 40.0` for the Clamped intermediate so the `Integer_8 (Clamped)`
      conversion discharges without solver timeout — the Float'Max/'Min
      bounds alone were not enough for SPARK at level=2 within the time
      budget.
- [x] On exceptional exit from Predict, Est_State and Est_P may be in an
      indeterminate state from SPARK's flow perspective.  Resolved for Poll
      and Follow_Path: wrapped the Predict call in a local handler that
      resets Est_State / Est_P to flow-safe defaults before re-raising.
      Only Predict needs this treatment: `Rover.Estimation.Update` has no
      `Exceptional_Cases` and thus cannot raise.
- [x] `Follow_Path` inherited the same two Poll-local fixes: (a) clamp
      `Gyro_Z` with `Float'Max/'Min` around `Gyro_Max_Rate_Rad_S` after the
      `Float (Raw_Gyro) / Gyro_Rate_Scale` divide, (b) wrap the Predict call
      in a local handler that resets Est_State/Est_P and re-raises.  Only
      the Predict call needs a handler; Update does not raise.
- [x] `Dx*Dx + Dy*Dy` float overflow in Follow_Path resolved by a runtime
      `In_Position_Envelope` guard (carve-out style) placed immediately
      before the waypoint distance computation.  After the guard,
      `Est_State.X/Y ∈ [-1000, 1000]` and WP.X/Y ∈ World_X/Y (≤±6/±4), so
      `Dx*Dx + Dy*Dy` stays well within Float bounds (max ≈ 2e6).
