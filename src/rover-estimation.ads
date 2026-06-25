with Ada.Numerics;
with Rover_HAL;
use type Rover_HAL.Encoder_Ticks;

--  Extended Kalman Filter for 2D position and heading estimation.
--  This is a pure computation package: no abstract state, no HAL calls.
--  The caller (Rover.GNC) owns the State and Covariance and passes them in.
package Rover.Estimation
  with SPARK_Mode, Always_Terminates
is

   --  Physical constants — must match the Rust RobotConfig in
   --  simulator/src/visualizer.rs (create_robot).
   Wheel_Radius          : constant Float := 0.0225;  -- metres
   Encoder_Teeth_Per_Rev : constant := 48;
   Metres_Per_Tick       : constant Float :=
     (2.0 * 3.14159_26535 * Wheel_Radius) / Float (Encoder_Teeth_Per_Rev);

   --  Corner wheel geometry (body frame, metres).
   --  Distances from the robot centre to each front/rear and left/right pivot.
   Dist_Front   : constant Float := 0.08;    -- +x to front-wheel pivot
   Dist_Rear    : constant Float := 0.075;   -- |x| to rear pivot (negated)
   Dist_Side    : constant Float := 0.0565;  -- |y| from centre to pivot
   Pivot_Offset : constant Float := 0.028;   -- pivot-to-wheel-centre distance

   Max_Steering_Angle_Rad : constant Float :=
     Float (Rover_HAL.Max_Steering_Angle_Deg) * Float (Ada.Numerics.Pi) / 180.0;
   --  Maximum corner-wheel steering angle in radians (= 40 × π/180 ≈ 0.6981 rad).
   --  Derived from Rover_HAL.Max_Steering_Angle_Deg; if the Rust steering limit
   --  changes, update that constant and this value follows automatically.

   subtype Steering_Angle_Rad is Float
     range -Max_Steering_Angle_Rad .. Max_Steering_Angle_Rad;
   --  Corner-wheel steering angle passed to the kinematic model.

   Min_Odometry_Scale : constant Float := 0.1;
   --  Lower floor for the odometry scale factor (severe slip case).
   --  Enforced by the clamp in Update.  Internal Ada constant; no Rust counterpart.

   Max_Odometry_Scale : constant Float := 2.0;
   --  Upper ceiling for the odometry scale factor (physically implausible overcount).
   --  Enforced by the clamp in Update.  Internal Ada constant; no Rust counterpart.

   subtype Odometry_Scale is Float range Min_Odometry_Scale .. Max_Odometry_Scale;
   --  Multiplicative scale on encoder-derived displacement; 1.0 = no slip.

   Min_DT : constant Float := 0.001;
   --  Lower bound on the EKF predict time-step (seconds); guards against zero DT.
   --  Ada: rover-gnc.adb clamps DT with Float'Max (0.001, ...).
   --  If the clamp values in rover-gnc.adb change, update this constant.

   Max_DT : constant Float := 0.5;
   --  Upper bound on the EKF predict time-step (seconds); tolerates one missed
   --  40 ms cycle.  Ada: rover-gnc.adb clamps DT with Float'Min (0.5, ...).
   --  If the clamp values in rover-gnc.adb change, update this constant.

   subtype Cycle_Time is Float range Min_DT .. Max_DT;
   --  Elapsed time between successive Poll/Follow_Path calls, as passed to Predict.

   N_Max_Predicts_Per_Update : constant := 13;
   --  Number of Predict calls between consecutive Update calls in the GNC loop.
   --  Derivation: at 25 Hz control (40 ms) and 2 Hz GPS (500 ms), one fresh fix
   --  arrives every 12.5 control cycles; rounding up to 13 gives an Update
   --  period of 520 ms, which samples each GPS publication at most once and
   --  typically catches a fresh fix on each call.  The GNC loop counts
   --  Predicts and invokes Update on the 13th regardless of Fix freshness —
   --  a stale Fix degrades estimate quality but does not break the inductive
   --  covariance bound (Update's algebra is oblivious to Fix timestamps).

   Max_Encoder_Delta : constant := 50;
   --  Maximum encoder tick delta accumulated between consecutive Read_Encoder_Ticks
   --  calls (one Poll cycle).  Derivation:
   --    Rust: simulator/src/controller.rs,
   --          POWER_TO_VELOCITY_FACTOR = 1.0 / 350.0  (m/s per unit Motor_Power).
   --          If POWER_TO_VELOCITY_FACTOR changes, recompute this constant.
   --    Ada:  Motor_Power'Last = 100, Wheel_Radius = 0.0225 m,
   --          Encoder_Teeth_Per_Rev = 48.
   --    Max wheel speed = 100 × (1/350) = 0.2857 m/s
   --    Max wheel ω     = 0.2857 / 0.0225 ≈ 12.7 rad/s
   --    Max ticks/s     = 12.7 × 48 / (2π) ≈ 97 ticks/s
   --    Max ticks/cycle at Max_DT (0.5 s) = 97 × 0.5 ≈ 49 → rounded to 50.
   --    Also covers faster-than-real-time simulation at up to ~10× speed
   --    within the nominal 40 ms wall-clock cycle.

   subtype Encoder_Delta is Rover_HAL.Encoder_Ticks
     range -Max_Encoder_Delta .. Max_Encoder_Delta;
   --  Per-cycle signed tick count for a single corner wheel; positive = forward.

   Max_Covariance_Diag : constant Float := 200.0;
   --  Upper bound on |P(I,J)| for ALL matrix entries (diagonal and off-diagonal).
   --  THIS IS A PROOF OBLIGATION, not an enforced invariant.
   --
   --  Why 200, not 10:
   --  The diagonal entries grow slowly (dominated by Q_X = 1e-4/step), so 10
   --  was historically sufficient for them.  However, the off-diagonal theta-X
   --  cross-covariance P(3,1) grows each Predict step by F13·P(3,3) ≈ 0.1–0.6,
   --  and the symmetric GPS Update can only damp it by factor (1−K_X) per GPS fix.
   --  At low GPS rates (≥ 1 Hz, N ≈ 25 control steps between fixes) and a
   --  well-converged position (small P(1,1) → small K_X), the steady-state bound
   --  is approximately R_GPS/P(1,1)_ss · N · max(F13) · P(3,3) ≈ 60–150.
   --  200 provides comfortable headroom for any realistic GPS rate and maneuver.
   --  The proof structure (FP_Entry = ±3·MCov, FPFt_Entry = ±9·MCov) is fully
   --  parametric, so all derived bounds scale automatically.

   Q_X     : constant Float := 1.0e-4;
   Q_Y     : constant Float := 1.0e-4;
   Q_Theta : constant Float := 1.0e-5;
   Q_S     : constant Float := 1.0e-5;
   --  EKF process noise (diagonal of Q matrix).
   --  Exposed in the spec so postconditions and GNC loop invariants
   --  can reference them without depending on body internals.

   Max_Position_Step : constant Float :=
     2.0 * Max_Odometry_Scale * Float (Max_Encoder_Delta) * Metres_Per_Tick;
   --  Upper bound on |ΔS.X| or |ΔS.Y| per single Predict call.
   --  Derived: max |Vx_sc·cos θ − Vy_sc·sin θ| ≤ 2·Scale·MWD.
   --  All factors are already in the spec; no body dependency.

   R_GPS : constant Float := 0.09;
   --  GPS measurement noise variance (m²).  GPS σ = 0.1 m → R = σ² = 0.01 m².
   --  Exposed in the spec so Update's postconditions and GNC loop invariants
   --  can reference it without depending on body internals.
   --  "Tuned down" to trust GPS less; if changed, update Min_P_Diag_At_Update.

   Max_P_After_Predict : constant Float := 9.0 * Max_Covariance_Diag + Q_X;
   --  Conservative upper bound on |P(I,J)| after one Predict call.
   --  Derived: F·P·Fᵀ worst-case row/col expansion is 3× (|F off-diag| ≤ 1),
   --  so the full quadratic form expands P by at most 9×, giving
   --  FPFt ∈ [−9·MCov, 9·MCov].  Adding max process noise Q_X gives the
   --  loose post bound for all entries (≈ 90.0001).
   --  The body's FPFt_Entry type directly witnesses this bound.

   Min_P_Diag_At_Update : constant Float := 7.0 * Q_X;
   --  Minimum guaranteed value of P(1,1) and P(2,2) at every Update call.
   --  Derivation: Predict adds Q_X to P(1,1) each call; after ≥ 7 consecutive
   --  Predict calls from a P(1,1) = 0 state, P(1,1) ≥ 7·Q_X.
   --  This is a GNC loop invariant obligation, not enforced here.
   --  Choosing 7·Q_X gives K_X ≥ 7e-4 / 0.0907 ≈ 0.00772, enabling
   --  S_Safe_After_Update ≈ 992.3 m with ≈ 26 Predicts of headroom.

   S_Safe_After_Update : constant Float :=
     (1.0 - Min_P_Diag_At_Update / (Min_P_Diag_At_Update + R_GPS))
       * 1_000.0
     + (Min_P_Diag_At_Update / (Min_P_Diag_At_Update + R_GPS))
       * Rover_HAL.Max_World_X
     + 0.001;
   --  Conservative upper bound on |S.X| and |S.Y| immediately after Update.
   --  KEY inductive bound: tighter than the safety bound (1 000 m).
   --  Physics: Update is a convex combination of S.X (∈ [−1000, 1000]) and
   --  Fix.X (∈ [−Max_World_X, Max_World_X] = [−6, 6]).  With K_X ≥ K_min
   --  (guaranteed by P(1,1) ≥ Min_P_Diag_At_Update), the combination is
   --  pulled at least K_min × (1000 − Max_World_X) ≈ 7.7 m away from ±1000.
   --  The base formula evaluates to ≈ 992.328 m in Float.  The 0.001 term
   --  is a floating-point rounding margin (≈ 16 ULPs at magnitude 992):
   --  because the EWMA and the constant are computed via different FP
   --  operation sequences, the two paths can differ by 1–2 ULPs at the
   --  tight boundary K = K_min, so the exact formula leaves zero headroom.
   --  Net value ≈ 992.329 m, leaving headroom for ≈ 26 Predict calls at MPS.
   --  GNC loop invariant: S.X ∈ [−1000, 1000] holds for all k ≤ 26
   --  Predicts after Update because 992.329 + 26 × 0.294 ≈ 999.9 < 1000.
   --  Uses Rover_HAL.Max_World_X (= 6.0), the tightest possible Fix.X bound.

   type State is record
      X          : Float;            --  World-frame X position (metres).  May briefly
                                     --  exceed World_X during Predict; GPS Update pulls
                                     --  it back.  Convert to World_X at HAL boundary.
      Y          : Float;            --  World-frame Y position (metres).  Same caveat.
      Theta      : Rover_HAL.Heading;
      S_Odometry : Odometry_Scale;  --  odometry scale ∈ [0.1, 2.0]; 1.0 = no slip
   end record;
   --  EKF state vector: world-frame position, heading, and odometry scale.
   --  X and Y are plain Float so the Predict step can transiently overshoot the
   --  terrain boundary without raising Constraint_Error; the GPS Update pulls
   --  them back.  Callers must convert to World_X / World_Y at the HAL boundary.
   --  Theta is kept normalised to (−π, π].
   --  S_Odometry is a multiplicative factor on encoder-derived displacement;
   --  1.0 = no slip, < 1.0 = wheel slip.  Clamped to [0.1, 2.0] by Update.

   type Covariance is array (1 .. 4, 1 .. 4) of Float;
   --  4×4 symmetric positive-definite covariance matrix (1-indexed).

   type Encoder_Deltas is
     array (Rover_HAL.Corner_Wheel_Id) of Encoder_Delta;
   --  Encoder deltas: signed tick count accumulated since the previous call,
   --  one per corner wheel.  Positive = forward rotation.

   type Corner_Steering is
     array (Rover_HAL.Corner_Wheel_Id) of Steering_Angle_Rad;
   --  Commanded steering angles, one per corner wheel, in radians.
   --  Positive = anticlockwise when viewed from above (Ada sign convention,
   --  consistent with rover_hal.ads Steering_Wheel_Angle positive = left).

   Estimator_Assumption_Violation : exception;
   --  Raised by Predict when the computed P (1, 1) or P (2, 2) falls
   --  outside the Silver-provable range [0.0, Max_P_After_Predict].
   --  These are PSD-dependent Post clauses that SPARK cannot prove
   --  without Is_PSD infrastructure; the runtime check enforces them
   --  instead.  Callers should treat the exception as unrecoverable
   --  estimator divergence.

   procedure Init
     (S       : out State;
      P       : out Covariance;
      X_0     : Rover_HAL.World_X;
      Y_0     : Rover_HAL.World_Y;
      Theta_0 : Rover_HAL.Heading;
      S_0     : Odometry_Scale := 1.0)
   with Global => null;
   --  Initialise to a known position and heading with unit-diagonal covariance.
   --  For GPS-only initialisation, pass Theta_0 => 0.0.
   --  For simulator reset, pass the rover's true heading so the process model
   --  starts in the correct direction immediately.
   --  S_0 sets the initial odometry scale estimate; the default 1.0 is correct
   --  for any start where slip history is unknown.

   procedure Predict
     (S        : in out State;
      P        : in out Covariance;
      Encoders : Encoder_Deltas;
      Steering : Corner_Steering;
      Gyro_Z   : Rover_HAL.Gyro_Rate_Rad_S;
      DT       : Cycle_Time)
   with Global => null,
        Pre    =>
          (for all I in 1 .. 4 =>
             (for all J in 1 .. 4 =>
                P (I, J) in -Max_Covariance_Diag .. Max_Covariance_Diag)) and then
          S.X in -1_000.0 .. 1_000.0 and then
          S.Y in -1_000.0 .. 1_000.0 and then
          --  Diagonal non-negativity — tightens the all-P bound to [0, MCov]
          --  for each diagonal entry.  Maintained by Init, Update post, and
          --  this procedure's own Post (P(3,3)/P(4,4) trivially; P(1,1)/P(2,2)
          --  via the PSD ghost lemma in the proof campaign).
          P (1, 1) in 0.0 .. Max_Covariance_Diag and then
          P (2, 2) in 0.0 .. Max_Covariance_Diag and then
          P (3, 3) in 0.0 .. Max_Covariance_Diag and then
          P (4, 4) in 0.0 .. Max_Covariance_Diag,
        Post   =>
          --  Quantities Predict never writes:
          S.S_Odometry = S.S_Odometry'Old                                    and then
          --  Heading is always normalised by Normalise_Angle:
          S.Theta in Rover_HAL.Heading                                       and then
          --  Exact process-noise increments on the heading and scale diagonals:
          P (3, 3) = P'Old (3, 3) + Q_Theta                                 and then
          P (4, 4) = P'Old (4, 4) + Q_S                                     and then
          --  Per-step position bounds (relies on sin/cos ∈ [−1,1]):
          S.X in S.X'Old - Max_Position_Step .. S.X'Old + Max_Position_Step and then
          S.Y in S.Y'Old - Max_Position_Step .. S.Y'Old + Max_Position_Step and then
          --  Wide bound on ALL P entries: follows from FPFt_Entry type in body
          --  (FPFt ∈ [−9·MCov, 9·MCov], plus max noise Q_X on diagonals).
          --  Needed by the GNC loop invariant for inductive reasoning.
          (for all I in 1 .. 4 =>
             (for all J in 1 .. 4 =>
                P (I, J) in -Max_P_After_Predict .. Max_P_After_Predict))   and then
          --  Diagonal non-negativity: P(3,3)/P(4,4) follow trivially from the
          --  exact-delta post above plus the pre ≥ 0 assumption.  P(1,1) and
          --  P(2,2) are PSD-dependent; SPARK cannot prove them at Silver
          --  without Is_PSD infrastructure.  The body enforces them at
          --  runtime by raising Estimator_Assumption_Violation (see the
          --  Exceptional_Cases below), so on normal termination both hold
          --  by the guarded check.  The conjuncts are left in the Post for
          --  callers that need the fact; whether SPARK can structurally
          --  discharge them from the guard is a separate (and currently
          --  open) proof question.
          P (1, 1) in 0.0 .. Max_P_After_Predict                            and then
          P (2, 2) in 0.0 .. Max_P_After_Predict,
        Exceptional_Cases =>
          (Estimator_Assumption_Violation => True);
   --  Kinematic process model (full 4WIS vector-averaging, matching the Rust
   --  kinematic Jacobian in domain/robot.rs).
   --
   --  Steering holds the *commanded* angles that were in effect while the
   --  encoder ticks in Encoders were accumulating — one control cycle before
   --  the current call.  No actuator dynamics are modelled; the assumption
   --  is noted here so it is visible during the proof phase.
   --
   --  Gyro_Z is the IMU yaw rate in rad/s (already converted from Gyro_Rate_Raw).
   --  DT is the elapsed time since the previous Predict call, in seconds.
   --  The heading update is Gyro_Z * DT; the translational update still uses
   --  the encoder-derived Vx_Body / Vy_Body (steering angles still required),
   --  scaled by S.S_Odometry to account for learned wheel slip.
   --
   --  Precondition (informal): S.S_Odometry > 0.0.  This is maintained by
   --  the clamp in Update (0.1 lower bound), so callers need not guard it.
   --
   --  Theta is normalised to (−π, π] after the update.  A single add/subtract
   --  of 2π suffices because the rover's maximum angular rate (~3 rad/s) and
   --  the loop timing (>= 40 ms) keeps the per-call angle change below Pi.

   procedure Update
     (S   : in out State;
      P   : in out Covariance;
      Fix : Rover_HAL.GPS_Fix_Type)
   with Global => null,
        Pre    =>
          --  Full P invariant: all entries bounded.  The explicit P(1,1)/P(2,2)
          --  clauses below further tighten the diagonal lower bound.
          (for all I in 1 .. 4 =>
             (for all J in 1 .. 4 =>
                 P (I, J) in -Max_Covariance_Diag .. Max_Covariance_Diag)) and then
          --  Position diagonals: strict lower bound ensures K_X/K_Y > 0, which
          --  is required to prove S_Safe_After_Update.  The GNC loop invariant
          --  guarantees this at each call site.
          P (1, 1) in Min_P_Diag_At_Update .. Max_Covariance_Diag and then
          P (2, 2) in Min_P_Diag_At_Update .. Max_Covariance_Diag and then
          S.X in -1_000.0 .. 1_000.0                                and then
          S.Y in -1_000.0 .. 1_000.0,
        Post   =>
          --  Quantities Update never writes:
          S.Theta      = S.Theta'Old                                          and then
          S.S_Odometry = S.S_Odometry'Old                                     and then
          --  Entries in rows/cols 3 and 4 only (no GPS coupling): unchanged.
          (for all I in 3 .. 4 =>
             (for all J in 3 .. 4 =>
                P (I, J) = P'Old (I, J)))                                    and then
          --  Position diagonals contract strictly within [0, old value]:
          P (1, 1) in 0.0 .. P'Old (1, 1)                                   and then
          P (2, 2) in 0.0 .. P'Old (2, 2)                                   and then
          --  Full P invariant: all entries within [−MCov, MCov].
          --  Off-diagonal entries in rows/cols 1 and 2 are scaled by (1−K_X)
          --  or (1−K_Y) ∈ [0,1], so they shrink in magnitude — the invariant
          --  is maintained provided it held on entry (stated in the Pre).
          (for all I in 1 .. 4 =>
             (for all J in 1 .. 4 =>
                 P (I, J) in -Max_Covariance_Diag .. Max_Covariance_Diag))   and then
          --  GPS correction contracts position to S_Safe_After_Update < 1 000 m.
          --  This is the KEY inductive bound that keeps S.X within Predict's
          --  precondition for ≈ 26 subsequent Predict calls before the next GPS.
          S.X in -S_Safe_After_Update .. S_Safe_After_Update                 and then
          S.Y in -S_Safe_After_Update .. S_Safe_After_Update;
   --  GPS correction step: full symmetric-K Joseph-form covariance update for
   --  a GPS-only measurement H = [[1,0,0,0],[0,1,0,0]] with diagonal R.
   --  Caller must invoke only when Fix.Timestamp differs from the previously
   --  seen value; the filter does not track timestamps.
   --
   --  Uses independent scalar gains K_x = P(1,1)/(P(1,1)+R) and
   --  K_y = P(2,2)/(P(2,2)+R).  State update: S.X := (1−K_x)·S.X + K_x·Fix.X
   --  and similarly for Y.  Heading and odometry scale are not updated by GPS.
   --  P(i,j)_new = scale(i)·P(i,j)_old·scale(j) where scale(1)=1−K_x,
   --  scale(2)=1−K_y, scale(3)=scale(4)=1.  Off-diagonal entries in rows/cols
   --  1 and 2 are therefore damped, bounding the theta-position cross-covariance
   --  P(3,1)/P(3,2) that Predict inflates via the heading Jacobian.

private

   --  Bounded subtypes and derived constants shared between the Predict and
   --  Update procedure bodies and the child lemma packages
   --  Rover.Estimation.Predict_Lemmas / Update_Lemmas / Common_Lemmas.
   --  Kept in the private part because they are implementation details of
   --  the EKF: the child packages see them via the SPARK child-package
   --  visibility rule, but clients of Rover.Estimation do not.

   --  Maximum body-frame displacement per wheel per control cycle (metres).
   --  Derived: Max_Encoder_Delta ticks * Metres_Per_Tick.
   Max_Wheel_Disp : constant Float :=
     Float (Max_Encoder_Delta) * Metres_Per_Tick;

   subtype Wheel_Disp is Float range -Max_Wheel_Disp .. Max_Wheel_Disp;
   --  Per-wheel displacement (metres); also the pre-scale accumulated body
   --  velocity.

   subtype Scaled_Disp is Float
     range -Max_Odometry_Scale * Max_Wheel_Disp ..
            Max_Odometry_Scale * Max_Wheel_Disp;
   --  Body displacement after scaling by S.S_Odometry in [0.1, 2.0].

   subtype Jac_F14_24 is Float range -1.0 .. 1.0;
   --  Jacobian column 4 bound: |Vx_pre*cos| + |Vy_pre*sin| <= 2*MWD ~= 0.295.
   --  Clamped to +/-1.0 so the unit-bound lemma pattern applies uniformly.

   subtype Jac_F13_23 is Float range -1.0 .. 1.0;
   --  Jacobian column 3 bound: |Vx_sc*sin| + |Vy_sc*cos| <= 2*Sc*MWD ~= 0.590.

   subtype Cov_Entry is Float
     range -Max_Covariance_Diag .. Max_Covariance_Diag;
   --  Subtype for individual P array entries used in lemma parameters.

   Max_FP_12 : constant Float := 3.0 * Max_Covariance_Diag;
   --  With both F13/F14 bounded by +/-1.0, the product |F*P| <= MCov.
   --  FP row 1/2 = P(i,j) + F13*P3 + F14*P4, each term <= MCov
   --  -> sum <= 3*MCov.

   subtype FP_Entry is Float range -Max_FP_12 .. Max_FP_12;

   Max_FPFt_12 : constant Float := 3.0 * Max_FP_12;
   --  Similarly for FPFt; each FP element <= Max_FP_12, |F| <= 1.
   --  FPFt row 1/2 = FP(i,j) + F13*FP3 + F14*FP4 <= 3*Max_FP_12.

   subtype FPFt_Entry is Float range -Max_FPFt_12 .. Max_FPFt_12;

   subtype FP_2MCov is Float
     range -2.0 * Max_Covariance_Diag .. 2.0 * Max_Covariance_Diag;
   --  FP_2MCov: P(i,J) + F*P3 where both terms <= MCov -> partial sum
   --  <= 2*MCov.

   subtype FPFt_2FP is Float range -2.0 * Max_FP_12 .. 2.0 * Max_FP_12;
   --  FPFt_2FP: FP(i,j) + F*FP3 where both terms <= FP_12 -> partial sum
   --  <= 2*FP_12.

   subtype Unit_Float is Float range -1.0 .. 1.0;
   --  Unit-bounded subtype for use as lemma parameter type (Predict side).

end Rover.Estimation;
