with Ada.Numerics;
with Rover.Path_Following;
with Rover_HAL;  use Rover_HAL;

package body Rover.GNC
  with SPARK_Mode,
       Refined_State =>
         (GNC_State =>
           (Est_State,
            Est_P,
            Filter_Initialized,
            Last_GPS_Timestamp,
            Predict_Count,
            Encoder_Total,
            Last_GPS_X,
            Last_GPS_Y,
            Current_Waypoint_Idx,
            Last_Poll_Time))
is

   Scale : constant := 1_000_000.0;

   --  EKF state owned by this package.
   Est_State : Rover.Estimation.State :=
     (X => 0.0, Y => 0.0, Theta => 0.0, S_Odometry => 1.0);
   Est_P : Rover.Estimation.Covariance := [others => [others => 0.0]];

   --  Filter lifecycle.
   Filter_Initialized : Boolean     := False;
   Last_GPS_Timestamp : Unsigned_32 := 0;

   --  Fixed-schedule Update: count Predicts since last Update; fire on
   --  reaching N_Max_Predicts_Per_Update (= 13, see rover-estimation.ads).
   --  The Fix used may be stale if no fresh GPS arrived this window —
   --  that is intentional and bounds n for the inductive proof.
   --  Bounded subtype: at the start of each cycle Predict_Count is in
   --  0 .. N - 1.  After the Predict step, the Update-and-reset branch fires
   --  when the counter would reach N, keeping the stored value within the
   --  subtype bound.  See Run_EKF_Step in the body for the invariant.
   subtype Predict_Count_Type is Natural
     range 0 .. Rover.Estimation.N_Max_Predicts_Per_Update - 1;
   Predict_Count : Predict_Count_Type := 0;

   --  Running cumulative encoder ticks per corner wheel (for console display).
   --  Signed 32-bit with a Max_Encoder_Delta margin reserved on each side so
   --  the saturating add below cannot overflow.  The console display never
   --  needs the extreme range; > ~44 000 hours at 1 rev/s, 48 teeth/rev is
   --  still representable.
   Tick_Total_Min : constant Integer_32 :=
     Integer_32'First + Integer_32 (Rover.Estimation.Max_Encoder_Delta);
   Tick_Total_Max : constant Integer_32 :=
     Integer_32'Last  - Integer_32 (Rover.Estimation.Max_Encoder_Delta);
   subtype Tick_Total is Integer_32 range Tick_Total_Min .. Tick_Total_Max;
   type Tick_Totals is array (Corner_Wheel_Id) of Tick_Total;
   Encoder_Total : Tick_Totals := [others => 0];

   --  Last GPS fix received by Ada (*1_000_000 integer format for console).
   Last_GPS_X : Integer_32 := 0;
   Last_GPS_Y : Integer_32 := 0;

   --  Waypoint index for path following; reset to 0 at each Follow_Path call.
   Current_Waypoint_Idx : Natural := 0;

   --  Clock reading at the end of the previous Poll / Follow_Path cycle.
   --  Used to compute DT for the gyro integration in Predict.
   --  Zero on the first call; guarded below.
   Last_Poll_Time : Rover_HAL.Time := 0;

   --  Runtime-check helpers for the Rover.Estimation Predict / Update
   --  preconditions.  These are expression functions (not ghost) so callers
   --  can use them in a runtime `if ... then raise ...` guard; after the
   --  guard SPARK unfolds the body and discharges the matching precondition
   --  clause from the fall-through fact.
   --  See the comment on Estimator_Assumption_Violation in rover-estimation.ads
   --  and the user's carve-out: an inductive invariant that would let SPARK
   --  discharge these preconditions statically is not in scope for Silver.

   MCov : constant Float := Rover.Estimation.Max_Covariance_Diag;

   function All_P_In_Range (P : Rover.Estimation.Covariance) return Boolean is
     (for all I in 1 .. 4 =>
        (for all J in 1 .. 4 => P (I, J) in -MCov .. MCov));

   function In_Position_Envelope (S : Rover.Estimation.State) return Boolean is
     (S.X in -1_000.0 .. 1_000.0 and then S.Y in -1_000.0 .. 1_000.0);

   function Predict_Diag_OK (P : Rover.Estimation.Covariance) return Boolean is
     (P (1, 1) in 0.0 .. MCov and then
      P (2, 2) in 0.0 .. MCov and then
      P (3, 3) in 0.0 .. MCov and then
      P (4, 4) in 0.0 .. MCov);

   function Update_Diag_OK (P : Rover.Estimation.Covariance) return Boolean is
     (P (1, 1) in Rover.Estimation.Min_P_Diag_At_Update .. MCov and then
      P (2, 2) in Rover.Estimation.Min_P_Diag_At_Update .. MCov);

   --  Scale a world coordinate to the console's fixed-point integer form.
   --  The input is conservatively bounded by Max_World_X (>= Max_World_Y),
   --  so a single helper covers both Fix.X and Fix.Y.  Encapsulating the
   --  `V * Scale` multiplication + Integer_32 conversion here lets SPARK
   --  discharge the conversion range check once, not per call site.
   subtype Console_Coord is Float
     range -Rover_HAL.Max_World_X .. Rover_HAL.Max_World_X;

   function To_Scaled_Int32 (V : Console_Coord) return Integer_32 is
     (Integer_32 (V * Scale));

   --  Combined guard: split into three independent if-statements at the call
   --  site (see diary: a single `or else` guard does not reliably propagate
   --  each conjunct to the downstream precondition VC).

   ----------
   -- Poll --
   ----------

   procedure Poll (Steering : Rover.Estimation.Corner_Steering) is
      Fix           : GPS_Fix_Type;
      Ticks         : Encoder_Ticks;
      Deltas        : Rover.Estimation.Encoder_Deltas;
      Reset_X       : Rover_HAL.World_X;
      Reset_Y       : Rover_HAL.World_Y;
      Reset_Th      : Rover_HAL.Heading;
      Now           : Rover_HAL.Time;
      DT            : Float;
      Gyro_Z        : Float;
      Raw_Gyro      : Rover_HAL.Gyro_Rate_Raw;
      Reset_Pending : Boolean;
   begin
      --  Compute cycle time for gyro integration.
      Now := Rover_HAL.Clock;
      DT  := (if Last_Poll_Time = 0 then 0.05
              else Float'Max (0.001, Float'Min (0.5,
                     Float (Now - Last_Poll_Time) /
                     Float (Rover_HAL.Ticks_Per_Second))));
      Last_Poll_Time := Now;

      --  Read gyro rate and convert to rad/s.  Latch the volatile read into
      --  a local so the conversion below is not an interfering context.
      --  Rover_HAL.To_Rad_S carries the Gyro_Rate_Rad_S bound as its return
      --  subtype, so Predict's Gyro_Z input type discharges trivially.
      Raw_Gyro := Read_IMU_Gyro_Z;
      Gyro_Z   := Rover_HAL.To_Rad_S (Raw_Gyro);

      --  1. Latch encoder deltas (resets hardware/simulator counters to zero).
      --  Clamp each read to the Encoder_Delta envelope (hardware may, under
      --  anomalous conditions, return more than Max_Encoder_Delta in one
      --  cycle).  The running total uses a saturating add so it never
      --  overflows the Tick_Total subtype.
      for W in Corner_Wheel_Id loop
         Ticks := Read_Encoder_Ticks (W);
         if Ticks > Rover.Estimation.Max_Encoder_Delta then
            Ticks := Rover.Estimation.Max_Encoder_Delta;
         elsif Ticks < -Rover.Estimation.Max_Encoder_Delta then
            Ticks := -Rover.Estimation.Max_Encoder_Delta;
         end if;
         Deltas (W) := Ticks;

         declare
            Sum : constant Integer_32 :=
              Integer_32 (Encoder_Total (W)) + Integer_32 (Ticks);
         begin
            if Sum > Tick_Total_Max then
               Encoder_Total (W) := Tick_Total_Max;
            elsif Sum < Tick_Total_Min then
               Encoder_Total (W) := Tick_Total_Min;
            else
               Encoder_Total (W) := Sum;
            end if;
         end;
      end loop;

      --  2. Sample GPS fix (before reset check; timestamp needed below).
      Fix := GPS_Fix;

      --  2a. Check for EKF reset request (rover repositioned in simulator).
      --  The reset carries the rover's true pose; initialise immediately.
      --  Latch the volatile-with-side-effects flag so the if-condition is
      --  not an interfering context.
      --  Two stale-data hazards are neutralised:
      --    - Last_GPS_Timestamp is set to Fix.Timestamp so the GPS reading
      --      already in the atomics is not treated as a stale update.
      --    - Deltas are zeroed so Predict ignores pre-teleportation wheel
      --      motion to the freshly initialised state.
      Reset_Pending := EKF_Reset_Pending;
      if Reset_Pending then
         Reset_X  := EKF_Reset_X;
         Reset_Y  := EKF_Reset_Y;
         Reset_Th := EKF_Reset_Theta;
         Rover.Estimation.Init (Est_State, Est_P, Reset_X, Reset_Y, Reset_Th);
         Filter_Initialized := True;
         Last_GPS_Timestamp := Fix.Timestamp;
         Deltas             := [others => 0];
         Encoder_Total      := [others => 0];
         Last_GPS_X         := To_Scaled_Int32 (Reset_X);
         Last_GPS_Y         := To_Scaled_Int32 (Reset_Y);
         Predict_Count      := 0;
      end if;

      --  3. EKF update (Fix already sampled in step 2).
      if not Filter_Initialized then
         --  Wait for the first valid GPS fix, then initialise.
         if Fix.Timestamp /= 0 then
            Rover.Estimation.Init (Est_State, Est_P, Fix.X, Fix.Y, 0.0);
            Last_GPS_Timestamp := Fix.Timestamp;
            Last_GPS_X         := To_Scaled_Int32 (Fix.X);
            Last_GPS_Y         := To_Scaled_Int32 (Fix.Y);
            Filter_Initialized := True;
            Predict_Count      := 0;
         end if;
      else
         --  Predict/Update preconditions that an inductive invariant would
         --  normally discharge are checked at runtime; a violation signals
         --  divergence of the EKF state from its assumptions and raises
         --  Rover.Estimation.Estimator_Assumption_Violation, which
         --  Rover.Tasks.Demo catches and falls through to the
         --  Autonomous / RC loop.  Split into independent guards (a single
         --  `or else` does not reliably propagate each conjunct).
         if not All_P_In_Range (Est_P) then
            raise Rover.Estimation.Estimator_Assumption_Violation;
         end if;
         if not In_Position_Envelope (Est_State) then
            raise Rover.Estimation.Estimator_Assumption_Violation;
         end if;
         if not Predict_Diag_OK (Est_P) then
            raise Rover.Estimation.Estimator_Assumption_Violation;
         end if;

         --  Wrap Predict's exceptional exit in a handler that resets the
         --  EKF state to flow-safe defaults and re-raises.  This preserves
         --  Poll's Exceptional_Cases contract while telling SPARK flow
         --  analysis that Est_State / Est_P are fully written on every
         --  path out of this block.
         begin
            Rover.Estimation.Predict
              (Est_State, Est_P, Deltas, Steering, Gyro_Z, DT);
         exception
            when Rover.Estimation.Estimator_Assumption_Violation =>
               Est_State :=
                 (X => 0.0, Y => 0.0, Theta => 0.0, S_Odometry => 1.0);
               Est_P := [others => [others => 0.0]];
               raise;
         end;

         --  Track fresh GPS fix for console display (decoupled from Update).
         if Fix.Timestamp /= Last_GPS_Timestamp then
            Last_GPS_Timestamp := Fix.Timestamp;
            Last_GPS_X         := To_Scaled_Int32 (Fix.X);
            Last_GPS_Y         := To_Scaled_Int32 (Fix.Y);
         end if;

         --  Fixed-schedule Update every N_Max_Predicts_Per_Update cycles.
         --  Fix may be stale; Update's algebra is oblivious to freshness.
         --  We pre-guard the Update call with the same structure; the
         --  comparison `= N - 1` keeps Predict_Count in its subtype range.
         if Predict_Count = Rover.Estimation.N_Max_Predicts_Per_Update - 1
         then
            if not All_P_In_Range (Est_P) then
               raise Rover.Estimation.Estimator_Assumption_Violation;
            end if;
            if not In_Position_Envelope (Est_State) then
               raise Rover.Estimation.Estimator_Assumption_Violation;
            end if;
            if not Update_Diag_OK (Est_P) then
               raise Rover.Estimation.Estimator_Assumption_Violation;
            end if;
            --  Update does not have Exceptional_Cases, so no handler needed.
            Rover.Estimation.Update (Est_State, Est_P, Fix);
            Predict_Count := 0;
         else
            Predict_Count := Predict_Count + 1;
         end if;

         --  Push EKF estimate to Rust simulator for the visualiser.
         if Est_State.X in World_X'Range
           and then Est_State.Y in World_Y'Range
         then
            Set_Estimated_Position
              (World_X (Est_State.X),
               World_Y (Est_State.Y),
               Est_State.Theta);
         end if;

      end if;

      --  4. Push encoder totals + sensed GPS to the console display.
      Report_GNC_State
        (Last_GPS_X, Last_GPS_Y,
         Encoder_Total (Front_Left),
         Encoder_Total (Front_Right),
         Encoder_Total (Rear_Left),
         Encoder_Total (Rear_Right));
   end Poll;

   ------------
   -- To_HAL --
   ------------

   function To_HAL (Rad : Rover.Estimation.Steering_Angle_Rad)
     return Steering_Wheel_Angle
   is
      Pi      : constant Float := Float (Ada.Numerics.Pi);
      Deg     : constant Float := Float (Rad) * (180.0 / Pi);
      --  Subtype with explicit bounds so SPARK discharges the Integer_8
      --  conversion without timeout.  Float'Max/'Min bound Clamped to
      --  [-40.0, 40.0], well within Integer_8's [-128, 127] range.
      subtype Bounded_Deg is Float range -40.0 .. 40.0;
      Clamped : constant Bounded_Deg :=
        Float'Max (-40.0, Float'Min (40.0, Deg));
   begin
      return Steering_Wheel_Angle (Integer_8 (Clamped));
   end To_HAL;

   -----------------
   -- Follow_Path --
   -----------------

   procedure Follow_Path is
      Ticks         : Encoder_Ticks;
      Deltas        : Rover.Estimation.Encoder_Deltas;
      Fix           : GPS_Fix_Type;
      Reset_X       : Rover_HAL.World_X;
      Reset_Y       : Rover_HAL.World_Y;
      Reset_Th      : Rover_HAL.Heading;
      Steering      : Rover.Estimation.Corner_Steering := [others => 0.0];
      Left_Power    : Motor_Power;
      Right_Power   : Motor_Power;
      WP            : Waypoint_Type;
      Dx            : Float;
      Dy            : Float;
      Now           : Rover_HAL.Time;
      DT            : Float;
      Gyro_Z        : Float;
      Raw_Gyro      : Rover_HAL.Gyro_Rate_Raw;
      Reset_Pending : Boolean;
      Sonar_Dist    : Unsigned_32;

      CR         : constant Float := Rover.Path_Following.Capture_Radius;
      Capture_R2 : constant Float := CR * CR;
   begin
      Current_Waypoint_Idx := 0;

      if Waypoint_Count = 0 then
         return;
      end if;

      loop
         --  Loop invariant: Current_Waypoint_Idx is always a valid waypoint
         --  index.  Established on entry by the `Waypoint_Count = 0 → return`
         --  guard above (so Waypoint_Count >= 1) and the `:= 0` initialisation.
         --  Preserved because every path that increments Current_Waypoint_Idx
         --  exits the loop when the new value reaches Waypoint_Count.
         pragma Loop_Invariant (Current_Waypoint_Idx < Waypoint_Count);

         --  Compute cycle time for gyro integration.
         Now := Rover_HAL.Clock;
         DT  := (if Last_Poll_Time = 0 then 0.05
                 else Float'Max (0.001, Float'Min (0.5,
                        Float (Now - Last_Poll_Time) /
                        Float (Rover_HAL.Ticks_Per_Second))));
         Last_Poll_Time := Now;

         --  Read gyro rate and convert to rad/s (see Poll).
         Raw_Gyro := Read_IMU_Gyro_Z;
         Gyro_Z   := Rover_HAL.To_Rad_S (Raw_Gyro);

         --  1. Latch encoder deltas (resets simulator counters to zero).
         --  See Poll for the clamp/saturating-add rationale.
         for W in Corner_Wheel_Id loop
            Ticks := Read_Encoder_Ticks (W);
            if Ticks > Rover.Estimation.Max_Encoder_Delta then
               Ticks := Rover.Estimation.Max_Encoder_Delta;
            elsif Ticks < -Rover.Estimation.Max_Encoder_Delta then
               Ticks := -Rover.Estimation.Max_Encoder_Delta;
            end if;
            Deltas (W) := Ticks;

            declare
               Sum : constant Integer_32 :=
                 Integer_32 (Encoder_Total (W)) + Integer_32 (Ticks);
            begin
               if Sum > Tick_Total_Max then
                  Encoder_Total (W) := Tick_Total_Max;
               elsif Sum < Tick_Total_Min then
                  Encoder_Total (W) := Tick_Total_Min;
               else
                  Encoder_Total (W) := Sum;
               end if;
            end;
         end loop;

         --  2. Sample GPS fix.
         Fix := GPS_Fix;

         --  2a. EKF reset (rover teleported in simulator): reinitialise and
         --  restart the path from waypoint 0 (latched, see Poll).
         Reset_Pending := EKF_Reset_Pending;
         if Reset_Pending then
            Reset_X  := EKF_Reset_X;
            Reset_Y  := EKF_Reset_Y;
            Reset_Th := EKF_Reset_Theta;
            Rover.Estimation.Init
              (Est_State, Est_P, Reset_X, Reset_Y, Reset_Th);
            Filter_Initialized   := True;
            Last_GPS_Timestamp   := Fix.Timestamp;
            Deltas               := [others => 0];
            Encoder_Total        := [others => 0];
            Last_GPS_X           := To_Scaled_Int32 (Reset_X);
            Last_GPS_Y           := To_Scaled_Int32 (Reset_Y);
            Current_Waypoint_Idx := 0;
            Predict_Count        := 0;
         end if;

         --  3. EKF cycle.
         if not Filter_Initialized then
            if Fix.Timestamp /= 0 then
               Rover.Estimation.Init (Est_State, Est_P, Fix.X, Fix.Y, 0.0);
               Last_GPS_Timestamp := Fix.Timestamp;
               Last_GPS_X         := To_Scaled_Int32 (Fix.X);
               Last_GPS_Y         := To_Scaled_Int32 (Fix.Y);
               Filter_Initialized := True;
               Predict_Count      := 0;
            end if;
         else
            --  EKF cycle with runtime-guarded preconditions (see Poll).
            if not All_P_In_Range (Est_P) then
               raise Rover.Estimation.Estimator_Assumption_Violation;
            end if;
            if not In_Position_Envelope (Est_State) then
               raise Rover.Estimation.Estimator_Assumption_Violation;
            end if;
            if not Predict_Diag_OK (Est_P) then
               raise Rover.Estimation.Estimator_Assumption_Violation;
            end if;

            --  Wrap Predict's exceptional exit in a handler that resets the
            --  EKF state to flow-safe defaults and re-raises (see Poll).
            begin
               Rover.Estimation.Predict
                 (Est_State, Est_P, Deltas, Steering, Gyro_Z, DT);
            exception
               when Rover.Estimation.Estimator_Assumption_Violation =>
                  Est_State :=
                    (X => 0.0, Y => 0.0, Theta => 0.0, S_Odometry => 1.0);
                  Est_P := [others => [others => 0.0]];
                  raise;
            end;

            --  Track fresh GPS fix for console display (decoupled from Update).
            if Fix.Timestamp /= Last_GPS_Timestamp then
               Last_GPS_Timestamp := Fix.Timestamp;
               Last_GPS_X         := To_Scaled_Int32 (Fix.X);
               Last_GPS_Y         := To_Scaled_Int32 (Fix.Y);
            end if;

            --  Fixed-schedule Update every N_Max_Predicts_Per_Update cycles.
            if Predict_Count = Rover.Estimation.N_Max_Predicts_Per_Update - 1
            then
               if not All_P_In_Range (Est_P) then
                  raise Rover.Estimation.Estimator_Assumption_Violation;
               end if;
               if not In_Position_Envelope (Est_State) then
                  raise Rover.Estimation.Estimator_Assumption_Violation;
               end if;
               if not Update_Diag_OK (Est_P) then
                  raise Rover.Estimation.Estimator_Assumption_Violation;
               end if;
               Rover.Estimation.Update (Est_State, Est_P, Fix);
               Predict_Count := 0;
            else
               Predict_Count := Predict_Count + 1;
            end if;

            if Est_State.X in World_X'Range
              and then Est_State.Y in World_Y'Range
            then
               Set_Estimated_Position
                 (World_X (Est_State.X),
                  World_Y (Est_State.Y),
                  Est_State.Theta);
            end if;

         end if;

         Report_GNC_State
           (Last_GPS_X, Last_GPS_Y,
            Encoder_Total (Front_Left),
            Encoder_Total (Front_Right),
            Encoder_Total (Rear_Left),
            Encoder_Total (Rear_Right));

         --  4. Guidance + Navigation (only once EKF is initialised).
         if Filter_Initialized then

            --  Runtime-guarded position envelope before the waypoint-distance
            --  computation.  After Predict/Update, Est_State.X/Y are bounded
            --  by Predict/Update posts, but SPARK cannot propagate those
            --  bounds across the surrounding control flow inductively at
            --  Silver.  A violation signals divergence of the EKF state from
            --  its assumptions and raises Estimator_Assumption_Violation,
            --  which Rover.Tasks catches.  Same carve-out style as Poll.
            if not In_Position_Envelope (Est_State) then
               raise Rover.Estimation.Estimator_Assumption_Violation;
            end if;

            --  Guidance: check whether the current waypoint has been captured.
            WP := Get_Waypoint (Current_Waypoint_Idx);
            Dx := Est_State.X - WP.X;
            Dy := Est_State.Y - WP.Y;
            if Dx * Dx + Dy * Dy < Capture_R2 then
               Current_Waypoint_Idx := Current_Waypoint_Idx + 1;
               if Current_Waypoint_Idx >= Waypoint_Count then
                  --  All waypoints reached: stop and return.
                  Set_Wheel_Angle (Front, Left,  0);
                  Set_Wheel_Angle (Front, Right, 0);
                  Set_Wheel_Angle (Back,  Left,  0);
                  Set_Wheel_Angle (Back,  Right, 0);
                  Set_Power (Left,  0);
                  Set_Power (Right, 0);
                  exit;
               end if;
               WP := Get_Waypoint (Current_Waypoint_Idx);
            end if;

            --  Navigation: pure computation, bearing to target, steering + power.
            Rover.Path_Following.Compute
              (Est_State, WP, Steering, Left_Power, Right_Power);

            --  Apply steering to each corner wheel.
            Set_Wheel_Angle (Front, Left,  To_HAL (Steering (Front_Left)));
            Set_Wheel_Angle (Front, Right, To_HAL (Steering (Front_Right)));
            Set_Wheel_Angle (Back,  Left,  To_HAL (Steering (Rear_Left)));
            Set_Wheel_Angle (Back,  Right, To_HAL (Steering (Rear_Right)));

            --  Sonar safety check (Cannot_Crash proof extension deferred).
            --  Latch volatile-with-side-effects read before the comparison.
            Sonar_Dist := Sonar_Distance;
            if Sonar_Dist < Safety_Distance then
               Left_Power  := 0;
               Right_Power := 0;
            end if;
            Set_Power (Left,  Left_Power);
            Set_Power (Right, Right_Power);

         else
            --  EKF not yet initialised: hold still.
            Set_Power (Left,  0);
            Set_Power (Right, 0);
         end if;

         Delay_Milliseconds (40);
      end loop;
   end Follow_Path;

end Rover.GNC;
