with Ada.Numerics;
with Ada.Numerics.Elementary_Functions; use Ada.Numerics.Elementary_Functions;

with Rover.Estimation.Common_Lemmas;  use Rover.Estimation.Common_Lemmas;
with Rover.Estimation.Predict_Lemmas; use Rover.Estimation.Predict_Lemmas;
with Rover.Estimation.Update_Lemmas;  use Rover.Estimation.Update_Lemmas;

package body Rover.Estimation
  with SPARK_Mode
is

   Pi : constant Float := Rover_HAL.Max_Heading;
   --  Package-level alias for π; using Max_Heading ties it to the Heading
   --  subtype so that Normalise_Angle's Post (result ∈ [-Pi, Pi]) is
   --  immediately seen by the prover as result ∈ Heading'Range.

   --  Shared bounded subtypes (Max_Wheel_Disp, Wheel_Disp, Scaled_Disp,
   --  Jac_F13_23, Jac_F14_24, Cov_Entry, FP_Entry, FPFt_Entry, FP_2MCov,
   --  FPFt_2FP, Unit_Float, Max_FP_12, Max_FPFt_12) live in the private
   --  part of the parent spec rover-estimation.ads and are visible here
   --  without qualification.

   --  R_GPS is a spec constant (rover-estimation.ads) so postconditions
   --  and GNC loop invariants can reference it.

   --  Normalise an angle to (−π, π].
   --  A single add/subtract is sufficient when the input is within (−3π, 3π),
   --  which is guaranteed by the bounded per-cycle angular rate.
   function Normalise_Angle (A : Float) return Float
     with Global => null,
          Pre    => A in -3.0 * Pi .. 3.0 * Pi,
          Post   => Normalise_Angle'Result in -Pi .. Pi
   is
   begin
      if A > Pi then
         return A - 2.0 * Pi;
      elsif A <= -Pi then
         return A + 2.0 * Pi;
      else
         return A;
      end if;
   end Normalise_Angle;

   --  Body-frame position of a corner wheel at the given steering angle.
   --  Mirrors local_wheel_position() in simulator/src/domain/robot.rs.
   --
   --  The wheel centre is offset from the pivot by Pivot_Offset along the
   --  local Y axis (outward from rover centre), rotated by the steering angle.
   procedure Wheel_Pos
     (Wheel : Rover_HAL.Corner_Wheel_Id;
      Alpha : Float;
      Px    : out Float;
      Py    : out Float)
   with Global => null
   is
      Sin_A : constant Float := Sin (Alpha);
      Cos_A : constant Float := Cos (Alpha);
   begin
      case Wheel is
         when Rover_HAL.Front_Left =>
            Px := Dist_Front - Pivot_Offset * Sin_A;
            Py := Dist_Side  + Pivot_Offset * Cos_A;
         when Rover_HAL.Front_Right =>
            Px := Dist_Front + Pivot_Offset * Sin_A;
            Py := -Dist_Side - Pivot_Offset * Cos_A;
         when Rover_HAL.Rear_Left =>
            Px := -Dist_Rear - Pivot_Offset * Sin_A;
            Py := Dist_Side  + Pivot_Offset * Cos_A;
         when Rover_HAL.Rear_Right =>
            Px := -Dist_Rear + Pivot_Offset * Sin_A;
            Py := -Dist_Side - Pivot_Offset * Cos_A;
      end case;
   end Wheel_Pos;

   ----------
   -- Init --
   ----------

   procedure Init
     (S       : out State;
      P       : out Covariance;
      X_0     : Rover_HAL.World_X;
      Y_0     : Rover_HAL.World_Y;
      Theta_0 : Rover_HAL.Heading;
      S_0     : Odometry_Scale := 1.0)
   is
   begin
      S := (X => X_0, Y => Y_0, Theta => Theta_0, S_Odometry => S_0);
      P := [others => [others => 0.0]];
      P (1, 1) := 1.0;
      P (2, 2) := 1.0;
      P (3, 3) := 1.0;
      P (4, 4) := 0.1;  --  ±0.3 initial 1-sigma uncertainty on scale
   end Init;

   -------------
   -- Predict --
   -------------

   procedure Predict
     (S        : in out State;
      P        : in out Covariance;
      Encoders : Encoder_Deltas;
      Steering : Corner_Steering;
      Gyro_Z   : Rover_HAL.Gyro_Rate_Rad_S;
      DT       : Cycle_Time)
   is
      use Rover_HAL;

      --  All shared bounded subtypes (Wheel_Disp, Scaled_Disp, Jac_F13_23,
      --  Jac_F14_24, Cov_Entry, FP_Entry, FPFt_Entry, FP_2MCov, FPFt_2FP,
      --  Unit_Float) and their bounding constants (Max_Wheel_Disp,
      --  Max_FP_12, Max_FPFt_12) are declared in the private part of
      --  Rover.Estimation's spec and are visible here without qualification.
      --  Ghost lemmas (Lemma_Sin_Bounded, Lemma_Cos_Bounded, Lemma_WD_Trig,
      --  Lemma_SD_Trig, Lemma_WD_Scale, Lemma_Unit_Cov, Lemma_Unit_FP,
      --  Lemma_Cov_Plus_Cov, Lemma_FP_2MCov_Plus_Cov, Lemma_FP_Plus_FP,
      --  Lemma_FPFt_2FP_Plus_FP, Lemma_Position_Step,
      --  Lemma_Position_Step_Add, Lemma_Step_Bound) live in the private
      --  child package Rover.Estimation.Predict_Lemmas (with-used above).

      --  Heading change from the gyro.
      D_Theta : constant Float := Gyro_Z * DT;

      --  Heading trig — computed once from S.Theta at entry, reused across
      --  Steps 2, 3, 4.
      Sin_T : Float;
      Cos_T : Float;

      --  Body-frame displacement per control cycle.  Filled first (pre-scale)
      --  by Accumulate_Body_Disp as Wheel_Disp; then scaled to Scaled_Disp by
      --  Compute_F14_F24_And_Scale.
      Vx_Body_WD : Wheel_Disp;
      Vy_Body_WD : Wheel_Disp;
      Vx_Body_SC : Scaled_Disp;
      Vy_Body_SC : Scaled_Disp;

      --  Jacobian off-diagonal elements.
      F13, F23 : Jac_F13_23;
      F14, F24 : Jac_F14_24;

      --  FP and FPFt use bounded element types so the prover gets bounds from
      --  the type.  Declared in Predict's scope so the nested step subprograms
      --  can accept/produce them via typed parameters.
      type FP_Matrix   is array (1 .. 4, 1 .. 4) of FP_Entry;
      type FPFt_Matrix is array (1 .. 4, 1 .. 4) of FPFt_Entry;
      FP   : FP_Matrix;
      FPFt : FPFt_Matrix;

      ------------------------------------------------------------------
      --  Nested step subprograms: each implements one Predict stage
      --  with a narrow, typed contract.  Proved together with Predict
      --  (they are nested, so they have no external callers), but each
      --  gets its own VC context, keeping the solver out of the full
      --  Predict state space on every check.
      ------------------------------------------------------------------

      procedure Compute_Heading_Trig
        (Theta   : Rover_HAL.Heading;
         Sin_Out : out Float;
         Cos_Out : out Float)
      with Global => null,
           Post   => Sin_Out in -1.0 .. 1.0
                     and then Cos_Out in -1.0 .. 1.0;
      --  Step 0: sin/cos of the current heading, bounded via lemmas.

      procedure Accumulate_Body_Disp
        (Enc   : Encoder_Deltas;
         Steer : Corner_Steering;
         Vx    : out Wheel_Disp;
         Vy    : out Wheel_Disp)
      with Global => null;
      --  Step 1: accumulate per-wheel body-frame displacements.

      procedure Compute_F14_F24_And_Scale
        (Vx_In    : Wheel_Disp;
         Vy_In    : Wheel_Disp;
         Sin_T_In : Float;
         Cos_T_In : Float;
         Sc       : Odometry_Scale;
         F14_Out  : out Jac_F14_24;
         F24_Out  : out Jac_F14_24;
         Vx_Out   : out Scaled_Disp;
         Vy_Out   : out Scaled_Disp)
      with Global => null,
           Pre    => Sin_T_In in -1.0 .. 1.0
                     and then Cos_T_In in -1.0 .. 1.0;
      --  Step 2: Jacobian 4th column from unscaled displacements,
      --  then scale displacements by the odometry estimate.

      procedure Advance_Position
        (S_InOut    : in out State;
         Vx_SC      : Scaled_Disp;
         Vy_SC      : Scaled_Disp;
         Sin_T_In   : Float;
         Cos_T_In   : Float;
         D_Theta_In : Float)
      with Global => null,
           Pre    => Sin_T_In in -1.0 .. 1.0
                     and then Cos_T_In in -1.0 .. 1.0
                     and then S_InOut.X in -1_000.0 .. 1_000.0
                     and then S_InOut.Y in -1_000.0 .. 1_000.0
                     and then D_Theta_In in -2.0 .. 2.0,
           Post   => S_InOut.S_Odometry = S_InOut.S_Odometry'Old
                     and then S_InOut.Theta in Rover_HAL.Heading
                     and then S_InOut.X in S_InOut.X'Old - Max_Position_Step ..
                                            S_InOut.X'Old + Max_Position_Step
                     and then S_InOut.Y in S_InOut.Y'Old - Max_Position_Step ..
                                            S_InOut.Y'Old + Max_Position_Step;
      --  Step 3: advance world-frame position and heading.

      procedure Compute_F13_F23
        (Vx_SC    : Scaled_Disp;
         Vy_SC    : Scaled_Disp;
         Sin_T_In : Float;
         Cos_T_In : Float;
         F13_Out  : out Jac_F13_23;
         F23_Out  : out Jac_F13_23)
      with Global => null,
           Pre    => Sin_T_In in -1.0 .. 1.0
                     and then Cos_T_In in -1.0 .. 1.0;
      --  Step 4: Jacobian 3rd column from scaled displacements.

      procedure Compute_FP
        (P_In    : Covariance;
         F13_In  : Jac_F13_23;
         F23_In  : Jac_F13_23;
         F14_In  : Jac_F14_24;
         F24_In  : Jac_F14_24;
         FP_Out  : out FP_Matrix)
      with Global => null,
           Pre    => (for all I in 1 .. 4 =>
                        (for all J in 1 .. 4 =>
                           P_In (I, J) in
                             -Max_Covariance_Diag .. Max_Covariance_Diag)),
           Post   => FP_Out (3, 3) = P_In (3, 3)
                     and then FP_Out (4, 4) = P_In (4, 4);
      --  FP = F * P.  Rows 3, 4 of F are identity, so FP rows 3, 4 equal
      --  P rows 3, 4.  The FP_Entry subtype on FP_Matrix elements encodes
      --  the |FP(I,J)| <= Max_FP_12 bound.

      procedure Compute_FPFt
        (FP_In     : FP_Matrix;
         F13_In    : Jac_F13_23;
         F23_In    : Jac_F13_23;
         F14_In    : Jac_F14_24;
         F24_In    : Jac_F14_24;
         FPFt_Out  : out FPFt_Matrix)
      with Global => null,
           Post   => FPFt_Out (3, 3) = FP_In (3, 3)
                     and then FPFt_Out (4, 4) = FP_In (4, 4);
      --  FPFt = FP * Fᵀ.  Columns 3, 4 of Fᵀ are identity (rows 3, 4 of F),
      --  so FPFt cols 3, 4 equal FP cols 3, 4.  The FPFt_Entry subtype on
      --  FPFt_Matrix elements encodes the |FPFt(I,J)| <= Max_FPFt_12 bound.

      procedure Assemble_P_From_FPFt_And_Q
        (FPFt_In : FPFt_Matrix;
         P_Out   : out Covariance)
      with Global => null,
           Post   => P_Out (1, 1) = FPFt_In (1, 1) + Q_X
                     and then P_Out (2, 2) = FPFt_In (2, 2) + Q_Y
                     and then P_Out (3, 3) = FPFt_In (3, 3) + Q_Theta
                     and then P_Out (4, 4) = FPFt_In (4, 4) + Q_S
                     --  Off-diagonal: P = FPFt, expressed universally so the
                     --  prover retains pattern-based reasoning for callers.
                     and then (for all I in 1 .. 4 =>
                                 (for all J in 1 .. 4 =>
                                    (if I /= J then
                                       P_Out (I, J) = FPFt_In (I, J))))
                     and then (for all I in 1 .. 4 =>
                                 (for all J in 1 .. 4 =>
                                    P_Out (I, J) in
                                      -Max_P_After_Predict ..
                                       Max_P_After_Predict));
      --  P := FPFt + Q (Q diagonal).  Single aggregate assignment so the
      --  quantified range postcondition reduces to sixteen direct
      --  instantiations.


      ------------------------------------------------------------------
      -- Compute_Heading_Trig
      ------------------------------------------------------------------

      procedure Compute_Heading_Trig
        (Theta   : Rover_HAL.Heading;
         Sin_Out : out Float;
         Cos_Out : out Float)
      is
      begin
         Sin_Out := Sin (Theta);
         Lemma_Sin_Bounded (Theta);
         Cos_Out := Cos (Theta);
         Lemma_Cos_Bounded (Theta);
      end Compute_Heading_Trig;

      ------------------------------------------------------------------
      -- Accumulate_Body_Disp
      ------------------------------------------------------------------

      procedure Accumulate_Body_Disp
        (Enc   : Encoder_Deltas;
         Steer : Corner_Steering;
         Vx    : out Wheel_Disp;
         Vy    : out Wheel_Disp)
      is
         D_W    : Wheel_Disp;
         Sin_A  : Float;
         Cos_A  : Float;
         Vx_Acc : Float := 0.0;
         Vy_Acc : Float := 0.0;
      begin
         for W in Corner_Wheel_Id loop
            D_W   := Wheel_Disp (Float (Enc (W)) * Metres_Per_Tick);
            Sin_A := Sin (Steer (W));
            Lemma_Sin_Bounded (Steer (W));
            Cos_A := Cos (Steer (W));
            Lemma_Cos_Bounded (Steer (W));
            Lemma_WD_Trig (D_W, Cos_A);
            Lemma_WD_Trig (D_W, Sin_A);
            Vx_Acc := Vx_Acc + D_W * Cos_A / 4.0;
            Vy_Acc := Vy_Acc + D_W * Sin_A / 4.0;
            pragma Loop_Invariant
              (Vy_Acc in
                 -(Float (Corner_Wheel_Id'Pos (W)) + 1.0) * Max_Wheel_Disp / 4.0 ..
                  (Float (Corner_Wheel_Id'Pos (W)) + 1.0) * Max_Wheel_Disp / 4.0);
            pragma Loop_Invariant
              (Vx_Acc in
                 -(Float (Corner_Wheel_Id'Pos (W)) + 1.0) * Max_Wheel_Disp / 4.0 ..
                  (Float (Corner_Wheel_Id'Pos (W)) + 1.0) * Max_Wheel_Disp / 4.0);
            pragma Loop_Invariant (Sin_A in -1.0 .. 1.0);
            pragma Loop_Invariant (Cos_A in -1.0 .. 1.0);
         end loop;
         pragma Assert (Vx_Acc in Wheel_Disp);
         pragma Assert (Vy_Acc in Wheel_Disp);
         Vx := Wheel_Disp (Vx_Acc);
         Vy := Wheel_Disp (Vy_Acc);
      end Accumulate_Body_Disp;

      ------------------------------------------------------------------
      -- Compute_F14_F24_And_Scale
      ------------------------------------------------------------------

      procedure Compute_F14_F24_And_Scale
        (Vx_In    : Wheel_Disp;
         Vy_In    : Wheel_Disp;
         Sin_T_In : Float;
         Cos_T_In : Float;
         Sc       : Odometry_Scale;
         F14_Out  : out Jac_F14_24;
         F24_Out  : out Jac_F14_24;
         Vx_Out   : out Scaled_Disp;
         Vy_Out   : out Scaled_Disp)
      is
      begin
         --  Step 2a: Jacobian 4th column (∂x'/∂s, ∂y'/∂s) from unscaled disp.
         Lemma_WD_Trig (Vx_In, Cos_T_In);
         Lemma_WD_Trig (Vy_In, Sin_T_In);
         Lemma_WD_Trig (Vx_In, Sin_T_In);
         Lemma_WD_Trig (Vy_In, Cos_T_In);
         F14_Out := Jac_F14_24 (Vx_In * Cos_T_In - Vy_In * Sin_T_In);
         F24_Out := Jac_F14_24 (Vx_In * Sin_T_In + Vy_In * Cos_T_In);

         --  Step 2b: scale unscaled displacements by the odometry estimate.
         Lemma_WD_Scale (Vx_In, Sc);
         Lemma_WD_Scale (Vy_In, Sc);
         Vx_Out := Scaled_Disp (Vx_In * Sc);
         Vy_Out := Scaled_Disp (Vy_In * Sc);
      end Compute_F14_F24_And_Scale;

      ------------------------------------------------------------------
      -- Advance_Position
      ------------------------------------------------------------------

      procedure Advance_Position
        (S_InOut    : in out State;
         Vx_SC      : Scaled_Disp;
         Vy_SC      : Scaled_Disp;
         Sin_T_In   : Float;
         Cos_T_In   : Float;
         D_Theta_In : Float)
      is
      begin
         pragma Assert (Max_Position_Step in 0.0 .. 1.0);
         declare
            Delta_X : constant Float := Vx_SC * Cos_T_In - Vy_SC * Sin_T_In;
         begin
            Lemma_SD_Trig (Vx_SC, Cos_T_In);
            Lemma_SD_Trig (Vy_SC, Sin_T_In);
            Lemma_Position_Step (Vx_SC, Cos_T_In, Vy_SC, Sin_T_In);
            pragma Assert (Delta_X in -Max_Position_Step .. Max_Position_Step);
            Lemma_Step_Bound (S_InOut.X, Delta_X, Max_Position_Step);
            S_InOut.X := S_InOut.X + Delta_X;
         end;
         declare
            Delta_Y : constant Float := Vx_SC * Sin_T_In + Vy_SC * Cos_T_In;
         begin
            Lemma_SD_Trig (Vx_SC, Sin_T_In);
            Lemma_SD_Trig (Vy_SC, Cos_T_In);
            Lemma_Position_Step_Add (Vx_SC, Sin_T_In, Vy_SC, Cos_T_In);
            pragma Assert (Delta_Y in -Max_Position_Step .. Max_Position_Step);
            Lemma_Step_Bound (S_InOut.Y, Delta_Y, Max_Position_Step);
            S_InOut.Y := S_InOut.Y + Delta_Y;
         end;
         pragma Assert (Float (S_InOut.Theta) in -Pi .. Pi);
         pragma Assert (Float (S_InOut.Theta) + D_Theta_In
                          in -3.0 * Pi .. 3.0 * Pi);
         S_InOut.Theta :=
           Rover_HAL.Heading
             (Normalise_Angle (Float (S_InOut.Theta) + D_Theta_In));
      end Advance_Position;

      ------------------------------------------------------------------
      -- Compute_F13_F23
      ------------------------------------------------------------------

      procedure Compute_F13_F23
        (Vx_SC    : Scaled_Disp;
         Vy_SC    : Scaled_Disp;
         Sin_T_In : Float;
         Cos_T_In : Float;
         F13_Out  : out Jac_F13_23;
         F23_Out  : out Jac_F13_23)
      is
      begin
         Lemma_SD_Trig (Vx_SC, Sin_T_In);
         Lemma_SD_Trig (Vy_SC, Cos_T_In);
         Lemma_SD_Trig (Vx_SC, Cos_T_In);
         Lemma_SD_Trig (Vy_SC, Sin_T_In);
         F13_Out := Jac_F13_23 (-Vx_SC * Sin_T_In - Vy_SC * Cos_T_In);
         F23_Out := Jac_F13_23 (Vx_SC * Cos_T_In - Vy_SC * Sin_T_In);
      end Compute_F13_F23;

      ------------------------------------------------------------------
      -- Compute_FP
      ------------------------------------------------------------------

      procedure Compute_FP
        (P_In    : Covariance;
         F13_In  : Jac_F13_23;
         F23_In  : Jac_F13_23;
         F14_In  : Jac_F14_24;
         F24_In  : Jac_F14_24;
         FP_Out  : out FP_Matrix)
      is
      begin
         FP_Out := [others => [others => 0.0]];
         for J in 1 .. 4 loop
            declare
               P3 : constant Cov_Entry := Cov_Entry (P_In (3, J));
               P4 : constant Cov_Entry := Cov_Entry (P_In (4, J));
            begin
               Lemma_Unit_Cov (Unit_Float (F13_In), P3);
               Lemma_Unit_Cov (Unit_Float (F14_In), P4);
               Lemma_Unit_Cov (Unit_Float (F23_In), P3);
               Lemma_Unit_Cov (Unit_Float (F24_In), P4);
               declare
                  P1J      : constant Cov_Entry := Cov_Entry (P_In (1, J));
                  Prod_13P : constant Cov_Entry :=
                    Cov_Entry (Unit_Float (F13_In) * P3);
                  Prod_14P : constant Cov_Entry :=
                    Cov_Entry (Unit_Float (F14_In) * P4);
                  P2J      : constant Cov_Entry := Cov_Entry (P_In (2, J));
                  Prod_23P : constant Cov_Entry :=
                    Cov_Entry (Unit_Float (F23_In) * P3);
                  Prod_24P : constant Cov_Entry :=
                    Cov_Entry (Unit_Float (F24_In) * P4);
               begin
                  Lemma_Cov_Plus_Cov (P1J, Prod_13P);
                  declare
                     Part_R1 : constant FP_2MCov :=
                       FP_2MCov (P1J + Prod_13P);
                  begin
                     Lemma_FP_2MCov_Plus_Cov (Part_R1, Prod_14P);
                     FP_Out (1, J) := FP_Entry (Part_R1 + Prod_14P);
                  end;
                  Lemma_Cov_Plus_Cov (P2J, Prod_23P);
                  declare
                     Part_R2 : constant FP_2MCov :=
                       FP_2MCov (P2J + Prod_23P);
                  begin
                     Lemma_FP_2MCov_Plus_Cov (Part_R2, Prod_24P);
                     FP_Out (2, J) := FP_Entry (Part_R2 + Prod_24P);
                  end;
               end;
               FP_Out (3, J) := FP_Entry (P_In (3, J));
               FP_Out (4, J) := FP_Entry (P_In (4, J));
            end;
            --  After iteration J, columns 1..J have been written; in
            --  particular FP_Out(3,3) and FP_Out(4,4) reflect P_In once
            --  J reaches 3 and 4 respectively.
            pragma Loop_Invariant
              (FP_Out (3, 3) = (if J >= 3 then P_In (3, 3)
                                else FP_Out (3, 3)'Loop_Entry));
            pragma Loop_Invariant
              (FP_Out (4, 4) = (if J >= 4 then P_In (4, 4)
                                else FP_Out (4, 4)'Loop_Entry));
         end loop;
      end Compute_FP;

      ------------------------------------------------------------------
      -- Compute_FPFt
      ------------------------------------------------------------------

      procedure Compute_FPFt
        (FP_In     : FP_Matrix;
         F13_In    : Jac_F13_23;
         F23_In    : Jac_F13_23;
         F14_In    : Jac_F14_24;
         F24_In    : Jac_F14_24;
         FPFt_Out  : out FPFt_Matrix)
      is
      begin
         FPFt_Out := [others => [others => 0.0]];
         for I in 1 .. 4 loop
            declare
               FP3   : constant FP_Entry := FP_In (I, 3);
               FP4   : constant FP_Entry := FP_In (I, 4);
               FP_I1 : constant FP_Entry := FP_In (I, 1);
               FP_I2 : constant FP_Entry := FP_In (I, 2);
            begin
               Lemma_Unit_FP (Unit_Float (F13_In), FP3);
               Lemma_Unit_FP (Unit_Float (F14_In), FP4);
               Lemma_Unit_FP (Unit_Float (F23_In), FP3);
               Lemma_Unit_FP (Unit_Float (F24_In), FP4);
               declare
                  Prod_13F : constant FP_Entry :=
                    FP_Entry (Unit_Float (F13_In) * FP3);
                  Prod_14F : constant FP_Entry :=
                    FP_Entry (Unit_Float (F14_In) * FP4);
                  Prod_23F : constant FP_Entry :=
                    FP_Entry (Unit_Float (F23_In) * FP3);
                  Prod_24F : constant FP_Entry :=
                    FP_Entry (Unit_Float (F24_In) * FP4);
               begin
                  Lemma_FP_Plus_FP (FP_I1, Prod_13F);
                  declare
                     Part_C1 : constant FPFt_2FP :=
                       FPFt_2FP (FP_I1 + Prod_13F);
                  begin
                     Lemma_FPFt_2FP_Plus_FP (Part_C1, Prod_14F);
                     FPFt_Out (I, 1) := FPFt_Entry (Part_C1 + Prod_14F);
                  end;
                  Lemma_FP_Plus_FP (FP_I2, Prod_23F);
                  declare
                     Part_C2 : constant FPFt_2FP :=
                       FPFt_2FP (FP_I2 + Prod_23F);
                  begin
                     Lemma_FPFt_2FP_Plus_FP (Part_C2, Prod_24F);
                     FPFt_Out (I, 2) := FPFt_Entry (Part_C2 + Prod_24F);
                  end;
               end;
               FPFt_Out (I, 3) := FPFt_Entry (FP_In (I, 3));
               FPFt_Out (I, 4) := FPFt_Entry (FP_In (I, 4));
            end;
            pragma Loop_Invariant
              (FPFt_Out (3, 3) = (if I >= 3 then FP_In (3, 3)
                                  else FPFt_Out (3, 3)'Loop_Entry));
            pragma Loop_Invariant
              (FPFt_Out (4, 4) = (if I >= 4 then FP_In (4, 4)
                                  else FPFt_Out (4, 4)'Loop_Entry));
         end loop;
      end Compute_FPFt;

      ------------------------------------------------------------------
      -- Assemble_P_From_FPFt_And_Q
      ------------------------------------------------------------------

      procedure Assemble_P_From_FPFt_And_Q
        (FPFt_In : FPFt_Matrix;
         P_Out   : out Covariance)
      is
      begin
         --  Single aggregate assignment: all sixteen entries written at once.
         --  This avoids the sequential-write quantifier-instantiation problem
         --  that arises when sixteen individual `P(I,J) := ...` assignments
         --  each leave the prover with a separate per-entry fact the universal
         --  quantifier cannot re-assemble.
         P_Out :=
           (1 => (1 => Float (FPFt_In (1, 1)) + Q_X,
                  2 => Float (FPFt_In (1, 2)),
                  3 => Float (FPFt_In (1, 3)),
                  4 => Float (FPFt_In (1, 4))),
            2 => (1 => Float (FPFt_In (2, 1)),
                  2 => Float (FPFt_In (2, 2)) + Q_Y,
                  3 => Float (FPFt_In (2, 3)),
                  4 => Float (FPFt_In (2, 4))),
            3 => (1 => Float (FPFt_In (3, 1)),
                  2 => Float (FPFt_In (3, 2)),
                  3 => Float (FPFt_In (3, 3)) + Q_Theta,
                  4 => Float (FPFt_In (3, 4))),
            4 => (1 => Float (FPFt_In (4, 1)),
                  2 => Float (FPFt_In (4, 2)),
                  3 => Float (FPFt_In (4, 3)),
                  4 => Float (FPFt_In (4, 4)) + Q_S));
      end Assemble_P_From_FPFt_And_Q;

   begin
      Compute_Heading_Trig (S.Theta, Sin_T, Cos_T);

      Accumulate_Body_Disp (Encoders, Steering, Vx_Body_WD, Vy_Body_WD);

      Compute_F14_F24_And_Scale
        (Vx_In    => Vx_Body_WD,
         Vy_In    => Vy_Body_WD,
         Sin_T_In => Sin_T,
         Cos_T_In => Cos_T,
         Sc       => S.S_Odometry,
         F14_Out  => F14,
         F24_Out  => F24,
         Vx_Out   => Vx_Body_SC,
         Vy_Out   => Vy_Body_SC);

      Advance_Position
        (S_InOut    => S,
         Vx_SC      => Vx_Body_SC,
         Vy_SC      => Vy_Body_SC,
         Sin_T_In   => Sin_T,
         Cos_T_In   => Cos_T,
         D_Theta_In => D_Theta);

      Compute_F13_F23
        (Vx_SC    => Vx_Body_SC,
         Vy_SC    => Vy_Body_SC,
         Sin_T_In => Sin_T,
         Cos_T_In => Cos_T,
         F13_Out  => F13,
         F23_Out  => F23);

      Compute_FP (P, F13, F23, F14, F24, FP);

      Compute_FPFt (FP, F13, F23, F14, F24, FPFt);

      Assemble_P_From_FPFt_And_Q
        (FPFt_In => FPFt,
         P_Out   => P);

      --  Runtime enforcement of PSD-dependent diagonal non-negativity.
      --  P(1,1) ∈ [0, Max_P_After_Predict] and P(2,2) ∈ [0, Max_P_After_Predict]
      --  are PSD-dependent; SPARK cannot prove them at Silver without
      --  Is_PSD infrastructure.  The Post of Predict has been weakened to
      --  omit these conjuncts; they are enforced at runtime instead by
      --  raising Estimator_Assumption_Violation (documented in the spec's
      --  Exceptional_Cases).
      if P (1, 1) not in 0.0 .. Max_P_After_Predict then
         raise Estimator_Assumption_Violation;
      end if;
      if P (2, 2) not in 0.0 .. Max_P_After_Predict then
         raise Estimator_Assumption_Violation;
      end if;

   end Predict;

   ------------
   -- Update --
   ------------

   procedure Update
     (S   : in out State;
      P   : in out Covariance;
      Fix : Rover_HAL.GPS_Fix_Type)
   is
      --  Scalar innovation-covariance denominators: P(i,i) + R > 0.
      --  P(1,1) ≥ 0 and P(2,2) ≥ 0 from the precondition; R_GPS > 0.
      Denom_X : constant Float := P (1, 1) + R_GPS;
      Denom_Y : constant Float := P (2, 2) + R_GPS;

      --  Scalar Kalman gains K_x, K_y ∈ [0, 1).
      --  K = P(i,i) / (P(i,i) + R) < 1 since R > 0.
      K_X : constant Float := P (1, 1) / Denom_X;
      K_Y : constant Float := P (2, 2) / Denom_Y;

      --  Update's ghost lemmas (Lemma_Gain_Contract, Lemma_Gain_Monotone,
      --  Lemma_S_Safe_Bound) live in Rover.Estimation.Update_Lemmas, and
      --  the shared Lemma_Scale_Cov and Lemma_Product_Unit_Interval live
      --  in Rover.Estimation.Common_Lemmas (both with-used above).  The
      --  K_Min ghost constant is declared in Update_Lemmas.

      ------------------------------------------------------------------
      --  Joseph_Scale_P — nested procedure implementing the symmetric
      --  Joseph-form covariance scaling step.  Moving the 50-odd-line
      --  declare block into this nested sub with a narrow Pre/Post
      --  gives the solver a small, local context for every scaling
      --  step.  Unit_Float-typed gain inputs carry the [−1, 1] bound
      --  from the type; the extra 0..1 Pre narrows it to non-negative.
      ------------------------------------------------------------------

      procedure Joseph_Scale_P
        (P_IO   : in out Covariance;
         K_X_In : Unit_Float;
         K_Y_In : Unit_Float)
      with Global => null,
           Pre    =>
             K_X_In in 0.0 .. 1.0
             and then K_Y_In in 0.0 .. 1.0
             and then (for all I in 1 .. 4 =>
                         (for all J in 1 .. 4 =>
                            P_IO (I, J) in
                              -Max_Covariance_Diag .. Max_Covariance_Diag))
             and then P_IO (1, 1) in 0.0 .. Max_Covariance_Diag
             and then P_IO (2, 2) in 0.0 .. Max_Covariance_Diag,
           Post   =>
             P_IO (1, 1) in 0.0 .. P_IO'Old (1, 1)
             and then P_IO (2, 2) in 0.0 .. P_IO'Old (2, 2)
             and then (for all I in 3 .. 4 =>
                         (for all J in 3 .. 4 =>
                            P_IO (I, J) = P_IO'Old (I, J)))
             and then (for all I in 1 .. 4 =>
                         (for all J in 1 .. 4 =>
                            P_IO (I, J) in
                              -Max_Covariance_Diag .. Max_Covariance_Diag));

      procedure Joseph_Scale_P
        (P_IO   : in out Covariance;
         K_X_In : Unit_Float;
         K_Y_In : Unit_Float)
      is
         --  Ghost captures of P diagonals at nested-sub entry — needed by
         --  Lemma_Gain_Contract whose Pre expects a ghost variable, and by
         --  intermediate pragma Asserts (pragma Assert cannot use 'Old).
         P11_Entry : constant Float := P_IO (1, 1) with Ghost;
         P22_Entry : constant Float := P_IO (2, 2) with Ghost;

         Sx : constant Float := 1.0 - K_X_In;
         Sy : constant Float := 1.0 - K_Y_In;
      begin
         --  Pre-anchor asserts: ground the universal Pre for the specific
         --  entries READ later inside this nested sub.  At whole-program
         --  level=2 the solver budget is split across many VCs and the
         --  universal Pre alone is not reliably instantiated for the
         --  scattered index pairs below.  Making these facts ground and
         --  immediate at entry keeps them cheap to reuse.
         pragma Assert
           (P_IO (1, 2) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert
           (P_IO (1, 3) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert
           (P_IO (1, 4) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert
           (P_IO (2, 3) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert
           (P_IO (2, 4) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert
           (P_IO (3, 4) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert
           (P_IO (4, 3) in -Max_Covariance_Diag .. Max_Covariance_Diag);

         pragma Assert (Sx in 0.0 .. 1.0);
         pragma Assert (Sy in 0.0 .. 1.0);

         --  Diagonal (1,1): scale by Sx.
         Lemma_Gain_Contract (K_X_In, P11_Entry);
         pragma Assert ((1.0 - K_X_In) * P11_Entry in 0.0 .. P11_Entry);
         P_IO (1, 1) := Sx * P_IO (1, 1);
         pragma Assert (P_IO (1, 1) in -Max_Covariance_Diag .. Max_Covariance_Diag);

         --  Diagonal (2,2): scale by Sy.
         Lemma_Gain_Contract (K_Y_In, P22_Entry);
         pragma Assert ((1.0 - K_Y_In) * P22_Entry in 0.0 .. P22_Entry);
         P_IO (2, 2) := Sy * P_IO (2, 2);
         pragma Assert (P_IO (2, 2) in -Max_Covariance_Diag .. Max_Covariance_Diag);

         --  Diagonals (3,3) and (4,4): unchanged (scale = 1).
         pragma Assert (P_IO (3, 3) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert (P_IO (4, 4) in -Max_Covariance_Diag .. Max_Covariance_Diag);

         --  P(1,2) and P(2,1): scale by Sx · Sy (product of both factors).
         Lemma_Product_Unit_Interval (Sx, Sy);
         declare
            Sxy : constant Float := Sx * Sy;
         begin
            pragma Assert (Sxy in 0.0 .. 1.0);
            Lemma_Scale_Cov (Sxy, P_IO (1, 2));
            P_IO (1, 2) := Sxy * P_IO (1, 2);
            P_IO (2, 1) := P_IO (1, 2);
         end;
         pragma Assert (P_IO (1, 2) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert (P_IO (2, 1) in -Max_Covariance_Diag .. Max_Covariance_Diag);

         --  P(1,3) and P(3,1): scale by Sx (scale(1)·scale(3) = Sx·1).
         Lemma_Scale_Cov (Sx, P_IO (1, 3));
         P_IO (1, 3) := Sx * P_IO (1, 3);
         P_IO (3, 1) := P_IO (1, 3);
         pragma Assert (P_IO (1, 3) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert (P_IO (3, 1) in -Max_Covariance_Diag .. Max_Covariance_Diag);

         --  P(1,4) and P(4,1): scale by Sx.
         Lemma_Scale_Cov (Sx, P_IO (1, 4));
         P_IO (1, 4) := Sx * P_IO (1, 4);
         P_IO (4, 1) := P_IO (1, 4);
         pragma Assert (P_IO (1, 4) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert (P_IO (4, 1) in -Max_Covariance_Diag .. Max_Covariance_Diag);

         --  P(2,3) and P(3,2): scale by Sy (scale(2)·scale(3) = Sy·1).
         Lemma_Scale_Cov (Sy, P_IO (2, 3));
         P_IO (2, 3) := Sy * P_IO (2, 3);
         P_IO (3, 2) := P_IO (2, 3);
         pragma Assert (P_IO (2, 3) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert (P_IO (3, 2) in -Max_Covariance_Diag .. Max_Covariance_Diag);

         --  P(2,4) and P(4,2): scale by Sy.
         Lemma_Scale_Cov (Sy, P_IO (2, 4));
         P_IO (2, 4) := Sy * P_IO (2, 4);
         P_IO (4, 2) := P_IO (2, 4);
         pragma Assert (P_IO (2, 4) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert (P_IO (4, 2) in -Max_Covariance_Diag .. Max_Covariance_Diag);

         --  P(3,4) and P(4,3): unchanged (scale(3)·scale(4) = 1·1 = 1).
         pragma Assert (P_IO (3, 4) in -Max_Covariance_Diag .. Max_Covariance_Diag);
         pragma Assert (P_IO (4, 3) in -Max_Covariance_Diag .. Max_Covariance_Diag);
      end Joseph_Scale_P;

   begin
      --  K_X and K_Y are Kalman gains: P(i,i) ≥ 0 (pre) and R_GPS > 0.
      pragma Assert (K_X in 0.0 .. 1.0);
      pragma Assert (K_Y in 0.0 .. 1.0);

      --  Establish gain monotonicity: K_X ≥ K_min, K_Y ≥ K_min.
      Lemma_Gain_Monotone (P (1, 1));
      pragma Assert (K_X >= K_Min);
      Lemma_Gain_Monotone (P (2, 2));
      pragma Assert (K_Y >= K_Min);

      --  State update: convex combination contracts position toward Fix, which
      --  lies in World_X/World_Y (≤ ±Max_World_X/Y ≈ ±6 m).  With K_X/K_Y ≥
      --  K_min (from P(1,1)/P(2,2) ≥ Min_P_Diag_At_Update in the pre), the
      --  result is pulled at least K_min × (1000 − Max_World_X) ≈ 7.7 m inside
      --  ±1000, giving S_Safe_After_Update ≈ 992.3 m.
      Lemma_S_Safe_Bound (S.X, Float (Fix.X), K_X);
      pragma Assert ((1.0 - K_X) * S.X + Float (Fix.X) * K_X
                     in -S_Safe_After_Update .. S_Safe_After_Update);
      S.X := (1.0 - K_X) * S.X + K_X * Float (Fix.X);
      Lemma_S_Safe_Bound (S.Y, Float (Fix.Y), K_Y);
      pragma Assert ((1.0 - K_Y) * S.Y + Float (Fix.Y) * K_Y
                     in -S_Safe_After_Update .. S_Safe_After_Update);
      S.Y := (1.0 - K_Y) * S.Y + K_Y * Float (Fix.Y);

      --  Covariance update: symmetric Joseph-form scaling.
      --  Let Sx = 1 − K_x ∈ [0,1], Sy = 1 − K_y ∈ [0,1].
      --  For H = [1,0,0,0; 0,1,0,0]:
      --    P_new(i,j) = scale(i) · P_old(i,j) · scale(j)
      --  where scale(1)=Sx, scale(2)=Sy, scale(3)=scale(4)=1.
      --  Damps all cross-covariances involving x or y; prevents P(3,1)/P(4,1)
      --  growing without bound between GPS updates.  Implemented as a nested
      --  sub with narrow Pre/Post so the covariance scaling proves in a
      --  local solver context, isolated from S.X/S.Y/state-update facts.
      Joseph_Scale_P (P, Unit_Float (K_X), Unit_Float (K_Y));
   end Update;

end Rover.Estimation;
