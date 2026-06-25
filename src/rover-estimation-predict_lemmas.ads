with Ada.Numerics.Elementary_Functions;
use Ada.Numerics.Elementary_Functions;

private package Rover.Estimation.Predict_Lemmas
  with SPARK_Mode, Ghost, Always_Terminates
is

   --  Private child: its spec sees the parent's private-part subtypes
   --  (Wheel_Disp, Scaled_Disp, Cov_Entry, FP_Entry, FPFt_Entry,
   --  FP_2MCov, FPFt_2FP, Unit_Float, Max_Wheel_Disp, Max_FP_12,
   --  Max_FPFt_12) via the SPARK child-package visibility rule.

   --  Ghost lemmas supporting Predict's inductive bound chain.  Null
   --  bodies -- GNATprove proves the postcondition from the parameter
   --  types.  Reduced context (only Pre visible) helps SMT.

   --  Sin (X) in [-1, 1] -- wraps the generic postcondition in a reduced
   --  context.
   procedure Lemma_Sin_Bounded (X : Float)
   with Post => Sin (X) in -1.0 .. 1.0;

   --  Cos (X) in [-1, 1]
   procedure Lemma_Cos_Bounded (X : Float)
   with Post => Cos (X) in -1.0 .. 1.0;

   --  |A| <= MWD, B in [-1,1] -> A*B in [-MWD, MWD]
   procedure Lemma_WD_Trig (A : Wheel_Disp; B : Float)
   with Pre  => B in -1.0 .. 1.0,
        Post => A * B in -Max_Wheel_Disp .. Max_Wheel_Disp;

   --  |A| <= Sc*MWD, B in [-1,1] -> A*B in Scaled_Disp
   procedure Lemma_SD_Trig (A : Scaled_Disp; B : Float)
   with Pre  => B in -1.0 .. 1.0,
        Post => A * B in Scaled_Disp;

   --  |A| <= MWD, B in Odometry_Scale -> A*B in Scaled_Disp
   procedure Lemma_WD_Scale (A : Wheel_Disp; B : Odometry_Scale)
   with Post => A * B in Scaled_Disp;

   --  |A| <= 1 -> A*B in Cov_Entry range.
   procedure Lemma_Unit_Cov (A : Unit_Float; B : Cov_Entry)
   with Post => A * B in -Max_Covariance_Diag .. Max_Covariance_Diag;

   --  |A| <= 1 -> A*B in FP_Entry range.
   procedure Lemma_Unit_FP (A : Unit_Float; B : FP_Entry)
   with Post => A * B in -Max_FP_12 .. Max_FP_12;

   --  A in Cov_Entry, B in Cov_Entry -> A+B in FP_2MCov = [-2*MCov, 2*MCov].
   procedure Lemma_Cov_Plus_Cov (A : Cov_Entry; B : Cov_Entry)
   with Post => A + B in FP_2MCov;

   --  A in FP_2MCov, B in Cov_Entry -> A+B in [-Max_FP_12, Max_FP_12].
   procedure Lemma_FP_2MCov_Plus_Cov (A : FP_2MCov; B : Cov_Entry)
   with Post => A + B in -Max_FP_12 .. Max_FP_12;

   --  A in FP_Entry, B in FP_Entry -> A+B in FPFt_2FP = [-2*FP_12, 2*FP_12].
   procedure Lemma_FP_Plus_FP (A : FP_Entry; B : FP_Entry)
   with Post => A + B in FPFt_2FP;

   --  A in FPFt_2FP, B in FP_Entry -> A+B in [-Max_FPFt_12, Max_FPFt_12].
   procedure Lemma_FPFt_2FP_Plus_FP (A : FPFt_2FP; B : FP_Entry)
   with Post => A + B in -Max_FPFt_12 .. Max_FPFt_12;

   --  |A*C - B*D| <= 2*Max_Odometry_Scale*Max_Wheel_Disp = Max_Position_Step
   --  when A, B in Scaled_Disp and C, D in [-1,1].
   procedure Lemma_Position_Step (A : Scaled_Disp; C : Float;
                                  B : Scaled_Disp; D : Float)
   with Pre  => C in -1.0 .. 1.0 and then D in -1.0 .. 1.0,
        Post => A * C - B * D in -Max_Position_Step .. Max_Position_Step;

   --  |A*C + B*D| <= 2*Max_Odometry_Scale*Max_Wheel_Disp = Max_Position_Step
   --  when A, B in Scaled_Disp and C, D in [-1,1].
   procedure Lemma_Position_Step_Add (A : Scaled_Disp; C : Float;
                                      B : Scaled_Disp; D : Float)
   with Pre  => C in -1.0 .. 1.0 and then D in -1.0 .. 1.0,
        Post => A * C + B * D in -Max_Position_Step .. Max_Position_Step;

   --  X + delta in [X - Bound, X + Bound] when |delta| <= Bound.
   --  Reduces the range proof to a linear relation the SMT solver
   --  handles directly.  Preconditions ensure X +/- Bound is within
   --  Float range (no overflow).
   procedure Lemma_Step_Bound (X : Float; Delta_V : Float; Bound : Float)
   with Pre  => Delta_V in -Bound .. Bound and then
                X       in -1_000.0 .. 1_000.0 and then
                Bound   in 0.0      .. 1.0,
        Post => X + Delta_V in X - Bound .. X + Bound;

end Rover.Estimation.Predict_Lemmas;
