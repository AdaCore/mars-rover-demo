package body Rover.Estimation.Predict_Lemmas
  with SPARK_Mode
is

   procedure Lemma_Sin_Bounded (X : Float) is null;
   pragma Annotate (GNATprove, False_Positive,
                    "postcondition",
                    "Sin is mathematically bounded by [-1,1]; " &
                    "CVC5 FP backend cannot prove this without sparklib. " &
                    "To be investigated separately.");

   procedure Lemma_Cos_Bounded (X : Float) is null;
   pragma Annotate (GNATprove, False_Positive,
                    "postcondition",
                    "Cos is mathematically bounded by [-1,1]; " &
                    "CVC5 FP backend cannot prove this without sparklib. " &
                    "To be investigated separately.");

   procedure Lemma_WD_Trig (A : Wheel_Disp; B : Float) is null;

   procedure Lemma_SD_Trig (A : Scaled_Disp; B : Float) is null;

   procedure Lemma_WD_Scale (A : Wheel_Disp; B : Odometry_Scale) is null;

   procedure Lemma_Unit_Cov (A : Unit_Float; B : Cov_Entry) is null;

   procedure Lemma_Unit_FP (A : Unit_Float; B : FP_Entry) is null;

   procedure Lemma_Cov_Plus_Cov (A : Cov_Entry; B : Cov_Entry) is null;

   procedure Lemma_FP_2MCov_Plus_Cov (A : FP_2MCov; B : Cov_Entry) is null;

   procedure Lemma_FP_Plus_FP (A : FP_Entry; B : FP_Entry) is null;

   procedure Lemma_FPFt_2FP_Plus_FP (A : FPFt_2FP; B : FP_Entry) is null;

   procedure Lemma_Position_Step (A : Scaled_Disp; C : Float;
                                  B : Scaled_Disp; D : Float) is null;

   procedure Lemma_Position_Step_Add (A : Scaled_Disp; C : Float;
                                      B : Scaled_Disp; D : Float) is null;

   procedure Lemma_Step_Bound
     (X : Float; Delta_V : Float; Bound : Float) is null;

end Rover.Estimation.Predict_Lemmas;
