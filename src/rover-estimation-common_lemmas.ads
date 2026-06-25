package Rover.Estimation.Common_Lemmas
  with SPARK_Mode, Ghost, Always_Terminates
is

   --  Public child package (Ghost): lemmas shared between Predict and
   --  Update.  Its spec sees the parent's private-part subtypes
   --  (Cov_Entry, Unit_Float, ...) via the SPARK child-package
   --  visibility rule.

   --  Factor in [0,1], V in [-MCov, MCov] -> Factor*V in [-MCov, MCov].
   --  Used to prove off-diagonal covariance entries stay in bounds
   --  after scaling.
   procedure Lemma_Scale_Cov (Factor : Float; V : Float)
   with Pre  => Factor in 0.0 .. 1.0 and then
                V in -Max_Covariance_Diag .. Max_Covariance_Diag,
        Post => Factor * V in -Max_Covariance_Diag .. Max_Covariance_Diag;

   --  A, B in [0,1] -> A*B in [0,1].  Used to prove the combined scale
   --  Sx*Sy is also a unit factor.
   procedure Lemma_Product_Unit_Interval (A, B : Float)
   with Pre  => A in 0.0 .. 1.0 and then B in 0.0 .. 1.0,
        Post => A * B in 0.0 .. 1.0;

end Rover.Estimation.Common_Lemmas;
