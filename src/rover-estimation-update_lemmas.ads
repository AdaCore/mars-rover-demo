with Rover_HAL;

private package Rover.Estimation.Update_Lemmas
  with SPARK_Mode, Ghost, Always_Terminates
is

   --  Private child package (Ghost): Update's lemmas.  Its spec sees the
   --  parent's private-part subtypes via the SPARK child-package
   --  visibility rule.  The parameter types are kept as plain Float
   --  with explicit Pre clauses to preserve the semantics of the
   --  inline Silver-era lemmas.

   --  Lower bound on the Kalman gain at any Update call site.  Since
   --  P(i,i) >= Min_P_Diag_At_Update on entry and K = P/(P+R) is
   --  monotone increasing in P, K >= K_Min.
   K_Min : constant Float :=
     Min_P_Diag_At_Update / (Min_P_Diag_At_Update + R_GPS);

   --  (1 - K) * V in [0, V] when K in [0,1] and V in [0, MCov].  The
   --  parameter Pre carries the range information so the SMT solver can
   --  prove the product bounds without nonlinear case analysis.
   procedure Lemma_Gain_Contract (K : Float; V : Float)
   with Pre  => K in 0.0 .. 1.0 and then
                V in 0.0 .. Max_Covariance_Diag,
        Post => (1.0 - K) * V in 0.0 .. V;

   --  K = P_Val / (P_Val + R_GPS) >= K_Min when P_Val >= Min_P_Diag_At_Update.
   --  K is monotone increasing in P (partial w.r.t. P = R / (P+R)^2 > 0);
   --  the proof reduces to the linear fact
   --    P_Val * R_GPS >= Min_P_Diag_At_Update * R_GPS.
   procedure Lemma_Gain_Monotone (P_Val : Float)
   with Pre  => P_Val in Min_P_Diag_At_Update .. Max_Covariance_Diag,
        Post => P_Val / (P_Val + R_GPS) >= K_Min;

   --  Convex combination (1-K)*X + K*Fix is in [-S_Safe, S_Safe] when
   --  K >= K_min, X in [-1000, 1000], Fix in [-Max_World_X, Max_World_X].
   --  Key: S_Safe = (1-K_min)*1000 + K_min*Max_World_X = 1000 - K_min*994.
   --  Since K >= K_min and Max_World_X < 1000, the combination can only
   --  be tighter (further from +/-1000) than the K_min case.
   procedure Lemma_S_Safe_Bound (X   : Float;
                                 Fix : Float;
                                 K   : Float)
   with Pre  => X   in -1_000.0 .. 1_000.0      and then
                Fix in -Rover_HAL.Max_World_X .. Rover_HAL.Max_World_X and then
                K   in K_Min .. 1.0,
        Post => (1.0 - K) * X + K * Fix
                in -S_Safe_After_Update .. S_Safe_After_Update;

end Rover.Estimation.Update_Lemmas;
