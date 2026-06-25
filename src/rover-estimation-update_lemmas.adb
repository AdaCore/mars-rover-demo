package body Rover.Estimation.Update_Lemmas
  with SPARK_Mode
is

   procedure Lemma_Gain_Contract (K : Float; V : Float) is
   begin
      --  Step 1: 1-K in [0,1] -- arithmetic from K in [0,1].
      pragma Assert (1.0 - K in 0.0 .. 1.0);
      --  Step 2: lower bound -- product of two non-negatives.
      pragma Assert ((1.0 - K) * V >= 0.0);
      --  Step 3: 1.0 * V = V exactly in IEEE 754.
      pragma Assert (1.0 * V = V);
      --  Step 4: monotonicity -- (1-K) <= 1.0 and V >= 0 -> (1-K)*V <= 1*V.
      pragma Assert ((1.0 - K) * V <= 1.0 * V);
   end Lemma_Gain_Contract;

   procedure Lemma_Gain_Monotone (P_Val : Float) is
   begin
      --  Step 1: P_Val >= Min_P -> P_Val * R_GPS >= Min_P * R_GPS.
      pragma Assert (P_Val * R_GPS >= Min_P_Diag_At_Update * R_GPS);
      --  Step 2: rearrange to cross-multiply form for the gain inequality.
      pragma Assert (P_Val * (Min_P_Diag_At_Update + R_GPS)
                     >= Min_P_Diag_At_Update * (P_Val + R_GPS));
      --  Step 3: the division inequality follows from positive denominators.
      pragma Assert (P_Val / (P_Val + R_GPS)
                     >= Min_P_Diag_At_Update / (Min_P_Diag_At_Update + R_GPS));
   end Lemma_Gain_Monotone;

   procedure Lemma_S_Safe_Bound (X   : Float;
                                 Fix : Float;
                                 K   : Float) is null;

end Rover.Estimation.Update_Lemmas;
