with Rover_HAL; use Rover_HAL;

with Ada.Numerics;
with Rover.Estimation;
with Rover.GNC;
with Rover.Mast_Control;

package body Rover.Autonomous
with SPARK_Mode
is

   --  Steering angles (rad) for the Set_Turn(Around) configuration.
   --  FL/RR are toed inward at -30°; FR/RL at +30°.  These match the
   --  Angles(...) values set by Set_Turn(Around) in rover_hal.adb.
   Pi_Over_6 : constant Float := Float (Ada.Numerics.Pi) / 6.0;
   Around_Steering : constant Rover.Estimation.Corner_Steering :=
     [Front_Left  => -Pi_Over_6,
      Front_Right =>  Pi_Over_6,
      Rear_Left   =>  Pi_Over_6,
      Rear_Right  => -Pi_Over_6];

   type Auto_State is record
      User_Exit : Boolean := False;

      Mast : Rover.Mast_Control.Instance;
   end record;

   ----------------------
   -- Check_User_Input --
   ----------------------

   procedure Check_User_Input (This : in out Auto_State)
   with
     Pre  => Initialized,
     Post => Initialized
   is
      State : Buttons_State;
   begin
      State := Update;
      This.User_Exit := (for some B in Buttons => State (B));
   end Check_User_Input;

   Distance_Threshold : constant := 40;
   --  We're allowed to get this close to an obstacle in Go_Forward.

   ----------------
   -- Go_Forward --
   ----------------

   procedure Go_Forward (This : in out Auto_State) with
     Pre  => Initialized and then
             Rover_HAL.Get_Sonar_Distance >= Distance_Threshold,
     Post => Initialized and then
             Rover.Cannot_Crash,
     Exceptional_Cases =>
       (Rover.Estimation.Estimator_Assumption_Violation => True)
   is
      Distance : Unsigned_32 := Distance_Threshold;
      --  The initial value of Distance (which becomes Last_Distance) is at
      --  worst Distance_Threhold, by our precondition. I.e., if we
      --  instantaneously read the sonar at the start of the subprogram, it
      --  would return a value >= Distance_Threshold.

      Last_Distance : Unsigned_32 with Ghost;

      Sonar_Sampling_Delay : constant := 40;
   begin
      Set_Display_Info ("Going forward");

      --  Go forward...
      Set_Turn (Straight);
      Set_Power (Left, 100);
      Set_Power (Right, 100);

      --  Rotate the mast and check for obstacle

      This.Mast.Set_Speed (200);
      This.Mast.Scan (-60, 60);

      loop
         Check_User_Input (This);
         exit when This.User_Exit;

         This.Mast.Update;

         Last_Distance := Distance;
         Distance := Sonar_Distance;

         Rover_Displacement_Model
           (Distance, Last_Distance, Sonar_Sampling_Delay);
         --  We invoke the Rover displacement model so that SPARK knows the
         --  limits on how far the rover can have traveled since the last
         --  distance measurement.

         exit when Distance < Distance_Threshold;

         pragma Loop_Invariant (Distance >= Distance_Threshold);
         pragma Loop_Invariant (Rover.Cannot_Crash);

         Delay_Milliseconds (Sonar_Sampling_Delay);

         --  GNC: one EKF cycle.  Go_Forward uses Set_Turn(Straight),
         --  so all steering angles are 0.0 rad.
         Rover.GNC.Poll
           (Steering => [others => 0.0]);
      end loop;
   end Go_Forward;

   -----------------
   -- Turn_Around --
   -----------------

   procedure Turn_Around
   with
     Pre  => Initialized,
     Post => Initialized and then
             Rover.Cannot_Crash,
     Exceptional_Cases =>
       (Rover.Estimation.Estimator_Assumption_Violation => True)
   is
   begin
      Set_Display_Info ("Turning around");

      --  Turn around, full speed
      --  TODO: Ramdom direction, keep turning if an obstacle is detected

      Set_Turn (Around);
      Set_Power (Left, -100);
      Set_Power (Right, 100);
      --  Drive the EKF during the turn (50 × 40 ms = 2 000 ms) so that
      --  theta tracks the actual heading change instead of being frozen
      --  for the full 2 s and then applied as one large batch step.
      for I in 1 .. 50 loop
         Delay_Milliseconds (40);
         Rover.GNC.Poll (Steering => Around_Steering);
      end loop;
   end Turn_Around;

   ------------------------
   -- Find_New_Direction --
   ------------------------

   procedure Find_New_Direction (This : in out Auto_State)
     with
      Pre  => Initialized,
      Post => Initialized and then
              (This.User_Exit or else
               Rover_HAL.Get_Sonar_Distance >= Distance_Threshold),
      Exceptional_Cases =>
        (Rover.Estimation.Estimator_Assumption_Violation => True)
   is
      Left_Dist : Unsigned_32 := 0;
      Right_Dist : Unsigned_32 := 0;

      Timeout, Now : Time;

      function Distance_Straight_Ahead return Unsigned_32 with
         Side_Effects,
         Pre  => Initialized,
         Post => Initialized and then
                 --  We have to make this promise because otherwise SPARK
                 --  cannot know that the result actually came from the
                 --  sonar.
                 Rover_HAL.Get_Sonar_Distance = Distance_Straight_Ahead'Result

      is
      begin
         Rover_HAL.Set_Mast_Angle (0);
         --  Set the mast to the straight position - this can take a bit of
         --  time, so:
         Delay_Milliseconds (50);

         return Distance : Unsigned_32 do
            Distance := Sonar_Distance;
         end return;
         --  Extended return used because SPARK won't allow a simple return
         --  statement here, because the function has side effects.
      end Distance_Straight_Ahead;

      Distance : Unsigned_32;
   begin
      Set_Display_Info ("Finding direction");

      Now := Clock;
      Timeout := Now + Milliseconds (10000);

      Set_Turn (Straight);
      Set_Power (Left, 0);
      Set_Power (Right, 0);

      --  Measure the distance straight ahead
      Distance := Distance_Straight_Ahead;

      This.Mast.Set_Speed (50);
      This.Mast.Scan (-60, 60);

      while Distance < Distance_Threshold and then not This.User_Exit loop
         --  Turn the mast back and forth and log the dected distance for the
         --  left and right side.
         loop
            Check_User_Input (This);
            Now := Clock;

            exit when This.User_Exit or else Now > Timeout;

            This.Mast.Update;

            if This.Mast.Current_Angle <= -40 then
               Left_Dist := Sonar_Distance;
            end if;
            if This.Mast.Current_Angle >= 40 then
               Right_Dist := Sonar_Distance;
            end if;

            Delay_Milliseconds (30);
         end loop;

         if Now > Timeout then
            if Left_Dist < 50 and then Right_Dist < 50 then
               --  Obstacles left and right, turn around to find a new
               --  direction
               Turn_Around;
            elsif Left_Dist > Right_Dist then
               --  Turn left a little (20 × 40 ms = 800 ms)
               Set_Display_Info ("Turning left");
               Set_Turn (Around);
               Set_Power (Left, -100);
               Set_Power (Right, 100);
               for I in 1 .. 20 loop
                  Delay_Milliseconds (40);
                  Rover.GNC.Poll (Steering => Around_Steering);
               end loop;
            else
               --  Turn right a little (20 × 40 ms = 800 ms)
               Set_Display_Info ("Turning right");
               Set_Turn (Around);
               Set_Power (Left, 100);
               Set_Power (Right, -100);
               for I in 1 .. 20 loop
                  Delay_Milliseconds (40);
                  Rover.GNC.Poll (Steering => Around_Steering);
               end loop;
            end if;

            Distance := Distance_Straight_Ahead;
         end if;
      end loop;
   end Find_New_Direction;

   ---------
   -- Run --
   ---------

   procedure Run is
      State : Auto_State;
   begin
      Set_Display_Info ("Autonomous");

      while not State.User_Exit loop
         Find_New_Direction (State);
         exit when State.User_Exit;

         Go_Forward (State);

         pragma Loop_Invariant (Rover.Cannot_Crash);
      end loop;

      --  Stop everything before leaving the autonomous mode
      Set_Turn (Straight);
      Set_Power (Left, 0);
      Set_Power (Right, 0);
   end Run;

end Rover.Autonomous;
