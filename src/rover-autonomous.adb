with Rover_HAL; use Rover_HAL;

with Rover.Mast_Control;
with Rover.Arm_A_Control;
with Rover.Arm_B_Control;

package body Rover.Autonomous
with SPARK_Mode
is
   type Drill_Angle_Step is (Low, Mid, High);
   A_In_Angle  : constant Rover_HAL.Arm_Angle_A := Rover_HAL.Arm_Angle_A'Last;

   B_Out_Angle : constant Rover_HAL.Arm_Angle_B := Rover_HAL.Arm_Angle_B'First;
   B_In_Angle  : constant Rover_HAL.Arm_Angle_B := Rover_HAL.Arm_Angle_B'Last;

   type Auto_State is record
      User_Exit : Boolean := False;

      Mast : Rover.Mast_Control.Instance;
      Arm_A : Rover.Arm_A_Control.Instance;
      Arm_B : Rover.Arm_B_Control.Instance;
      Next_Drill_Angle : Drill_Angle_Step := Drill_Angle_Step'First;
   end record;

   ------------------
   -- Servo_Update --
   ------------------

   procedure Servo_Update (This : in out Auto_State) is
   begin
      This.Mast.Update;
      This.Arm_A.Update;
      This.Arm_B.Update;
   end Servo_Update;

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
             Rover.Cannot_Crash
   is
      Distance : Unsigned_32 := Distance_Threshold;
      --  The initial value of Distance (which becomes Last_Distance) is at
      --  worst Distance_Threhold, by our precondition. I.e., if we
      --  instantaneously read the sonar at the start of the subprogram, it
      --  would return a value >= Distance_Threshold.

      Last_Distance : Unsigned_32 with Ghost;

      Sonar_Sampling_Delay : constant := 40;
   begin
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

         Servo_Update (This);

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
      end loop;
   end Go_Forward;

   -----------------
   -- Turn_Around --
   -----------------

   procedure Turn_Around
   with
     Pre  => Initialized,
     Post => Initialized and then
             Rover.Cannot_Crash
   is
   begin
      --  Turn around, full speed
      --  TODO: Ramdom direction, keep turning if an obstacle is detected

      Set_Turn (Around);
      Set_Power (Left, -100);
      Set_Power (Right, 100);
      Delay_Milliseconds (2000);
   end Turn_Around;

   ------------------------
   -- Find_New_Direction --
   ------------------------

   procedure Find_New_Direction (This : in out Auto_State)
     with
      Pre  => Initialized,
      Post => Initialized and then
              (This.User_Exit or else
               Rover_HAL.Get_Sonar_Distance >= Distance_Threshold)
   is
      Left_Mast_Angle : constant := -60;
      Right_Mast_Angle : constant := 60;

      Left_Dist : Unsigned_32;
      Right_Dist : Unsigned_32;
      Distance : Unsigned_32;
   begin

      Set_Power (Left, 0);
      Set_Power (Right, 0);

      loop
         --  Start with the mast straight ahead
         This.Mast.Set_Speed (40);
         This.Mast.Move_To (0, Wait_For_Completion => True);
         Delay_Milliseconds (500);

         This.Mast.Move_To (Left_Mast_Angle, Wait_For_Completion => True);
         Delay_Milliseconds (500);
         Left_Dist := Sonar_Distance;

         This.Mast.Move_To (Right_Mast_Angle, Wait_For_Completion => True);
         Delay_Milliseconds (500);
         Right_Dist := Sonar_Distance;

         --  Choose the next direction:
         if Left_Dist < 50 and then Right_Dist < 50 then
            --  Obstacles left and right, turn around to find a new
            --  direction
            Turn_Around;
         elsif Left_Dist > Right_Dist then
            --  Look left
            This.Mast.Move_To (Left_Mast_Angle, Wait_For_Completion => True);
            --  Turn left a little
            Set_Turn (Around);
            Set_Power (Left, -100);
            Set_Power (Right, 100);
            Delay_Milliseconds (800);
         else
            --  Look right
            This.Mast.Move_To (Right_Mast_Angle, Wait_For_Completion => True);
            --  Turn right a little
            Set_Turn (Around);
            Set_Power (Left, 100);
            Set_Power (Right, -100);
            Delay_Milliseconds (800);
         end if;

         --  Stop the rotation
         Set_Power (Left, 0);
         Set_Power (Right, 0);

         --  Measure the distance straight ahead
         This.Mast.Move_To (0, Wait_For_Completion => True);
         Delay_Milliseconds (500);
         Distance := Sonar_Distance;

         Check_User_Input (This);
         exit when This.User_Exit or else Distance >= Distance_Threshold;
      end loop;
   end Find_New_Direction;

   ------------------------
   -- Drilling_Animation --
   ------------------------

   procedure Drilling_Animation (This : in out Auto_State)
     with
      Pre  => Initialized,
      Post => Initialized
   is
      A_Out_Angle  : constant Rover_HAL.Arm_Angle_A :=
        (Rover_HAL.Arm_Angle_A'Last / 10) *
        (case This.Next_Drill_Angle is
            when Low  => 1,
            when Mid  => 3,
            when High => 6);

      Drilling_Time : constant Time := Milliseconds (4_000);
      Drilling_Timeout : Time := 0;
      Now : Time;
      type Steps is (Start, A_Out, B_Out, Drilling, B_In, A_In);

      Current_Step : Steps := Steps'First;

   begin

      if This.Next_Drill_Angle = Drill_Angle_Step'Last then
         This.Next_Drill_Angle := Drill_Angle_Step'First;
      else
         This.Next_Drill_Angle :=
           Drill_Angle_Step'Succ (This.Next_Drill_Angle);
      end if;

      Set_Turn (Straight);
      Set_Power (Left, 0);
      Set_Power (Right, 0);
      This.Mast.Move_To (0);

      This.Arm_A.Set_Speed (10);
      This.Arm_B.Set_Speed (10);

      loop
         Servo_Update (This);

         case Current_Step is
            when Start =>
               Current_Step := A_Out;
               This.Arm_A.Move_To (A_Out_Angle);

            when A_Out =>
               if This.Arm_A.Current_Angle = A_Out_Angle then
                  This.Arm_B.Move_To (B_Out_Angle);
                  Current_Step := B_Out;
               end if;

            when B_Out =>
               if This.Arm_B.Current_Angle = B_Out_Angle then
                  Current_Step := Drilling;
                  Now := Clock;
                  Drilling_Timeout := Now + Drilling_Time;
               end if;

            when Drilling =>
               Now := Clock;
               if Now > Drilling_Timeout then
                  This.Arm_B.Move_To (B_In_Angle);
                  Current_Step := B_In;
               end if;

            when B_In =>
               if This.Arm_B.Current_Angle = B_In_Angle then
                  Current_Step := A_In;
                  This.Arm_A.Move_To (A_In_Angle);
               end if;

            when A_In =>
               if This.Arm_A.Current_Angle = A_In_Angle then
                  return;
               end if;
         end case;

         Delay_Milliseconds (10);
      end loop;
   end Drilling_Animation;

   ---------
   -- Run --
   ---------

   procedure Run is
      State : Auto_State;

      type Drilling_Animation_Time is mod 5;
      --  Drill every 5 loop
      Drilling_Timer : Drilling_Animation_Time := 0;

   begin
      Set_Display_Info ("Autonomous");

      Set_Turn (Straight);
      Set_Power (Left, 0);
      Set_Power (Right, 0);

      State.Arm_B.Move_To (B_In_Angle, Wait_For_Completion => True);
      State.Arm_A.Move_To (A_In_Angle, Wait_For_Completion => True);

      while not State.User_Exit loop
         if Drilling_Timer = 0 then
            Drilling_Animation (State);
            exit when State.User_Exit;
         end if;

         Find_New_Direction (State);
         exit when State.User_Exit;

         Go_Forward (State);

         Drilling_Timer := Drilling_Timer + 1;
         pragma Loop_Invariant (Rover.Cannot_Crash);
      end loop;

      --  Stop everything before leaving the autonomous mode
      Set_Turn (Straight);
      Set_Power (Left, 0);
      Set_Power (Right, 0);
   end Run;

end Rover.Autonomous;
