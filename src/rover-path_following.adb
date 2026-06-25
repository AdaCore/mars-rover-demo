with Ada.Numerics;
with Ada.Numerics.Elementary_Functions;

with Rover_HAL; use Rover_HAL;

package body Rover.Path_Following
  with SPARK_Mode => Off
is
   use Ada.Numerics.Elementary_Functions;

   Pi        : constant Float := Float (Ada.Numerics.Pi);
   Two_Pi    : constant Float := 2.0 * Pi;
   --  Use the spec constant for the clamp so the clamped value is always
   --  within Steering_Angle_Rad's range (avoids 1-ULP violations).
   Max_Steer_Rad : constant Float := Rover.Estimation.Max_Steering_Angle_Rad;

   -------------
   -- Compute --
   -------------

   procedure Compute
     (Estimated   : Rover.Estimation.State;
      Target      : Rover_HAL.Waypoint_Type;
      Steering    : out Rover.Estimation.Corner_Steering;
      Left_Power  : out Rover_HAL.Motor_Power;
      Right_Power : out Rover_HAL.Motor_Power)
   is
      Bearing : Float;
      Alpha   : Float;
      Steer   : Float;
   begin
      --  Bearing from current estimated position to target waypoint.
      Bearing := Arctan (Target.Y - Estimated.Y, Target.X - Estimated.X);

      --  Heading error: signed angle from current heading to target bearing.
      Alpha := Bearing - Estimated.Theta;

      --  Normalise to (-pi, pi].
      Alpha := Alpha - Two_Pi * Float'Floor (Alpha / Two_Pi + 0.5);

      if abs Alpha > Rotate_Threshold then
         --  Large heading error: stop and rotate in place toward the target.
         --  Wheels point straight ahead; differential power spins the rover.
         --  Alpha > 0 → target is to the left → spin CCW:
         --    left side back, right side forward.
         Steering := [others => 0.0];
         if Alpha > 0.0 then
            Left_Power  := -Rotate_Power;
            Right_Power :=  Rotate_Power;
         else
            Left_Power  :=  Rotate_Power;
            Right_Power := -Rotate_Power;
         end if;
      else
         --  Proportional steering clamped to ±40°.
         --  Symmetric 4WIS: front steer in error direction; rear counter-steers.
         --  Positive Corner_Steering = anticlockwise (leftward) per convention.
         Steer := Float'Max (-Max_Steer_Rad, Float'Min (Max_Steer_Rad, Alpha));
         Steering := [Front_Left  =>  Steer,
                      Front_Right =>  Steer,
                      Rear_Left   => -Steer,
                      Rear_Right  => -Steer];
         Left_Power  := Follow_Power;
         Right_Power := Follow_Power;
      end if;
   end Compute;

end Rover.Path_Following;
