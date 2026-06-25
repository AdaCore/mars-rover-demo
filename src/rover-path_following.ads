with Ada.Numerics;
with Rover_HAL;
with Rover.Estimation;

package Rover.Path_Following
  with SPARK_Mode
is

   Capture_Radius : constant Float := 0.05;
   --  A waypoint is considered reached when the estimated position is within
   --  this radius (metres).  2× GPS σ — tight enough for accurate waypoint
   --  tracking with the EKF in steady state.

   Rotate_Threshold : constant Float := Float (Ada.Numerics.Pi) / 4.0;
   --  Heading-error magnitude (45°) above which the rover stops and rotates
   --  in place before driving forward.

   Follow_Power : constant Rover_HAL.Motor_Power := 60;
   --  Forward motor power during path following.  Half throttle; tunable.

   Rotate_Power : constant Rover_HAL.Motor_Power := 40;
   --  Motor power for in-place rotation prior to resuming forward drive.

   procedure Compute
     (Estimated   : Rover.Estimation.State;
      Target      : Rover_HAL.Waypoint_Type;
      Steering    : out Rover.Estimation.Corner_Steering;
      Left_Power  : out Rover_HAL.Motor_Power;
      Right_Power : out Rover_HAL.Motor_Power)
     with Global => null;
   --  Pure computation: no HAL access.
   --
   --  If the heading error |α| ≤ Rotate_Threshold (45°): steers proportionally
   --  toward the target with both sides at Follow_Power (normal drive).
   --
   --  If |α| > Rotate_Threshold: zeros all wheel angles and outputs differential
   --  power for an in-place spot turn toward the target bearing.  The rover
   --  will spin until α falls within threshold, after which normal drive resumes.

end Rover.Path_Following;
