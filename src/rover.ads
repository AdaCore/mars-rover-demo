with Interfaces; use Interfaces;
with Rover_HAL;

package Rover with SPARK_Mode is

   Safety_Distance : constant Unsigned_32 := 20;

   function Cannot_Crash return Boolean
     with
       Pre    => Rover_HAL.Initialized,
       Global => (Rover_HAL.HW_Init,
                  Rover_HAL.Power_State,
                  Rover_HAL.Turn_State,
                  Rover_HAL.Distance_State),
       Ghost;
   --  Safety Requirement: The Mars Rover shall not proceed straight forward
   --  when the distance to an obstacle is less than the Safety Distance.

   Max_Speed : constant Float := 0.015;
   --  The maximum speed of the rover in units of distance per millisecond.
   --  Measured on a Rover instance with fresh batters as 0.0112 - rounded up
   --  here for added safety margin.

   function Max_Distance
     (Last_Distance : Unsigned_32;
      Ellapsed_Time : Unsigned_32)
   return Unsigned_32
     with
       Ghost;

   procedure Rover_Displacement_Model
     (Distance      : Unsigned_32;
      Last_Distance : Unsigned_32;
      Ellapsed_Time : Unsigned_32)
   with
     Global => null,
     Post => Distance >= Max_Distance (Last_Distance, Ellapsed_Time),
     Ghost,
     Import,
     Always_Terminates;

private
   use type Rover_HAL.Turn_Kind;
   use type Rover_HAL.Motor_Power;

   function Cannot_Crash return Boolean is
     (if Rover_HAL.Get_Sonar_Distance < Safety_Distance and then
         Rover_HAL.Get_Turn = Rover_HAL.Straight
      then
         Rover_HAL.Get_Power (Rover_HAL.Left)  <= 0 and then
         Rover_HAL.Get_Power (Rover_HAL.Right) <= 0);

   function Max_Distance
     (Last_Distance : Unsigned_32;
      Ellapsed_Time : Unsigned_32)
   return Unsigned_32 is
     (declare
        Displacement renames Unsigned_32 (Max_Speed * Float (Ellapsed_Time));
     begin
        (if Last_Distance < Displacement then
           0
        else
           Last_Distance - Displacement));
end Rover;
