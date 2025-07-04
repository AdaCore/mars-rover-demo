with Rover_HAL;

package Rover.Tasks
with SPARK_Mode
is

   procedure Demo
     with
      Pre => Rover_HAL.Initialized,
      No_Return;

   pragma Export (C, Demo, "mars_rover_demo_task");

end Rover.Tasks;
