with Rover_HAL;
with Rover.Estimation;

package Rover.GNC
  with SPARK_Mode,
       Abstract_State => GNC_State,
       Initializes    => GNC_State
is

   procedure Poll (Steering : Rover.Estimation.Corner_Steering)
     with
       Pre    => Rover_HAL.Initialized,
       Global => (Input  => (Rover_HAL.HW_Init,
                             Rover_HAL.GPS_State,
                             Rover_HAL.IMU_State),
                  In_Out => (Rover_HAL.Encoder_State,
                             Rover_HAL.HW_State,
                             GNC_State)),
       Exceptional_Cases =>
         (Rover.Estimation.Estimator_Assumption_Violation => True);
   --  Run one EKF cycle:
   --    1. Read encoder deltas and GPS fix from the HAL.
   --    2. Rover.Estimation.Predict with the current Steering angles.
   --    3. If a fresh GPS fix is available, run Rover.Estimation.Update.
   --    4. Push the EKF estimate to the simulator via Set_Estimated_Position.
   --  Also pushes encoder totals and sensed GPS to the console display.
   --  On hardware, Set_Estimated_Position is a no-op until telemetry is added.

   procedure Follow_Path
     with
       Pre    => Rover_HAL.Initialized,
       Global => (Input  => (Rover_HAL.HW_Init,
                             Rover_HAL.GPS_State,
                             Rover_HAL.IMU_State),
                  In_Out => (Rover_HAL.Encoder_State,
                             Rover_HAL.HW_State,
                             Rover_HAL.Distance_State,
                             Rover_HAL.Power_State,
                             GNC_State)),
       Exceptional_Cases =>
         (Rover.Estimation.Estimator_Assumption_Violation => True);
   --  Drive the rover through the waypoints loaded in the HAL.
   --  Returns when all waypoints have been captured or none are loaded.
   --  Waits for EKF initialisation before commanding motion.
   --  Sonar preemption is active throughout; power zeroed if obstacle detected.
   --  Cannot_Crash proof extension is deferred; see DESIGN.md.

end Rover.GNC;
