with Rover.Autonomous;
with Rover.Estimation;
with Rover.GNC;
with Rover.Remote_Controlled;

package body Rover.Tasks
with SPARK_Mode
is

   ----------
   -- Demo --
   ----------

   procedure Demo is
   begin
      Rover_HAL.Set_Power (Rover_HAL.Left, 100);

      --  GNC path-following mode: if waypoints are loaded, drive the path
      --  once before entering the autonomous / RC loop.
      --  Cannot_Crash proof extension for GNC mode is deferred; see DESIGN.md.
      if Rover_HAL.Waypoint_Count > 0 then
         begin
            Rover.GNC.Follow_Path;
         exception
            when Rover.Estimation.Estimator_Assumption_Violation =>
               Rover_HAL.Set_Display_Info ("EKF fail; fallback");
         end;
      end if;

      --  Alternate between autonomous and remote controlled mode. Automous
      --  will run until a command is received from the remote, remote
      --  controlled will run as long as commands are received from the remote.
      --  An EKF assumption violation raised from Autonomous is caught here;
      --  we print a short message and continue the loop.
      loop
         begin
            Rover.Autonomous.Run;
            Rover.Remote_Controlled.Run;
         exception
            when Rover.Estimation.Estimator_Assumption_Violation =>
               --  Restore a safe control state so that the loop invariant
               --  Cannot_Crash holds for the next iteration.
               Rover_HAL.Set_Turn (Rover_HAL.Straight);
               Rover_HAL.Set_Power (Rover_HAL.Left, 0);
               Rover_HAL.Set_Power (Rover_HAL.Right, 0);
               Rover_HAL.Set_Display_Info ("EKF retry");
         end;

         pragma Loop_Invariant (Rover.Cannot_Crash);
      end loop;
   end Demo;

end Rover.Tasks;
