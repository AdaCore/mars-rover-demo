with Rover_HAL; use Rover_HAL;

package Rover.Mast_Control with SPARK_Mode is

   type Instance is tagged private;

   type Degree_Per_Seconds is range 1 .. 250;

   procedure Set_Speed (This : in out Instance;
                        Speed : Degree_Per_Seconds)
     with Pre'Class  => Rover_HAL.Initialized,
          Post'Class => Rover_HAL.Initialized;
   --  Set the angular speed for futur motion of the mast

   procedure Scan (This     : in out Instance;
                   Min, Max :        Mast_Angle)
     with Pre'Class  => Rover_HAL.Initialized and then Min <= Max,
          Post'Class => Rover_HAL.Initialized,
          Global     => (Input  => Rover_HAL.HW_Init,
                         In_Out => Rover_HAL.HW_State);
   --  Enable scan mode with automatic sweep between the Min and Max value
   --  (requires regular calls to the Update procedure). The mast will first
   --  reach the Min position

   procedure Move_To (This                : in out Instance;
                      Angle               :        Mast_Angle;
                      Wait_For_Completion :        Boolean := False)
     with Pre'Class  => Rover_HAL.Initialized,
          Post'Class => Rover_HAL.Initialized,
          Global     => (Input  => Rover_HAL.HW_Init,
                         In_Out => Rover_HAL.HW_State);
   --  Enable fixed mode with a target mast angle. When Wait_For_Completion
   --  is True, the procedure will not return until the mast has reached
   --  the target angle. Otherwise, regular calls to the Update procedure
   --  are required to update the angle until it reaches the target.

   procedure Update (This : in out Instance)
     with Pre'Class  => Rover_HAL.Initialized,
          Post'Class => Rover_HAL.Initialized,
          Global     => (Input  => Rover_HAL.HW_Init,
                         In_Out => Rover_HAL.HW_State);
   --  This procedure must be called at regular interval (around 10 times per
   --  second) to update the mast angle for sweep motion in scan mode, or to
   --  reach the target angle in fixed mode.

   function Current_Angle (This : Instance) return Mast_Angle;
   --  Return the current position of the mast

private

   type Mast_Mode is (Scan_Mode, Fixed_Mode);
   type Mast_Direction is (Left, Right);

   type Instance is tagged record
      Mode      : Mast_Mode := Fixed_Mode;
      Angle     : Mast_Angle := 0;
      Direction : Mast_Direction := Left;
      Speed     : Degree_Per_Seconds := 20;

      Scan_Min : Mast_Angle := Mast_Angle'First;
      Scan_Max : Mast_Angle := Mast_Angle'Last;

      Fixed_Target : Mast_Angle := 0;

      Last_Update : Rover_HAL.Time := 0;
   end record;

end Rover.Mast_Control;
