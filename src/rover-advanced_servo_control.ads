with Rover_HAL;

generic
   type Angle_Type is range <>;
   with procedure Set_Angle (A : Angle_Type);
package Rover.Advanced_Servo_Control with SPARK_Mode is

   type Instance is tagged private;

   type Degree_Per_Seconds is range 1 .. 250;

   procedure Set_Speed (This : in out Instance;
                        Speed : Degree_Per_Seconds)
     with Pre'Class  => Rover_HAL.Initialized,
          Post'Class => Rover_HAL.Initialized;
   --  Set the angular speed for futur motion of the servo

   procedure Scan (This     : in out Instance;
                   Min, Max :        Angle_Type)
     with Pre'Class  => Rover_HAL.Initialized and then Min <= Max,
          Post'Class => Rover_HAL.Initialized,
          Global     => (Input  => Rover_HAL.HW_Init,
                         In_Out => Rover_HAL.HW_State);
   --  Enable scan mode with automatic sweep between the Min and Max value
   --  (requires regular calls to the Update procedure). The servo will first
   --  reach the Min position

   procedure Move_To (This                : in out Instance;
                      Angle               :        Angle_Type;
                      Wait_For_Completion :        Boolean := False)
     with Pre'Class  => Rover_HAL.Initialized,
          Post'Class => Rover_HAL.Initialized,
          Global     => (Input  => Rover_HAL.HW_Init,
                         In_Out => Rover_HAL.HW_State);
   --  Enable fixed mode with a target servo angle. When Wait_For_Completion
   --  is True, the procedure will not return until the servo has reached
   --  the target angle. Otherwise, regular calls to the Update procedure
   --  are required to update the angle until it reaches the target.

   procedure Update (This : in out Instance)
     with Pre'Class  => Rover_HAL.Initialized,
          Post'Class => Rover_HAL.Initialized,
          Global     => (Input  => Rover_HAL.HW_Init,
                         In_Out => Rover_HAL.HW_State);
   --  This procedure must be called at regular interval (around 10 times per
   --  second) to update the servo angle for sweep motion in scan mode, or to
   --  reach the target angle in fixed mode.

   function Current_Angle (This : Instance) return Angle_Type;
   --  Return the current position of the mast

private

   type Motion_Mode is (Scan_Mode, Fixed_Mode);
   type Mast_Direction is (Left, Right);

   type Instance is tagged record
      Mode      : Motion_Mode := Fixed_Mode;
      Angle     : Angle_Type := 0;
      Direction : Mast_Direction := Left;
      Speed     : Degree_Per_Seconds := 20;

      Scan_Min : Angle_Type := Angle_Type'First;
      Scan_Max : Angle_Type := Angle_Type'Last;

      Fixed_Target : Angle_Type := 0;

      Last_Update : Rover_HAL.Time := 0;
   end record;

end Rover.Advanced_Servo_Control;
