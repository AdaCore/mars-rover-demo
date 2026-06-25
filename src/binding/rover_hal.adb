with System;

package body Rover_HAL
with
  SPARK_Mode => Off
is

   -----------------
   -- Initialized --
   -----------------

   function Initialized return Boolean is
   begin
      return True;
   end Initialized;

   ----------------
   -- Initialize --
   ----------------

   procedure Initialize is null;

   -----------
   -- Timer --
   -----------

   function Clock return Time is
      function Clock_Import return Unsigned_64;
      pragma Import (C, Clock_Import, "mars_rover_clock");
   begin
      return Time (Clock_Import);
   end Clock;

   ------------------------
   -- Delay_Microseconds --
   ------------------------

   procedure Delay_Microseconds (Us : Unsigned_16) is
      procedure Delay_Microsecsonds_Inport (Us : Unsigned_16);
      pragma Import (C, Delay_Microsecsonds_Inport,
                     "mars_rover_delay_microseconds");
   begin
      Delay_Microsecsonds_Inport (Us);
   end Delay_Microseconds;

   ------------------------
   -- Delay_Milliseconds --
   ------------------------

   procedure Delay_Milliseconds (Ms : Unsigned_16) is
      procedure Delay_Milliseconds_Import (Ms : Unsigned_16);
      pragma Import (C, Delay_Milliseconds_Import,
                     "mars_rover_delay_milliseconds");
   begin
      Delay_Milliseconds_Import (Ms);
   end Delay_Milliseconds;

   --------------
   -- Encoders --
   --------------

   function Read_Encoder_Ticks (Wheel : Corner_Wheel_Id) return Encoder_Ticks is
      function Import (Wheel : Unsigned_8) return Integer_16;
      pragma Import (C, Import, "mars_rover_encoder_ticks");
   begin
      return Encoder_Ticks (Import (Corner_Wheel_Id'Enum_Rep (Wheel)));
   end Read_Encoder_Ticks;

   ------------------------
   -- EKF_Reset_Pending  --
   ------------------------

   function EKF_Reset_Pending return Boolean is
      function Import return Integer_32;
      pragma Import (C, Import, "mars_rover_ekf_reset_pending");
   begin
      return Import /= 0;
   end EKF_Reset_Pending;

   function EKF_Reset_X return World_X is
      function Import return Integer_32;
      pragma Import (C, Import, "mars_rover_ekf_reset_x");
      Scale : constant := 1_000_000.0;
   begin
      return World_X (Float (Import) / Scale);
   end EKF_Reset_X;

   function EKF_Reset_Y return World_Y is
      function Import return Integer_32;
      pragma Import (C, Import, "mars_rover_ekf_reset_y");
      Scale : constant := 1_000_000.0;
   begin
      return World_Y (Float (Import) / Scale);
   end EKF_Reset_Y;

   function EKF_Reset_Theta return Heading is
      function Import return Integer_32;
      pragma Import (C, Import, "mars_rover_ekf_reset_theta");
      Scale : constant := 1_000_000.0;
   begin
      return Heading (Float (Import) / Scale);
   end EKF_Reset_Theta;

   ---------
   -- GPS --
   ---------

   function GPS_Fix return GPS_Fix_Type is
      function GPS_X_Import return Integer_32;
      pragma Import (C, GPS_X_Import, "mars_rover_gps_x");
      function GPS_Y_Import return Integer_32;
      pragma Import (C, GPS_Y_Import, "mars_rover_gps_y");
      function GPS_Timestamp_Import return Unsigned_32;
      pragma Import (C, GPS_Timestamp_Import, "mars_rover_gps_timestamp");
      Scale : constant := 1_000_000.0;
   begin
      return (X         => World_X (Float (GPS_X_Import) / Scale),
              Y         => World_Y (Float (GPS_Y_Import) / Scale),
              Timestamp => GPS_Timestamp_Import);
   end GPS_Fix;

   ---------
   -- IMU --
   ---------

   function Read_IMU_Gyro_Z return Gyro_Rate_Raw is
      function Import return Integer_16;
      pragma Import (C, Import, "mars_rover_imu_gyro_z");
   begin
      return Gyro_Rate_Raw (Import);
   end Read_IMU_Gyro_Z;

   ---------------
   -- Waypoints --
   ---------------

   function Waypoint_Count return Natural is
      function Import return Unsigned_32;
      pragma Import (C, Import, "mars_rover_waypoint_count");
   begin
      return Natural (Import);
   end Waypoint_Count;

   function Get_Waypoint (Idx : Natural) return Waypoint_Type is
      function Import_X (I : Unsigned_32) return Integer_32;
      pragma Import (C, Import_X, "mars_rover_waypoint_x");
      function Import_Y (I : Unsigned_32) return Integer_32;
      pragma Import (C, Import_Y, "mars_rover_waypoint_y");
      Scale : constant := 1_000_000.0;
      I     : constant Unsigned_32 := Unsigned_32 (Idx);
   begin
      return (X => Float (Import_X (I)) / Scale,
              Y => Float (Import_Y (I)) / Scale);
   end Get_Waypoint;

   -----------
   -- Sonar --
   -----------

   function Sonar_Distance return Unsigned_32 is
      function Sonar_Distance_Import return Unsigned_32;
      pragma Import (C, Sonar_Distance_Import, "mars_rover_sonar_distance");

   begin
      return Sonar_Distance_Import;
   end Sonar_Distance;

   ----------
   -- Mast --
   ----------

   procedure Set_Mast_Angle (V : Mast_Angle) is
      procedure Set_Mast_Angle_Import (V : Integer_8);
      pragma Import (C, Set_Mast_Angle_Import, "mars_rover_set_mast_angle");
   begin
      Set_Mast_Angle_Import (Integer_8 (V));
   end Set_Mast_Angle;

   ------------
   -- Remote --
   ------------

   function Update return Buttons_State is
      function Controller_State_Import return Unsigned_16;
      pragma Import (C, Controller_State_Import,
                     "mars_rover_controller_state");

      Raw : constant Unsigned_16 := Controller_State_Import;
      Result : Buttons_State;
   begin
      for B in Buttons loop
         Result (B) := (Raw and Unsigned_16 (2**Buttons'Pos (B))) /= 0;
      end loop;
      return Result;
   end Update;

   ------------
   -- Wheels --
   ------------

   procedure Set_Wheel_Angle (Wheel : Steering_Wheel_Id;
                              Side  : Side_Id;
                              V     : Steering_Wheel_Angle)
   is
      procedure Set_Wheel_Angle_Inport (Wheel : Unsigned_8;
                                        Side  : Unsigned_8;
                                        V     : Integer_8);
      pragma Import (C, Set_Wheel_Angle_Inport, "mars_rover_set_wheel_angle");
   begin
      Set_Wheel_Angle_Inport (Wheel'Enum_Rep, Side'Enum_Rep, Integer_8 (V));
   end Set_Wheel_Angle;

   --------------
   -- Set_Turn --
   --------------

   procedure Set_Turn (Turn : Turn_Kind) is
      type Wheels_Angle
      is array (Steering_Wheel_Id, Side_Id) of Steering_Wheel_Angle;

      Angles : Wheels_Angle;

      Inside_Angle : constant := 30;
      Outside_Angle : constant := 20;
   begin
      case Turn is
         when Straight =>
            Angles := [others => [others => 0]];
         when Left =>
            --  Inside
            Angles (Front, Left) := Inside_Angle;
            Angles (Back, Left) := -Inside_Angle;

            --  Outside
            Angles (Front, Right) := Outside_Angle;
            Angles (Back, Right) := -Outside_Angle;
         when Right =>
            --  Inside
            Angles (Front, Right) := -Inside_Angle;
            Angles (Back, Right) := Inside_Angle;

            --  Outside
            Angles (Front, Left) := -Outside_Angle;
            Angles (Back, Left) := Outside_Angle;

         when Around =>
            Angles (Front, Left) := -Inside_Angle;
            Angles (Front, Right) := Inside_Angle;

            Angles (Back, Left) := Inside_Angle;
            Angles (Back, Right) := -Inside_Angle;
      end case;

      for Side in Side_Id loop
         for Wheel in Steering_Wheel_Id loop
            Set_Wheel_Angle (Wheel, Side, Angles (Wheel, Side));
         end loop;
      end loop;
   end Set_Turn;

   ---------------
   -- Set_Power --
   ---------------

   procedure Set_Power (Side : Side_Id;
                        Pwr  : Motor_Power)
   is
      procedure Set_Power_Import (Side : Unsigned_8;
                                  Pwr  : Integer_8);
      pragma Import (C, Set_Power_Import, "mars_rover_set_power");
   begin
      Set_Power_Import (Side'Enum_Rep, Integer_8 (Pwr));
   end Set_Power;

   ----------------------------
   -- Set_Estimated_Position --
   ----------------------------

   procedure Set_Estimated_Position
     (X : World_X; Y : World_Y; Theta : Heading)
   is
      procedure Import (Est_X, Est_Y, Est_Theta : Interfaces.Integer_32);
      pragma Import (C, Import, "mars_rover_set_estimated_position");
      Scale : constant := 1_000_000.0;
   begin
      Import (Interfaces.Integer_32 (Float (X) * Scale),
              Interfaces.Integer_32 (Float (Y) * Scale),
              Interfaces.Integer_32 (Float (Theta) * Scale));
   end Set_Estimated_Position;

   -----------------------
   -- Report_GNC_State  --
   -----------------------

   procedure Report_GNC_State
     (GPS_X, GPS_Y,
      Enc_FL, Enc_FR, Enc_RL, Enc_RR : Interfaces.Integer_32)
   is
      procedure Import
        (P1, P2, P3, P4, P5, P6 : Interfaces.Integer_32);
      pragma Import (C, Import, "mars_rover_report_gnc_state");
   begin
      Import (GPS_X, GPS_Y, Enc_FL, Enc_FR, Enc_RL, Enc_RR);
   end Report_GNC_State;

   ----------------------
   -- Set_Display_Info --
   ----------------------

   procedure Set_Display_Info (Str : String) is
      procedure Display_Import (Addr : System.Address; Len : Integer);
      pragma Import (C, Display_Import, "mars_rover_set_display_info");
   begin
      Display_Import (Str'Address, Str'Length);
   end Set_Display_Info;

end Rover_HAL;
