with Ada.Numerics;
with Interfaces; use Interfaces;

package Rover_HAL
with
  Abstract_State => (HW_Init,
                     (HW_State      with Synchronous),
                     (Encoder_State with Synchronous),
                     (GPS_State     with Synchronous),
                     (IMU_State     with Synchronous),
                     (Power_State   with Ghost),
                     (Turn_State    with Ghost),
                     (Distance_State with Ghost)),
  Initializes    => (HW_State, Encoder_State, GPS_State, IMU_State,
                     Power_State, Turn_State, Distance_State),
  SPARK_Mode,
  Always_Terminates
is

   function Initialized return Boolean
     with Global => HW_Init;

   procedure Initialize
     with
       Global => (Output => HW_Init),
       Post   => Initialized;

   --------------------
   -- World Geometry --
   --------------------

   Terrain_Half_Width : constant := 5.0;
   --  Half of TERRAIN_WORLD_WIDTH (metres).
   --  Rust: simulator/src/visualizer.rs, TERRAIN_WORLD_WIDTH = 10.0
   --  Wall inner row at X = ±4.9 m (X_MAX=49), outer row at X = ±5.0 m.
   --  If the terrain width changes, update this constant; Max_World_X and
   --  World_X derive from it automatically.

   Terrain_Half_Height : constant := 3.0;
   --  Half of TERRAIN_WORLD_HEIGHT (metres).
   --  Rust: simulator/src/visualizer.rs, TERRAIN_WORLD_HEIGHT = 6.0
   --  Wall inner row at Y = ±2.9 m (Y_MAX=29), outer row at Y = ±3.0 m.
   --  If the terrain height changes, update this constant; Max_World_Y and
   --  World_Y derive from it automatically.

   GPS_Position_Margin : constant := 1.0;
   --  Margin added to terrain half-bounds for GPS noise and brief EKF
   --  transients (metres).
   --  Rust: simulator/src/controller.rs, GPS_SIGMA = 0.1 m (1σ);
   --  3σ = 0.3 m; 0.7 m headroom for EKF transients → total 1.0 m per side.

   Max_World_X : constant Float := Terrain_Half_Width  + GPS_Position_Margin;
   --  Outer bound for all world X-coordinates.  Shared by World_X subtype.

   Max_World_Y : constant Float := Terrain_Half_Height + GPS_Position_Margin;
   --  Outer bound for all world Y-coordinates.  Shared by World_Y subtype.

   subtype World_X is Float range -Max_World_X .. Max_World_X;
   --  World-frame X coordinate (metres).  Used by GPS_Fix_Type, Waypoint_Type,
   --  Estimation.State, EKF reset pose, and Set_Estimated_Position.

   subtype World_Y is Float range -Max_World_Y .. Max_World_Y;
   --  World-frame Y coordinate (metres).  Same sharing as World_X.

   Max_Heading : constant Float := Float (Ada.Numerics.Pi);
   --  Upper bound for the Heading subtype (= π rad).

   subtype Heading is Float range -Max_Heading .. Max_Heading;
   --  Heading normalised to (−π, π].  Maintained by Normalise_Angle after
   --  every Predict/Update call.

   -----------
   -- Timer --
   -----------

   type Time is new Interfaces.Unsigned_64;
   Ticks_Per_Second : constant := 1_000_000;

   function Clock return Time with
     Pre => Initialized,
     Global => (HW_State, HW_Init),
     Volatile_Function;
   --  Monotonic clock running at Ticks_Per_Seconds

   function Milliseconds
     (T : Unsigned_32)
      return Time
   is ((Ticks_Per_Second / 1_000) * Time (T));
   --  Return the number of Time ticks per milliseconds

   procedure Delay_Microseconds (Us : Unsigned_16)
     with
       Pre => Initialized,
       Global => (HW_State, HW_Init);

   procedure Delay_Milliseconds (Ms : Unsigned_16)
     with
       Pre => Initialized,
       Global => (HW_State, HW_Init);

   -----------
   -- Sonar --
   -----------

   function Sonar_Distance return Unsigned_32
     with
       Pre    => Initialized,
       Post   => Get_Sonar_Distance = Sonar_Distance'Result,
       Global => (Input => (HW_State, HW_Init),
                  In_Out => Distance_State),
       Side_Effects,
       Volatile_Function;

   function Get_Sonar_Distance return Unsigned_32
     with
      Pre    => Initialized,
      Global => (HW_Init, Distance_State),
      Ghost,
      Import;
   --  Return the value of the last Sonar Distance obtained by calling
   --  `Sonar_Distance`.

   --------------
   -- Encoders --
   --------------

   type Corner_Wheel_Id is (Front_Left, Front_Right, Rear_Left, Rear_Right)
     with Size => 8;
   --  The four driven corner wheels.  Centre wheels are cosmetic only.

   type Encoder_Ticks is new Interfaces.Integer_16;
   --  Delta wheel rotation since the last call (48 teeth = one full revolution).
   --  Forward = positive, reverse = negative.  Resets to 0 on each read.

   function Read_Encoder_Ticks (Wheel : Corner_Wheel_Id) return Encoder_Ticks
     with
       Pre           => Initialized,
       Volatile_Function,
       Side_Effects,
       Global        => (Input  => HW_Init,
                         In_Out => Encoder_State);
   --  Returns the ticks accumulated since the previous call for this wheel and
   --  resets the counter.  Latch the value into a local at the start of each
   --  control cycle; a second call in the same cycle returns 0.

   function EKF_Reset_Pending return Boolean
     with
       Pre              => Initialized,
       Volatile_Function,
       Side_Effects,
       Global           => (Input  => HW_Init,
                            In_Out => Encoder_State);
   --  Returns True once if the simulator has requested EKF re-initialisation
   --  (rover repositioned or R key pressed), then clears the flag.
   --  Consume-on-read semantics — identical threading contract to Read_Encoder_Ticks.
   --  Always returns False on real hardware.

   function EKF_Reset_X return World_X
     with Pre => Initialized, Volatile_Function,
          Global => (Input => (HW_Init, Encoder_State));
   function EKF_Reset_Y return World_Y
     with Pre => Initialized, Volatile_Function,
          Global => (Input => (HW_Init, Encoder_State));
   function EKF_Reset_Theta return Heading
     with Pre => Initialized, Volatile_Function,
          Global => (Input => (HW_Init, Encoder_State));
   --  Reset pose accompanying the pending reset signal (×1_000_000 scaled on
   --  the wire; returned as Float metres/radians here).  Read these in the same
   --  Poll cycle as EKF_Reset_Pending before the flag is consumed.
   --  Return 0.0 on real hardware (the reset path is never taken).

   ---------
   -- GPS --
   ---------

   type GPS_Fix_Type is record
      X         : World_X;     --  World coordinate (metres)
      Y         : World_Y;     --  World coordinate (metres)
      Timestamp : Unsigned_32; --  ms since simulator start; unchanged between fixes
   end record;

   function GPS_Fix return GPS_Fix_Type
     with
       Pre           => Initialized,
       Volatile_Function,
       Global        => (Input => (GPS_State, HW_Init));
   --  Returns the most recent GPS fix.  Ada detects a fresh fix by comparing
   --  Timestamp against the previously seen value (no side effects on read).

   ---------
   -- IMU --
   ---------

   type Gyro_Rate_Raw is new Interfaces.Integer_16;
   --  Signed angular rate in units of (1/8192) rad/s.
   --  Range: ±4.0 rad/s  (covers the rover's ~3 rad/s max yaw rate).
   --  Resolution: 1/8192 ≈ 0.000122 rad/s ≈ 0.007 °/s.

   Gyro_Rate_Scale : constant Float := 8192.0;
   --  Conversion factor: Float (Read_IMU_Gyro_Z) / Gyro_Rate_Scale → rad/s.
   --  Rust: simulator/src/controller.rs, GYRO_SCALE = 8192.0 LSB/(rad/s).
   --  Must remain equal to the Rust constant.

   Gyro_Max_Rate_Rad_S : constant Float := 4.0;
   --  Maximum angular rate in rad/s, matching the Gyro_Rate_Raw ±Integer_16
   --  range.  Rust: simulator/src/controller.rs, GYRO_SCALE = 8192.0; FSR
   --  ±250°/s = ±4.363 rad/s; Integer_16 caps positive side at
   --  32767/8192 ≈ 3.9998 rad/s, negative side at −32768/8192 = −4.0 rad/s.
   --  4.0 used as a symmetric bound; conservative by < 0.01% on positive side.
   --  If GYRO_SCALE changes in Rust, update Gyro_Rate_Scale and this constant.

   subtype Gyro_Rate_Rad_S is Float
     range -Gyro_Max_Rate_Rad_S .. Gyro_Max_Rate_Rad_S;
   --  Angular rate as delivered to Ada after the Integer_16 → Float conversion.

   function Read_IMU_Gyro_Z return Gyro_Rate_Raw
     with
       Pre           => Initialized,
       Volatile_Function,
       Global        => (Input => (HW_Init, IMU_State));
   --  Returns the most recent gyroscope Z-axis angular rate (yaw rate).
   --  No side effects on read — Ada always gets the latest value.
   --  Convert to rad/s with To_Rad_S below.
   --  Returns 0 on real hardware until an IMU is wired up.

   function To_Rad_S (Raw : Gyro_Rate_Raw) return Gyro_Rate_Rad_S is
     (Float (Raw) / Gyro_Rate_Scale);
   --  Convert a raw gyro reading to rad/s.  The result's subtype bound
   --  (Gyro_Rate_Rad_S = ±4.0 rad/s) is established at this declaration
   --  and inherited at every call site; callers do not need to re-prove
   --  the scaling bound.  Gyro_Rate_Raw'First / Gyro_Rate_Scale = -4.0 is
   --  exact in IEEE 754; Gyro_Rate_Raw'Last / Gyro_Rate_Scale ≈ 3.9999 ≤ 4.0.

   ---------------
   -- Waypoints --
   ---------------

   type Waypoint_Type is record
      X : World_X;  --  World coordinate (metres)
      Y : World_Y;  --  World coordinate (metres)
   end record;

   function Waypoint_Count return Natural;
   --  Number of waypoints loaded at simulator startup; 0 when none were provided.
   --  Always 0 on real hardware.

   function Get_Waypoint (Idx : Natural) return Waypoint_Type
     with Pre => Idx < Waypoint_Count;
   --  Return the Idx-th waypoint (zero-based).

   ----------
   -- Mast --
   ----------

   type Mast_Angle is new  Interfaces.Integer_8 range -70 .. 100;

   procedure Set_Mast_Angle (V : Mast_Angle)
     with
       Pre  => Initialized,
       Post => Initialized,
       Global => (Input  => HW_Init,
                  Output => HW_State);

   ------------
   -- Remote --
   ------------

   type Buttons is (Up, Down, Left, Right,
                    A, B, C, D,
                    L1, L2, R1, R2,
                    Sel, Start);

   type Buttons_State is array (Buttons) of Boolean;
   --  True if the button is pressed

   function Update return Buttons_State
     with
       Pre  => Initialized,
       Global => (HW_State, HW_Init),
       Side_Effects,
       Volatile_Function;

   ------------
   -- Wheels --
   ------------

   type Side_Id is (Left, Right)
     with Size => 8;

   for Side_Id use (Left => 0,
                    Right => 1);

   type Wheel_Id is (Front, Back, Mid)
     with Size => 8;

   for Wheel_Id use (Front => 0,
                     Back  => 1,
                     Mid   => 2);

   subtype Steering_Wheel_Id is Wheel_Id range Front .. Back;

   Max_Steering_Angle_Deg : constant := 40;
   --  Maximum steering angle in integer degrees.
   --  Rust: simulator/src/domain/robot.rs, local_wheel_position — wheel
   --  steering limit is ±40°.  If the Rust steering range changes, update
   --  this constant; Steering_Wheel_Angle below and Max_Steering_Angle_Rad
   --  in Rover.Estimation both update automatically.

   type Steering_Wheel_Angle is new Interfaces.Integer_8
     range -Max_Steering_Angle_Deg .. Max_Steering_Angle_Deg;

   procedure Set_Wheel_Angle (Wheel : Steering_Wheel_Id;
                              Side  : Side_Id;
                              V     : Steering_Wheel_Angle)
     with
       Pre  => Initialized,
       Post => Initialized,
       Global => (HW_State, HW_Init);

   type Turn_Kind is (Straight, Left, Right, Around)
     with Size => 8;
   for Turn_Kind use (Straight => 0,
                      Left => 1,
                      Right => 2,
                      Around => 3);

   procedure Set_Turn (Turn : Turn_Kind)
     with
       Pre  => Initialized,
       Post => Initialized and then
               Get_Turn = Turn,
       Global => (Input  => (HW_State, HW_Init),
                  In_Out => Turn_State);

   function Get_Turn return Turn_Kind
     with
       Pre => Initialized,
       Global => (HW_Init, Turn_State),
       Ghost,
       Import;
   --  Return the value set in the last call to `Set_Turn`.

   type Motor_Power is new Interfaces.Integer_8 range -100 .. 100;

   pragma Unevaluated_Use_Of_Old (Allow);
   procedure Set_Power (Side : Side_Id;
                        Pwr  : Motor_Power)
     with
       Pre  => Initialized,
       Post => Initialized and then
               Get_Power (Side) = Pwr and then

               (if Side = Left then
                  Get_Power (Right) = Get_Power (Right)'Old
                else
                  Get_Power (Left) = Get_Power (Left)'Old),
       Global => (Input  => (HW_State, HW_Init),
                  In_Out => Power_State);
   pragma Unevaluated_Use_Of_Old (Error);

   function Get_Power (Side : Side_Id) return Motor_Power
     with
       Pre => Initialized,
       Global => (HW_Init, Power_State),
       Ghost,
       Import;
   --  Return the value set in the last call to `Set_Power`.

   ------------------------
   -- Estimated Position --
   ------------------------

   procedure Set_Estimated_Position (X : World_X; Y : World_Y; Theta : Heading)
     with
       Pre    => Initialized,
       Global => (Input  => HW_Init,
                  Output => HW_State);
   --  Push the EKF estimated position and heading (radians, normalised to
   --  (−π, π]) to the simulator for visualisation.
   --  No-op on hardware until a telemetry channel is added.

   procedure Report_GNC_State
     (GPS_X, GPS_Y,
      Enc_FL, Enc_FR, Enc_RL, Enc_RR : Interfaces.Integer_32)
     with
       Pre    => Initialized,
       Global => (Input  => HW_Init,
                  Output => HW_State);
   --  Push last-seen GPS position (×1_000_000 integer format) and cumulative
   --  encoder tick totals to the Rust simulator console display.
   --  No-op on hardware until a telemetry channel is added.

   -------------
   -- Display --
   -------------

   Display_Text_Length : constant := 128 / 7;

   procedure Set_Display_Info (Str : String)
     with Pre => Str'Length <= Display_Text_Length;

end Rover_HAL;
