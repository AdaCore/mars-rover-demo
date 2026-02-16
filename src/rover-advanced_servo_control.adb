with Rover_HAL; use Rover_HAL;

package body Rover.Advanced_Servo_Control with SPARK_Mode is

   ---------------
   -- Set_Speed --
   ---------------

   procedure Set_Speed (This : in out Instance;
                        Speed : Degree_Per_Seconds)
   is
   begin
      This.Speed := Speed;
   end Set_Speed;

   ----------
   -- Scan --
   ----------

   procedure Scan (This     : in out Instance;
                   Min, Max :        Angle_Type)
   is
   begin
      This.Move_To (Min, Wait_For_Completion => True);
      This.Direction := Right;
      This.Scan_Min := Min;
      This.Scan_Max := Max;
      This.Mode := Scan_Mode;
   end Scan;

   -------------
   -- Move_To --
   -------------

   procedure Move_To (This                : in out Instance;
                      Angle               :        Angle_Type;
                      Wait_For_Completion :        Boolean := False)
   is
   begin
      This.Fixed_Target := Angle;
      This.Mode := Fixed_Mode;

      if Wait_For_Completion then
         while This.Angle /= This.Fixed_Target loop
            This.Update;
            Rover_HAL.Delay_Milliseconds (10);
         end loop;
      end if;
   end Move_To;

   ------------
   -- Update --
   ------------

   procedure Update (This : in out Instance) is
      Now     : constant Time := Clock;
      Elapsed : constant Time := Now - This.Last_Update;

      Max_Motion_F : constant Float :=
        (Float (Elapsed) / Float (Ticks_Per_Second)) * Float (This.Speed);

      Max_Motion : constant Mast_Angle :=
        Mast_Angle'Max (1,
                        Mast_Angle
                          (Float'Min (Max_Motion_F, Float (Mast_Angle'Last))));

      subtype S32 is Integer_32;

      Diff : S32;
      New_Angle : Angle_Type;

   begin

      case This.Mode is
         when Fixed_Mode =>
            if This.Angle < This.Fixed_Target then
               Diff := S32 (This.Fixed_Target) - S32 (This.Angle);
               Diff := S32'Min (Diff, S32 (Max_Motion));
               New_Angle := This.Angle + Angle_Type (Diff);
            elsif This.Angle > This.Fixed_Target then
               Diff := S32 (This.Angle) - S32 (This.Fixed_Target);
               Diff := S32'Min (Diff, S32 (Max_Motion));
               New_Angle := This.Angle - Angle_Type (Diff);
            else
               New_Angle := This.Angle;
            end if;

         when Scan_Mode =>
            case This.Direction is
               when Left =>
                  Diff := S32 (This.Angle) - S32 (This.Scan_Min);
                  Diff := S32'Max (0, S32'Min (Diff, S32 (Max_Motion)));
                  New_Angle := This.Angle - Angle_Type (Diff);

                  if New_Angle <= This.Scan_Min then
                     This.Direction := Right;
                  end if;

               when Right =>
                  Diff := S32 (This.Scan_Max) - S32 (This.Angle);
                  Diff := S32'Max (0, S32'Min (Diff, S32 (Max_Motion)));
                  New_Angle := This.Angle + Angle_Type (Diff);

                  if New_Angle >= This.Scan_Max then
                     This.Direction := Left;
                  end if;
            end case;
      end case;

      This.Angle := New_Angle;
      Set_Angle (This.Angle);
      This.Last_Update := Now;
   end Update;

   ----------------
   -- Last_Angle --
   ----------------

   function Current_Angle (This : Instance) return Angle_Type
   is (This.Angle);

end Rover.Advanced_Servo_Control;
