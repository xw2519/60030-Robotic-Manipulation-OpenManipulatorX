% Tests the write to all servo functionality

% Test summary:
% 1. Move robot to starting default position in sync
% 2. Move robot to position to pick up cube in sync
% 3. Open and close cube
% 4. Move robot back to default position using angles in sync
% 5. Open and close cube

arm = openManipX();
position_control_mode(arm);
toggle_torque(arm, 1);

set_all_servo_speed_limits(arm, 45);

% Test: Write raw encoder functions
logger(mfilename, "Test Log: Write to all servos")

write_angles_to_all_servos(arm, 180.0, 180.0, 180.0, 180.0);
pause(2);
write_angles_to_all_servos(arm, 180.0, 220.968, 129.008, 180.0);
pause(2);
open_gripper(arm);
pause(2);
write_angles_to_all_servos(arm, 180.0, 220.968, 129.008, 99.39);
pause(2)
close_gripper(arm);
pause(2);

% Test: Read all angles
[ID11_Angle, ID12_Angle, ID13_Angle, ID14_Angle, ID15_Angle] = read_all_servo_angles(arm);

fprintf('[ID:%03d] Angle(Deg): %.4f \n', 11, typecast(uint32(ID11_Angle), 'int32'));
fprintf('[ID:%03d] Angle(Deg): %.4f \n', 12, typecast(uint32(ID12_Angle), 'int32'));
fprintf('[ID:%03d] Angle(Deg): %.4f \n', 13, typecast(uint32(ID13_Angle), 'int32'));
fprintf('[ID:%03d] Angle(Deg): %.4f \n', 14, typecast(uint32(ID14_Angle), 'int32'));
fprintf('[ID:%03d] Angle(Deg): %.4f \n', 15, typecast(uint32(ID15_Angle), 'int32'));

delete(arm);