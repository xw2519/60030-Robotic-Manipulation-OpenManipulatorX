% Tests basic arm functions and gripper functions 

% Test summary:
% 1. Move robot to starting default position using raw encoder values
% 2. Move robot to position to pick up cube using raw encoder values
% 3. Open and close cube
% 4. Move robot back to default position using angles
% 5. Open and close gripper

arm = openManipX();
position_control_mode(arm);
toggle_torque(arm, 1);

set_all_servo_speed_limits(arm, 40);

% Test write angles functions
logger(mfilename, "Test Log: Testing write angle functions")

write_angles(arm, 14, 180); % 180 deg
pause(2);
write_angles(arm, 13, 180); % 180 deg
pause(2);
write_angles(arm, 12, 180); % 180 deg
pause(2);

% move to position
write_angles(arm, 11, 180); % 180 deg
pause(2);
write_angles(arm, 12, 156); % 220.968 deg
pause(2);
write_angles(arm, 13, 205); % 129.008 deg
pause(2);

open_gripper(arm);
pause(2);

write_angles(arm, 14, 223); % 99.39 deg
pause(2);

close_gripper(arm);
pause(2);

delete(arm);