% Tests the gripper functionality

% Test summary:
% 1. Open and close gripper

arm = openManipX();
position_control_mode(arm);
toggle_torque(arm, 1);

set_all_servo_speed_limits(arm, 40);

% Test write raw encoder functions
logger(mfilename, "Test Log: Testing gripper functionality")

open_gripper(arm);
pause(2);
close_gripper(arm);
pause(2);

delete(arm);