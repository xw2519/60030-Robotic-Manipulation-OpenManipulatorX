% Test picking up and dropping cube

% Test summary:
% 1. Go to location
% 2. Grip cube 
% 3. Return to starting position 
% 4. Go to another location 
% 5. Grip cube 
% 6. Move to another location 
% 7. Drop cube

arm = openManipX();
trajectoryLib = trajectoryLib();

position_control_mode(arm);
toggle_torque(arm, 1);
set_all_servo_speed_limits(arm, 35);

% Step 1: Go to location
[P_X, P_Y, P_Z] = trajectoryLib.get_board_location(11, 9);
fprintf('Test Log: X: %.4f - Y: %.4f - Z: %.4f \n', P_X, P_Y, P_Z);

[SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, (P_Z+0.025), -90);
fprintf('Test Log: Theta 1(Deg): %.4f - Theta 2(Deg): %.4f - Theta 3(Deg): %.4f - Theta 4(Deg): %.4f\n', typecast(uint32(SERVO_THETA_1), 'int32'), typecast(uint32(SERVO_THETA_2), 'int32'), typecast(uint32(SERVO_THETA_3), 'int32'), typecast(uint32(SERVO_THETA_4), 'int32'));

write_angles_to_all_servos(arm, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, (SERVO_THETA_4+0.0096)); % Move to directly above cube
pause(3);
open_gripper(arm);

write_angles_to_all_servos(arm, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, (SERVO_THETA_4)); % Move arm downward
pause(3);
close_gripper(arm);
pause(3);

% Move to another place


% Step 2: Release and return to position
open_gripper(arm);
pause(2);
write_angles_to_all_servos(arm, 180.0, 180.0, 180.0, 180.0);
pause(3);


delete(arm);