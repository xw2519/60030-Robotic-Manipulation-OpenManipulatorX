% Tests the accuracy of the converter

% Test summary:
% 1. Get IK of default position
% 2. Move robot to starting default position in sync
% 3. Get IK to pick up cube locations
% 4. Move robot to position to pick up cube in sync
% 5. Open and close cube
% 6. Get IK of default position
% 7. Move robot back to default position using angles in sync
% 8. Open and close cube

arm = openManipX();
trajectoryLib = trajectoryLib();

position_control_mode(arm);
toggle_torque(arm, 1);

set_all_servo_speed_limits(arm, 1300);

% Step 1
logger(mfilename, "Test Log: Get angles of default position")
[SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = IK_with_PHI(0.2740, 0, 0.2048, 0);

fprintf('Test Log: Theta 1(Deg): %.4f - Theta 2(Deg): %.4f - Theta 3(Deg): %.4f - Theta 4(Deg): %.4f\n', typecast(uint32(SERVO_THETA_1), 'int32'), typecast(uint32(SERVO_THETA_2), 'int32'), typecast(uint32(SERVO_THETA_3), 'int32'), typecast(uint32(SERVO_THETA_4), 'int32'));

assert(1 == 0, "Test Breaker 1 Triggered"); % Breaker

% Step 2
write_angles_to_all_servos(arm, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
pause(4);

% Step 3
[SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = IK_with_PHI(0.2226, 0, 0.3053 + 0.0096, 0);

fprintf('Test Log: Theta 1(Deg): %.4f - Theta 2(Deg): %.4f - Theta 3(Deg): %.4f - Theta 4(Deg): %.4f\n', typecast(uint32(SERVO_THETA_1), 'int32'), typecast(uint32(SERVO_THETA_2), 'int32'), typecast(uint32(SERVO_THETA_3), 'int32'), typecast(uint32(SERVO_THETA_4), 'int32'));

assert(1 == 0, "Test Breaker 2 Triggered"); % Breaker

% Step 4
write_angles_to_all_servos(arm, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
open_gripper(arm);
pause(4);

% Step 5
[SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = IK_with_PHI(0.2226, 0, 0.3053, 0);

fprintf('Test Log: Theta 1(Deg): %.4f - Theta 2(Deg): %.4f - Theta 3(Deg): %.4f - Theta 4(Deg): %.4f\n', typecast(uint32(SERVO_THETA_1), 'int32'), typecast(uint32(SERVO_THETA_2), 'int32'), typecast(uint32(SERVO_THETA_3), 'int32'), typecast(uint32(SERVO_THETA_4), 'int32'));

assert(1 == 0, "Test Breaker 3 Triggered"); % Breaker

% Step 6
write_angles_to_all_servos(arm, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
pause(2);
close_gripper(arm);
pause(4);

open_gripper(arm);
pause(2);

delete(arm);

