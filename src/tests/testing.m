
arm = openManipX();
trajectoryLib = trajectoryLib();

position_control_mode(arm);
toggle_torque(arm, 1);
set_all_servo_speed_limits(arm, 35);

[P_X, P_Y, P_Z] = trajectoryLib.get_board_location(11, 9);
[SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, (P_Z+0.05), 0);
write_angles_to_all_servos(arm, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4); % Move to directly above cube
pause(3);
open_gripper(arm);

[SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, (P_Z+0.025), 0);
write_angles_to_all_servos(arm, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, (SERVO_THETA_4)); % Move arm downward
pause(3);

close_gripper(arm);
pause(3);

[SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, (P_Z+0.05), 0);
write_angles_to_all_servos(arm, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, (SERVO_THETA_4)); % Move to directly above cube
pause(3);

delete(arm);