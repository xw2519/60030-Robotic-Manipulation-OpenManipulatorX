
arm = openManipX();
trajectoryLib = trajectoryLib();

position_control_mode(arm);
toggle_torque(arm, 1);
set_all_servo_speed_limits(arm, 30);
set_all_servo_acceleration_limits(arm, 5);

write_angles_to_all_servos(arm, 180.0, 180.0, 180.0, 180.0);

[P_X1, P_Y1, P_Z1] = trajectoryLib.get_board_location(12, 9);
[P_X2, P_Y2, P_Z2] = trajectoryLib.get_board_location(9, 15);

move_cube_coord(arm, trajectoryLib, P_X1, P_Y1, (P_Z1+0.025+0.0098), P_X2, P_Y2, (P_Z2+0.025+0.017));

delete(arm);