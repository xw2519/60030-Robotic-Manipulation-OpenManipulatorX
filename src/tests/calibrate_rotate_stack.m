arm = openManipX();
trajectoryLib = trajectoryLib();

% Setting up
position_control_mode(arm);
toggle_torque(arm, 1);
set_all_servo_speed_limits(arm, 60);
set_all_servo_acceleration_limits(arm, 25);

set_servo_speed_limit(arm, 15, 420);
set_servo_acceleration(arm, 15, 150);

write_angles_to_all_servos(arm, 180.0, 180.0, 180.0, 180.0);

% Get all positions to go to
[P_X1, P_Y1, P_Z1] = trajectoryLib.get_board_location(10, 2, 0, 0);
[P_X4, P_Y4, P_Z4] = trajectoryLib.get_board_location(3, 3, 0, 0);

[P_X2, P_Y2, P_Z2] = trajectoryLib.get_board_location(10, 11, 0, 0);
[P_X6, P_Y6, P_Z6] = trajectoryLib.get_board_location(12, 9, 0, 0);

[P_X3, P_Y3, P_Z3] = trajectoryLib.get_board_location(3, 16, 0, 0);
[P_X5, P_Y5, P_Z5] = trajectoryLib.get_board_location(7, 4, 0, 0);

% Offsets
drop_constant = 0.000;

offset1 = -0.005;

rotate_drop_x_offset = -0.004;
rotate_drop_z_offset = -0.008;

x_offset1 = -0.007;
z_offset1 = -0.001;

x_offset2 = -0.009;
z_offset2 = -0.004;

drop_all_x_offset = 0.004;
drop_all_z_offset = -0.005+0.004;

% Rotate cube 1
rotate_cube_backward_at_coord(arm, trajectoryLib, P_X1, P_Y1, (P_Z1+0.025-constant+0.008), (rotate_drop_x_offset-0.000), (rotate_drop_z_offset+0.0011), -0.0025);
rotate_cube_backward_at_coord(arm, trajectoryLib, P_X1, P_Y1, (P_Z1+0.025-constant+0.005), (rotate_drop_x_offset+0.002), (rotate_drop_z_offset+0.009), -0.0035);
rotate_cube_backward_at_coord(arm, trajectoryLib, P_X1, P_Y1, (P_Z1+0.025-constant+0.005), (rotate_drop_x_offset+0.002), (rotate_drop_z_offset+0.009), -0.0035);

% Rotate cube 2
rotate_cube_backward_at_coord(arm, trajectoryLib, P_X2, P_Y2, (P_Z2+0.025-constant+0.005), (rotate_drop_x_offset+0.002), (rotate_drop_z_offset+0.009), -0.0010);
rotate_cube_backward_at_coord(arm, trajectoryLib, P_X2, P_Y2, (P_Z2+0.025-constant+0.005), (rotate_drop_x_offset+0.002), (rotate_drop_z_offset+0.009), -0.0010);

% Rotate cube 3
rotate_cube_backward_at_coord(arm, trajectoryLib, P_X5, P_Y5, (P_Z5+0.025-constant+0.006), (rotate_drop_x_offset+0.002), (rotate_drop_z_offset+0.009), -0.002);

% Move commands
move_cube_coord(arm, trajectoryLib, P_X1, P_Y1, (P_Z1+0.025+0.0098-constant+0.001), P_X2, P_Y2, (P_Z2+0.025+0.017-constant), drop_all_x_offset+0.001, drop_all_z_offset+0.002, 0);

move_cube_coord(arm, trajectoryLib, P_X3, P_Y3, (P_Z3+0.025+0.0098-constant-0.005), P_X4-0.001, P_Y4, ((P_Z4+0.025+0.017-constant)), drop_all_x_offset-0.004, (drop_all_z_offset-0.004), 0);

move_cube_coord(arm, trajectoryLib, P_X5, P_Y5, (P_Z5+0.025+0.0098-constant+0.001), P_X6, P_Y6, (P_Z6+0.025+0.017-constant), drop_all_x_offset-0.002, drop_all_z_offset-0.001, 0);

delete(arm);
