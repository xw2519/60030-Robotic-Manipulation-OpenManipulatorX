arm = openManipX();
trajectoryLib = trajectoryLib();

% Setting up
position_control_mode(arm);
toggle_torque(arm, 1);
set_all_servo_speed_limits(arm, 50);
set_all_servo_acceleration_limits(arm, 25);

set_servo_speed_limit(arm, 15, 420);
set_servo_acceleration(arm, 15, 150);

write_angles_to_all_servos(arm, 180.0, 180.0, 180.0, 180.0);

% Get all positions to go to
[P_X1, P_Y1, P_Z1] = trajectoryLib.get_board_location(12, 9, 0, 0);
[P_X2_2, P_Y2_2, P_Z2_2] = trajectoryLib.get_board_location(8, 4, 0, 0);
[P_X2, P_Y2, P_Z2] = trajectoryLib.get_board_location(8, 4, 0, 0);
[P_X3, P_Y3, P_Z3] = trajectoryLib.get_board_location(6, 1, 0, 0);
[P_X4, P_Y4, P_Z4] = trajectoryLib.get_board_location(3, 13, 0, 0);
[P_X5, P_Y5, P_Z5] = trajectoryLib.get_board_location(9, 15, 0, 0);
[P_X6, P_Y6, P_Z6] = trajectoryLib.get_board_location(7, 9, 0, 0);

% Offsets
constant = 0.001;

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
%rotate_cube_forward_at_coord(arm, trajectoryLib, P_X1, P_Y1, (P_Z1+0.005-constant+0.008), (rotate_drop_x_offset+0.001), (rotate_drop_z_offset+0.020), -0.0025);
%rotate_cube_forward_at_coord(arm, trajectoryLib, P_X1, P_Y1, (P_Z1+0.025-constant+0.005), (rotate_drop_x_offset+0.002), (rotate_drop_z_offset+0.009), -0.0035);

% Rotate cube 2
%rotate_cube_forward_at_coord(arm, trajectoryLib, P_X3, P_Y3, (P_Z3+0.025-constant+0.005), (rotate_drop_x_offset+0.002), (rotate_drop_z_offset+0.009), -0.0012);

% Rotate cube 3
rotate_cube_forward_at_coord(arm, trajectoryLib, P_X5, P_Y5, (P_Z5+0.025-constant-0.009), (rotate_drop_x_offset+0.002), (rotate_drop_z_offset+0.015), -0.002);

delete(arm);
