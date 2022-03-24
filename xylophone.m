arm = openManipX();
trajectoryLib = trajectoryLib();

position_control_mode(arm);
toggle_torque(arm, 1);
set_all_servo_speed_limits(arm, 30);
set_all_servo_acceleration_limits(arm, 14);

[P_X, P_Y, P_Z] = trajectoryLib.get_board_location(7, 5, 0, 0);

arm.close_gripper_stick();
arm.write_angles_to_all_servos(180, 180, 180, 180);


%pick_up_stick(arm, trajectoryLib, P_X, P_Y, 0.115);

hit_1_faster(arm, trajectoryLib, 0, 0, 0);
drums(arm, trajectoryLib, 0, 0, 0);
hit_1(arm, trajectoryLib, 0, 0, 0);
bow(arm, trajectoryLib, 0, 0, 0);
%hit_2(arm);
%hit_3(arm);
%hit_4(arm);
%hit_5(arm);
%hit_6(arm);

delete(arm);