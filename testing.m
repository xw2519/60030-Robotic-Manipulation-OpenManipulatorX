[SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(0.38, 0, 0.077, 0);

disp("Servo Theta 1");
disp(SERVO_THETA_1);
disp("Servo Theta 2");
disp(SERVO_THETA_2);
disp("Servo Theta 3");
disp(SERVO_THETA_3);
disp("Servo Theta 4");
disp(SERVO_THETA_4);


% Test summary:
% 1. Move robot to starting default position using raw encoder values
% 2. Move robot to position to pick up cube using raw encoder values
% 3. Open and close cube
% 4. Move robot back to default position using angles
% 5. Open and close gripper

arm = openManipX();
trajectoryLib = trajectoryLib();
position_control_mode(arm);
toggle_torque(arm, 1);

set_all_servo_speed_limits(arm, 35);
set_all_servo_acceleration_limits(arm, 5);

write_angles_to_all_servos(arm, 135, 218, 238, 86); 


