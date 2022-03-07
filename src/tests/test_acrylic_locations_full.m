% Tests locations of acrylic board locations 

% Test summary:
% 1. Fetch critical locations from acrylic board
% 2. Move robot to position to position
% 3. Loop to move robot to all acrylic board locations

arm = openManipX();
trajectoryLib = trajectoryLib();

position_control_mode(arm);
toggle_torque(arm, 1);
set_all_servo_speed_limits(arm, 1300);

for ROW = 5:12 
   for COLUMN = 1:17
        [P_X, P_Y, P_Z] = trajectoryLib.get_board_location(ROW, COLUMN);
        fprintf('Test Log: X: %.4f - Y: %.4f - Z: %.4f \n', P_X, P_Y, P_Z);
       
        [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z, 0);
        write_angles_to_all_servos(arm, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
        
        pause(4);
   end 
end