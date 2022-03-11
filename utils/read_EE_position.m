% Infinitely loops and reads the position of the tool

arm = openManipX();
trajectoryLib = trajectoryLib();

position_control_mode(arm);
toggle_torque(arm, 0); % Switch of Torque Mode

while 1
   % Read servo positions
   [ID11_Angle, ID12_Angle, ID13_Angle, ID14_Angle, ID15_Angle] = read_all_servo_angles(arm);
   fprintf('Test Log: Theta 1(Deg): %.4f - Theta 2(Deg): %.4f - Theta 3(Deg): %.4f - Theta 4(Deg): %.4f\n', typecast(uint32(SERVO_THETA_1), 'int32'), typecast(uint32(SERVO_THETA_2), 'int32'), typecast(uint32(SERVO_THETA_3), 'int32'), typecast(uint32(SERVO_THETA_4), 'int32'));
   
   % Pass into FK
   [P_X, P_Y, P_Z] = FK(SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
   fprintf('Test Log: X: %.4f - Y: %.4f - Z: %.4f \n', P_X, P_Y, P_Z);
end
