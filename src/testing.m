arm = openManipX();
position_control_mode(arm);
toggle_torque(arm, 1);

set_servo_speed_limits(arm, 1300);

write_raw_encoder(arm, 14, 2048);
pause(2)
write_raw_encoder(arm, 13, 2048);
pause(2)
write_raw_encoder(arm, 12, 2048);
pause(2)

% move to position
write_raw_encoder(arm, 11, 2048);
pause(2)
write_raw_encoder(arm, 12, 2357);
pause(2)
write_raw_encoder(arm, 13, 1710.77);
pause(2)

open_gripper(arm);
pause(2)

write_raw_encoder(arm, 14, 3125);
pause(2)

close_gripper(arm);
pause(2)


delete(arm);