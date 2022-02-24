arm = openManipX();
position_control_mode(arm);
toggle_torque(arm, 1);

set_servo_speed_limits(arm, 2000);

write(arm, 13, 2383);
write(arm, 12, 1528);
write(arm, 11, 1528);

pause(2)


delete(arm);