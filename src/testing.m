arm = openManipX();
position_control_mode(arm);
toggle_torque(arm, 1);

% 11 -> 15 joints at different positions

% min -> max = 0 -> 4096

set_servo_speed_limits(arm, 2000);

%Getting base position
write(arm, 14, 2048);
pause(2)
write(arm, 13, 2048);  
pause(2)
write(arm, 12, 2048);  
pause(2)
write(arm, 11, 2048);

pause(2);

%Getting into cube position
write(arm, 11, 1194.666667);
pause(2)
write(arm, 12, 1945.029344);  
pause(2)
write(arm, 14, 2329.366756); 
pause(2)
write(arm, 13, 2465.5719);   
pause(2)

pause(2)

write(arm, 13, 2383);
write(arm, 12, 1528);
write(arm, 11, 1528);

pause(2)


delete(arm);