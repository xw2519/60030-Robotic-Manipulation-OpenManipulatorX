arm = openManipX();
position_control_mode(arm);
toggle_torque(arm, 1);

set_all_servo_speed_limits(arm, 1300);

write_raw_encoder(arm, 14, 2048);
pause(2)
write_raw_encoder(arm, 13, 2048);
pause(2)
write_raw_encoder(arm, 12, 2048);
pause(2)

% move to position
write_raw_encoder(arm, 11, 2045); % 180 deg
pause(2)
write_raw_encoder(arm, 12, 2511); % 220.968 deg
pause(2)
write_raw_encoder(arm, 13, 1466); % 129.008 deg
pause(2)

open_gripper(arm);
pause(2)

write_raw_encoder(arm, 14, 1129.43); % 99.39 deg
pause(2)


% 11 -> 15 joints at different positions

% min -> max = 0 -> 4096

% set_servo_speed_limits(arm, 2000);

% Getting base position
% write(arm, 14, 2048);
% pause(2)
% write(arm, 13, 2048);  
% pause(2)
% write(arm, 12, 2048);  
% pause(2)
% write(arm, 11, 2048);

% pause(2);

% Getting into cube position
% write(arm, 11, 1194.666667); % 102 deg 
% pause(2)
% write(arm, 12, 2186.684871); % 192.43 deg
% pause(2) 
% write(arm, 14, 2329.366756); % 204 deg
% pause(2)
% write(arm, 13, 2465.5719); % 216 deg
% pause(2)

% write(arm, 13, 2383);
% write(arm, 12, 1528);
% write(arm, 11, 1528);

close_gripper(arm);
pause(2)


delete(arm);