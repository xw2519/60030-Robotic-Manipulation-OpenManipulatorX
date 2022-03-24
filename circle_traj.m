%Start Coordinates
x_start = 0.175; %60mm
y_start = 0.100; %200mm
z_start = 0.115; %120mm

%Coordinates of circle center
x = 0.175; %175mm
y = 0.175; %175mm

%Degree of circle
deg = 270;
r = 0.025; %25mm radius

h_l = 0.10;%length of horizontal line
v_l = 0.10;%length of vertical line

final_x = [];
final_y = [];

%% draw vertical line
yunit = 0.10:0.001:0.20;
xunit = x_start+0*ones(size(yunit));

final_x=[final_x xunit];
final_y=[final_y yunit];

%% draw diagonal line
yunit = linspace(0.200,0.150,60);
xunit = linspace(0.175,0.125,60);

final_x=[final_x xunit];
final_y=[final_y yunit];

%% draw horizontal line
xunit = linspace(0.125,0.175,60);
yunit = linspace(0.150,0.150,60);
final_x=[final_x xunit];
final_y=[final_y yunit];

%% draw circle
rad = pi - deg2rad(deg);
th = linspace(pi,rad,50); % for thetas from 0 to 2pi with step pi/50 (small steps)
xunit = flip(r * cos(th) + x);
yunit = flip(r * sin(th) + y);

disp(yunit);

final_x=[final_x xunit];
final_y=[final_y yunit];

THETA_MATRIX = [0 10.6388 -79.2528 -0.1084];
RAW_THETA_MATRIX = [0 10.6388 -79.2528 -0.1084];
disp("Final x:");
disp(final_x);
disp("Final y:");
disp(final_y);

%instantiate the arm object
arm = openManipX();
trajectoryLib = trajectoryLib();
position_control_mode(arm);
toggle_torque(arm, 1);
set_all_servo_speed_limits(arm, 50);

[P_X, P_Y, P_Z] = trajectoryLib.get_board_location(9, 5, 0, 0);

pick_up_pen(arm, trajectoryLib, P_X, P_Y, 0.125, 0.002, 0, 0);

[RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4] = trajectoryLib.IK_with_PHI_draw(final_y(1), final_x(1), final_z(1) + 0.01, 0);
write_angles_to_all_servos(arm, RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4)
pause(1);
[RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4] = trajectoryLib.IK_with_PHI_draw(final_y(1), final_x(1), final_z(1), 0);
write_angles_to_all_servos(arm, RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4)
pause(1);

for i = 1:length(final_x)
    [RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4] = trajectoryLib.IK_with_PHI_draw(final_y(i), final_x(i), final_z(i), 0);
    %THETA_MATRIX = [THETA_MATRIX ; [THETA_1, THETA_2, THETA_3, THETA_4]];

    disp(RAW_THETA_MATRIX);
    write_angles_to_all_servos(arm, RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4)
    %close_grupper();
end
pause(1);
[RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4] = trajectoryLib.IK_with_PHI_draw(final_y(length(final_x)), final_x(length(final_x)), final_z(1) + 0.02, 0);
write_angles_to_all_servos(arm, RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4);
pause(1);
disp(RAW_THETA_MATRIX);
write_angles_to_all_servos(arm, 180, 180, 180, RAW_OPT_T4);

drop_pen(arm, trajectoryLib, P_X, P_Y, 0.125, 0.002, 0, 0);


%rotated_x = (final_x-90).*cos(rot) - (final_y-90).*sin(rot);
%rotated_y = (final_x-90).*sin(rot) + (final_y-90).*cos(rot);
%hold on
%plot(rotated_x+90, rotated_y+90)
%axis([0 200 0 175])

delete(arm);