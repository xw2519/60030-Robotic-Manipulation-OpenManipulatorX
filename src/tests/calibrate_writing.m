%Start Coordinates
x_start = 0.06; %60mm
y_start = 0.20; %200mm
z_start = 0.115; %120mm
%Coordinates of circle center
x = 0.1; %160mm
y = 0.2; %100mm
%z = height of pen;
%Degree of circle
deg = -180;
%Value of radius
r = 0.04; %20mm radius

h_l = 0.08;%length of horizontal line
v_l = 0.075;%length of vertical line

final_x = [];
final_y = [];
%% draw horizontal line%%
%xunit = x_start:0.01:x_start+h_l;
%yunit = y_start*ones(size(xunit));
%final_x=[final_x xunit];
%final_y=[final_y yunit];
%% draw diagonal line
yunit = y_start:-0.00093:y_start-v_l;
xunit = x_start:0.001:x_start+h_l;
final_x=[final_x xunit];
final_y=[final_y yunit];
%% draw vertical line
yunit = y_start-v_l:0.001:y_start;
xunit = x_start+h_l*ones(size(yunit));
disp(yunit);
disp(xunit);
final_x=[final_x xunit];
final_y=[final_y yunit];
%% draw diagonal line
%yunit = [y_start+v_l:-0.01:y_start];
%xunit = x_start:0.01:x_start+0.01*size(yunit,2)-0.01;
%final_x=[final_x xunit];
%final_y=[final_y yunit];
%% draw horizontal line%%
xunit = x_start+h_l:-0.001:x_start;
yunit = y_start*ones(size(xunit));
final_x=[final_x xunit];
final_y=[final_y yunit];
%% draw circle
rad = pi-deg2rad(deg);
th = linspace(pi,rad,50); %for thetas from 0 to 2pi with step pi/50 (small steps)
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
final_x=[final_x xunit];
final_y=[final_y yunit];
%zunit = z;
%% draw horizontal line%%
%xunit = [x_start+h_l-0.01:-0.01:x_start];
%yunit = (y-r)*ones(size(xunit));
%final_x=[final_x xunit];
%final_y=[final_y yunit];
final_z= z_start*ones(size(final_x));
hold on
plot(final_x, final_y)
axis([0 0.200 0.1 0.3])

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

pick_up_pen(arm, trajectoryLib, P_X, P_Y, 0.125);

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

drop_pen(arm, trajectoryLib, P_X, P_Y, 0.125);



%rotated_x = (final_x-90).*cos(rot) - (final_y-90).*sin(rot);
%rotated_y = (final_x-90).*sin(rot) + (final_y-90).*cos(rot);
%hold on
%plot(rotated_x+90, rotated_y+90)
%axis([0 200 0 175])

delete(arm);