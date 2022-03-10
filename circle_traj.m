%Start Coordinates
x_start = 0.0317 ;
y_start = 0.26; %100mm
z_start = 0.12; %120mm
%Coordinates of circle center
x = 0.16; %160mm
y = 0.26; %100mm
%z = height of pen;
%Degree of circle
deg = 270;
%Value of radius
r = 0.02; %20mm radius

h_l = 0.02;%length of horizontal line
v_l = 0.03;%length of vertical line

final_x = [];
final_y = [];
%% draw horizontal line%%
xunit = x_start:0.001:x_start+h_l;
yunit = y_start*ones(size(xunit));
final_x=[final_x xunit];
final_y=[final_y yunit];
%% draw vertical line
yunit = y_start:0.001:y_start+v_l;
xunit = x_start+h_l*ones(size(yunit));
final_x=[final_x xunit];
final_y=[final_y yunit];
%% draw diagonal line
yunit = [y_start+v_l:-0.001:y_start];
xunit = x_start+h_l:0.001:x_start+h_l+0.001*size(yunit,2)-0.001;
final_x=[final_x xunit];
final_y=[final_y yunit];
%% draw horizontal line%%
xunit = x_start+h_l+size(yunit,2)-1:0.001:x-r;
yunit = y_start*ones(size(xunit));
final_x=[final_x xunit];
final_y=[final_y yunit];
%% draw circle
rad = pi-deg2rad(deg);
th = linspace(pi,rad,100); %for thetas from 0 to 2pi with step pi/50 (small steps)
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
final_x=[final_x xunit];
final_y=[final_y yunit];
%zunit = z;
%% draw horizontal line%%
xunit = [x:-0.001:x_start];
yunit = (y-r)*ones(size(xunit));
final_x=[final_x xunit];
final_y=[final_y yunit];
final_z= z_start*ones(size(final_x));
hold on
plot(final_x, final_y)
axis([0 0.200 0 0.175])

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
set_all_servo_speed_limits(arm, 38);

for i = 1:length(final_x)
    [THETA_1, THETA_2, THETA_3, THETA_4, phi, RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4] = IK_Optimal(final_y(i),final_x(i),final_z(i),RAW_THETA_MATRIX(i,1),RAW_THETA_MATRIX(i,2),RAW_THETA_MATRIX(i,3),RAW_THETA_MATRIX(i,4));
    THETA_MATRIX = [THETA_MATRIX ; [THETA_1, THETA_2, THETA_3, THETA_4]];
    RAW_THETA_MATRIX = [RAW_THETA_MATRIX ; [RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4]];
    disp(RAW_THETA_MATRIX);
    
    SERVO_THETA_1 = real((180-RAW_THETA_MATRIX(i,1))+1.41);
    SERVO_THETA_2 = real((180+(RAW_THETA_MATRIX(i,2)-10.61965528)));
    SERVO_THETA_3 = real((180+(-RAW_THETA_MATRIX(i,3)-79.38034472)));
    SERVO_THETA_4 = real((180-RAW_THETA_MATRIX(i,4)));
    
    write_angles_to_all_servos(arm, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4)
    pause(0.5);
end
disp(RAW_THETA_MATRIX);

%rotated_x = (final_x-90).*cos(rot) - (final_y-90).*sin(rot);
%rotated_y = (final_x-90).*sin(rot) + (final_y-90).*cos(rot);
%hold on
%plot(rotated_x+90, rotated_y+90)
%axis([0 200 0 175])
