function RAW_THETA_MATRIX = draw(x_start,y_start,z_start,circle_x,circle_y,circle_deg,circle_rad,h_l,v_l)
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
rad = pi-deg2rad(circle_deg);
th = linspace(pi,rad,100); %for thetas from 0 to 2pi with step pi/50 (small steps)
xunit = circle_rad * cos(th) + circle_x;
yunit = circle_rad * sin(th) + circle_y;
final_x=[final_x xunit];
final_y=[final_y yunit];
%zunit = z;
%% draw horizontal line%%
xunit = [x:-0.001:x_start];
yunit = (y-r)*ones(size(xunit));
final_x=[final_x xunit];
final_y=[final_y yunit];
final_z=z_start*ones(size(final_x));
%% Pass through IK all the points
RAW_THETA_MATRIX = [0 10.6388 -79.2528 -0.1084];

for i = 1:length(final_x)
    [~, ~, ~, ~, ~, RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4] = IK_Optimal(final_y(i),final_x(i),final_z(i),RAW_THETA_MATRIX(i,1),RAW_THETA_MATRIX(i,2),RAW_THETA_MATRIX(i,3),RAW_THETA_MATRIX(i,4));
    RAW_THETA_MATRIX = [RAW_THETA_MATRIX ; [RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4]];
end
end

