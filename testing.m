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


hold on
scatter(final_x, final_y)
set(gca, 'YDir','reverse')
axis([0 0.200 0.05 0.3])