%% --------------------------------------------------------------------- %%
% task_1_d.m
%
% Task 1 - Modelling the Robot 
%
% Re-run your simulation with the inverse kinematics.  
 
% Get your robot to trace a square of 10 x 10 cm in each cartesian plane. 
% Be sure to draw the square (use line objects of different colours). 
%% --------------------------------------------------------------------- %%
clf;
clear;

M(96) = struct('cdata',[],'colormap',[]);
fig = gcf;

ARM_PLOT = plot3([0 0], [0 0], [0 0]);
title(['Inverse Kinematics Demostration']);

xlabel('X');
ylabel('Y'); 
zlabel('Z');

grid on
axis([0 0.5 -0.5 0.5 0 0.5])
pbaspect([1 1 1])

%% --- Constants

THETA_1 = 0;
THETA_2 = 0;
THETA_3 = 0;
THETA_4 = 0;

%% --- Plot [10 x 10 cm] square trajectory lines on 3 planes
X_COORDS_INIT = linspace(0.2739, 0.2, 15);
Y_COORDS_INIT = linspace(0, -0.1, 15);
Z_COORDS_INIT = linspace(0.2048, 0.15, 15);

line(X_COORDS_INIT,                   Y_COORDS_INIT,    Z_COORDS_INIT,   'color', '#808080', 'marker', 'o');

X_COORDS = linspace(0.2, 0.3, 20);
Y_COORDS = linspace(0.1, -0.1, 20);
Z_COORDS = linspace(0.15, 0.25, 20);

line(X_COORDS,                   Y_COORDS(1)*ones(1, 20),    Z_COORDS(1)*ones(1, 20),   'color', '#808080', 'marker', 'o');
line(X_COORDS,                   Y_COORDS(1)*ones(1, 20),    Z_COORDS(20)*ones(1, 20),  'color', '#808080', 'marker', 'o');
line(X_COORDS(1)*ones(1, 20),    Y_COORDS(1)*ones(1, 20),    Z_COORDS,                  'color', '#808080', 'marker', 'o');
line(X_COORDS(20)*ones(1, 20),   Y_COORDS(1)*ones(1, 20),    Z_COORDS,                  'color', '#808080', 'marker', 'o');

line(X_COORDS,                   Y_COORDS(20)*ones(1, 20),   Z_COORDS(1)*ones(1, 20),   'color', '#808080', 'marker', 'o');
line(X_COORDS,                   Y_COORDS(20)*ones(1, 20),   Z_COORDS(20)*ones(1, 20),  'color', '#808080', 'marker', 'o');
line(X_COORDS(1)*ones(1, 20),    Y_COORDS(20)*ones(1, 20),   Z_COORDS,                  'color', '#808080', 'marker', 'o');
line(X_COORDS(20)*ones(1, 20),   Y_COORDS(20)*ones(1, 20),   Z_COORDS,                  'color', '#808080', 'marker', 'o');

line(X_COORDS(1)*ones(1, 20),    Y_COORDS,                   Z_COORDS(1)*ones(1, 20),   'color', '#808080', 'marker', 'o');
line(X_COORDS(1)*ones(1, 20),    Y_COORDS,                   Z_COORDS(20)*ones(1, 20),  'color', '#808080', 'marker', 'o');

line(X_COORDS(20)*ones(1, 20),   Y_COORDS,                   Z_COORDS(1)*ones(1, 20),   'color', '#808080', 'marker', 'o');
line(X_COORDS(20)*ones(1, 20),   Y_COORDS,                   Z_COORDS(20)*ones(1, 20),  'color', '#808080', 'marker', 'o');

hold on

%% --- Precalculate coordinates and angles
% Calculate all location coordinates 

% Move to cube corner starting position
XYZ_COORDS = [(X_COORDS_INIT).' (Y_COORDS_INIT).' (Z_COORDS_INIT).'];

% Starting from cube corner
XYZ_COORDS = [XYZ_COORDS ; flip([(X_COORDS(1)*ones(1, 20)).' (Y_COORDS).' (Z_COORDS(1)*ones(1, 20)).']);         ]; 
XYZ_COORDS = [XYZ_COORDS ; (X_COORDS(1)*ones(1, 20)).' (Y_COORDS(1)*ones(1, 20)).'  (Z_COORDS).'                 ];
XYZ_COORDS = [XYZ_COORDS ; (X_COORDS(1)*ones(1, 20)).' (Y_COORDS).'                 (Z_COORDS(20)*ones(1, 20)).' ];
XYZ_COORDS = [XYZ_COORDS ; flip([(X_COORDS(1)*ones(1, 20)).' (Y_COORDS(20)*ones(1, 20)).' (Z_COORDS).'])         ];

% Feed coordinate matrix into IK to find all angles and package into matrix
THETA_MATRIX = [0 0 0 0];

for i=1:length(XYZ_COORDS)
    [THETA_1, THETA_2, THETA_3, THETA_4] = IK(XYZ_COORDS(i,1), XYZ_COORDS(i,2), XYZ_COORDS(i,3));
    
    THETA_MATRIX = [THETA_MATRIX ; [THETA_1, THETA_2, THETA_3, THETA_4]];
end

% Angle matrices checks
assert(length(XYZ_COORDS) == (length(THETA_MATRIX) - 1), 'Assert Failed: Coordinate length and Angles length should be the same');

%% --- Execute robotic arm based on angle matrices
for i = 1:length(THETA_MATRIX)
    if i > 1
        %% --- Remove previous arm plot (only works in second loop)
        delete(ARM_PLOT);
        delete(ROBOT_BASE);
        delete(X_1);
        delete(Y_1);
        delete(Z_1);
        delete(X_2);
        delete(Y_2);
        delete(Z_2);
        delete(X_3);
        delete(Y_3);
        delete(Z_3);
        delete(X_4);
        delete(Y_4);
        delete(Z_4);
        delete(X_5);
        delete(Y_5);
        delete(Z_5);
        delete(X_6);
        delete(Y_6);
        delete(Z_6);
    end
    
    %% --- Constants 
    THETA_1 = THETA_MATRIX(i, 1);
    THETA_2 = THETA_MATRIX(i, 2);
    THETA_3 = THETA_MATRIX(i, 3);
    THETA_4 = THETA_MATRIX(i, 4);

    %% --- Plot settings    
    % Annotations
    a = gca; 
    a.Position(3) = 0.6;
    delete(findall(gcf,'type','annotation'))
    
    str = {['\theta_1 = ',num2str(THETA_1)],['\theta_2 = ',num2str(THETA_2)],['\theta_3 = ',num2str(THETA_3)],['\theta_4 = ',num2str(THETA_4)]};
    annotation('textbox', [0.75, 0.6, 0.1, 0.1], 'String', str)

    %% --- FK: Robotic arm and coordinate frames
    [BASE_ROTATION_TM, SHOULDER_TM, ELBOW_TM, WRIST_TM, EE_TM] = FK(THETA_1, THETA_2, THETA_3, THETA_4);

    [F_1_X, F_1_Y, F_1_Z, F_2_X, F_2_Y, F_2_Z, F_3_X, F_3_Y, F_3_Z, F_4_X, F_4_Y, F_4_Z, F_5_X, F_5_Y, F_5_Z] = FK_coordinate_frames(THETA_1, THETA_2, THETA_3, THETA_4);

    %% --- Process data
    ARM_COORDINATES = [BASE_ROTATION_TM SHOULDER_TM ELBOW_TM WRIST_TM EE_TM];

    ARM_COORDINATES_X = ARM_COORDINATES(1, :);
    ARM_COORDINATES_Y = ARM_COORDINATES(2, :);
    ARM_COORDINATES_Z = ARM_COORDINATES(3, :);

    % Prepare coordinate frame coordinates
    ARM_FRAME_COORDINATE_FORMAT = [ARM_COORDINATES_X; ARM_COORDINATES_Y; ARM_COORDINATES_Z];

    F_1_X = [ARM_FRAME_COORDINATE_FORMAT(:, 1) F_1_X];
    F_1_Y = [ARM_FRAME_COORDINATE_FORMAT(:, 1) F_1_Y];
    F_1_Z = [ARM_FRAME_COORDINATE_FORMAT(:, 1) F_1_Z];

    F_2_X = [ARM_FRAME_COORDINATE_FORMAT(:, 2) F_2_X];
    F_2_Y = [ARM_FRAME_COORDINATE_FORMAT(:, 2) F_2_Y];
    F_2_Z = [ARM_FRAME_COORDINATE_FORMAT(:, 2) F_2_Z];

    F_3_X = [ARM_FRAME_COORDINATE_FORMAT(:, 3) F_3_X];
    F_3_Y = [ARM_FRAME_COORDINATE_FORMAT(:, 3) F_3_Y];
    F_3_Z = [ARM_FRAME_COORDINATE_FORMAT(:, 3) F_3_Z];

    F_4_X = [ARM_FRAME_COORDINATE_FORMAT(:, 4) F_4_X];
    F_4_Y = [ARM_FRAME_COORDINATE_FORMAT(:, 4) F_4_Y];
    F_4_Z = [ARM_FRAME_COORDINATE_FORMAT(:, 4) F_4_Z];

    F_5_X = [ARM_FRAME_COORDINATE_FORMAT(:, 5) F_5_X];
    F_5_Y = [ARM_FRAME_COORDINATE_FORMAT(:, 5) F_5_Y];
    F_5_Z = [ARM_FRAME_COORDINATE_FORMAT(:, 5) F_5_Z];

    %% --- Plot robotic arm structure
    
    ARM_PLOT = plot3(ARM_COORDINATES_X, ARM_COORDINATES_Y, ARM_COORDINATES_Z, '-ok');

    ROBOT_BASE = plot3([0 0], [0 0], [0 0.077], '-ok'); % Add base stem

    % Plot frames
    X_1 = plot3([0, 0.021],  [0, 0],     [0, 0],     '-r', 'LineWidth',2);
    Y_1 = plot3([0, 0],      [0, 0.021], [0, 0],     '-g', 'LineWidth',2);
    Z_1 = plot3([0, 0],      [0, 0],     [0, 0.021], '-b', 'LineWidth',2);

    X_2 = plot3(F_1_X(1, :), F_1_X(2, :), F_1_X(3, :), '-r', 'LineWidth',2);
    Y_2 = plot3(F_1_Y(1, :), F_1_Y(2, :), F_1_Y(3, :), '-g', 'LineWidth',2);
    Z_2 = plot3(F_1_Z(1, :), F_1_Z(2, :), F_1_Z(3, :), '-b', 'LineWidth',2);

    X_3 = plot3(F_2_X(1, :), F_2_Y(2, :), F_2_Z(3, :), '-r', 'LineWidth',2);
    Y_3 = plot3(F_2_X(1, :), F_2_Y(2, :), F_2_Z(3, :), '-g', 'LineWidth',2);
    Z_3 = plot3(F_2_X(1, :), F_2_Y(2, :), F_2_Z(3, :), '-b', 'LineWidth',2);

    X_4 = plot3(F_3_X(1, :), F_3_X(2, :), F_3_X(3, :), '-r', 'LineWidth',2);
    Y_4 = plot3(F_3_Y(1, :), F_3_Y(2, :), F_3_Y(3, :), '-g', 'LineWidth',2);
    Z_4 = plot3(F_3_Z(1, :), F_3_Z(2, :), F_3_Z(3, :), '-b', 'LineWidth',2);

    X_5 = plot3(F_4_X(1, :), F_4_X(2, :), F_4_X(3, :), '-r', 'LineWidth',2);
    Y_5 = plot3(F_4_Y(1, :), F_4_Y(2, :), F_4_Y(3, :), '-g', 'LineWidth',2);
    Z_5 = plot3(F_4_Z(1, :), F_4_Z(2, :), F_4_Z(3, :), '-b', 'LineWidth',2);

    X_6 = plot3(F_5_X(1, :), F_5_X(2, :), F_5_X(3, :), '-r', 'LineWidth',2);
    Y_6 = plot3(F_5_Y(1, :), F_5_Y(2, :), F_5_Y(3, :), '-g', 'LineWidth',2);
    Z_6 = plot3(F_5_Z(1, :), F_5_Z(2, :), F_5_Z(3, :), '-b', 'LineWidth',2);
    
    % Record movies
    M(i) = getframe(gcf);
    
    pause(0.25);
end

v = VideoWriter('newfile.avi');
open(v);
writeVideo(v,M);
close(v);