%% --------------------------------------------------------------------- %%
% task_1_b.m
%
% Task 1 - Modelling the Robot 
%
% Create a graphical simulation of the robot, based on your DH Table. This 
% only needs to consist of line() objects in Matlab. Increasing the 
% thickness of the line can make the simulation easier to see, particularly 
% after video compression. 
 
% Plot the co-ordinate frames (as I do in class, with appropriate colours) 
% for additional marks. 
 
% Run forward kinematics by incrementing joint angles and plotting the 
% robot. This is a good way to ensure that your DH parameters make sense. 
%% --------------------------------------------------------------------- %%

clf;
clear;

%% --- Constants 
THETA_1 = 0;
THETA_2 = 117.08;
THETA_3 = -85.32;
THETA_4 = -38.416;

%% --- Plot settings
arm_plot = plot3([0 0], [0 0], [0 0]);
title(['Forward Kinematics Arm Simulation']);

xlabel('X');
ylabel('Y'); 
zlabel('Z');

hold on;

pause(2);

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
plot3(ARM_COORDINATES_X, ARM_COORDINATES_Y, ARM_COORDINATES_Z, '-ok');

plot3([0 0], [0 0], [0 0.077], '-ok'); % Add base stem

% Plot frames
plot3([0, 0.021],  [0, 0],     [0, 0],     '-r', 'LineWidth',2);
plot3([0, 0],      [0, 0.021], [0, 0],     '-g', 'LineWidth',2);
plot3([0, 0],      [0, 0],     [0, 0.021], '-b', 'LineWidth',2);

plot3(F_1_X(1, :), F_1_X(2, :), F_1_X(3, :), '-r', 'LineWidth',2);
plot3(F_1_Y(1, :), F_1_Y(2, :), F_1_Y(3, :), '-g', 'LineWidth',2);
plot3(F_1_Z(1, :), F_1_Z(2, :), F_1_Z(3, :), '-b', 'LineWidth',2);

plot3(F_2_X(1, :), F_2_X(2, :), F_2_X(3, :), '-r', 'LineWidth',2);
plot3(F_2_Y(1, :), F_2_Y(2, :), F_2_Y(3, :), '-g', 'LineWidth',2);
plot3(F_2_Z(1, :), F_2_Z(2, :), F_2_Z(3, :), '-b', 'LineWidth',2);

plot3(F_3_X(1, :), F_3_X(2, :), F_3_X(3, :), '-r', 'LineWidth',2);
plot3(F_3_Y(1, :), F_3_Y(2, :), F_3_Y(3, :), '-g', 'LineWidth',2);
plot3(F_3_Z(1, :), F_3_Z(2, :), F_3_Z(3, :), '-b', 'LineWidth',2);

plot3(F_4_X(1, :), F_4_X(2, :), F_4_X(3, :), '-r', 'LineWidth',2);
plot3(F_4_Y(1, :), F_4_Y(2, :), F_4_Y(3, :), '-g', 'LineWidth',2);
plot3(F_4_Z(1, :), F_4_Z(2, :), F_4_Z(3, :), '-b', 'LineWidth',2);

plot3(F_5_X(1, :), F_5_X(2, :), F_5_X(3, :), '-r', 'LineWidth',2);
plot3(F_5_Y(1, :), F_5_Y(2, :), F_5_Y(3, :), '-g', 'LineWidth',2);
plot3(F_5_Z(1, :), F_5_Z(2, :), F_5_Z(3, :), '-b', 'LineWidth',2);


%% --- Plot settings
grid on
axis([-0.2 0.5 -0.5 0.5 0 0.5])