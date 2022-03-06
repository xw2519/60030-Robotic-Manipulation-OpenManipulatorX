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

clear;
clf;

THETA_1_ARRAY = [-90 -45 0 45 90];
THETA_2_ARRAY = [-30 0 30 60];
THETA_3_ARRAY = [-10 0 10];
THETA_4_ARRAY = [-5 0 5];

%% --- Plot settings
arm_plot = plot3([0 0], [0 0], [0 0]);
title(['Forward Kinematics Demostration']);

xlabel('X');
ylabel('Y'); 
zlabel('Z');

hold on;

pause(8);

iteration_count = 0;

for theta_1_index = 1:5

    for theta_2_index = 1:4
        for theta_3_index = 1:3
            for theta_4_index = 1:3

                if iteration_count >= 1
                    %% --- Remove previous arm plot (only works in second loop)
                    delete(arm_plot);
                    delete(BASE_ROTATION_TM_X);
                    delete(BASE_ROTATION_TM_Y);
                    delete(BASE_ROTATION_TM_Z);
                    delete(SHOULDER_TM_X);
                    delete(SHOULDER_TM_Y);
                    delete(SHOULDER_TM_Z);
                    delete(ELBOW_TM_X);
                    delete(ELBOW_TM_Y);
                    delete(ELBOW_TM_Z);
                    delete(WRIST_TM_X);
                    delete(WRIST_TM_Y);
                    delete(WRIST_TM_Z);
                    delete(EE_TM_X);
                    delete(EE_TM_Y);
                    delete(EE_TM_Z);
                end

                %% --- Constants 
                THETA_1 = THETA_1_ARRAY(theta_1_index);
                THETA_2 = THETA_2_ARRAY(theta_2_index);
                THETA_3 = THETA_3_ARRAY(theta_3_index);
                THETA_4 = THETA_4_ARRAY(theta_4_index);

                %% --- Plot settings
                grid on
                axis([-0.25 0.5 -0.5 0.5 0 0.5])

                set(gcf, 'Pointer', 'custom', 'PointerShapeCData', NaN(16,16)) % Hides the mouse cursor

                % Annotations
                a = gca; 
                a.Position(3) = 0.6;
                delete(findall(gcf,'type','annotation'))

                str = {['\theta_1 = ',num2str(THETA_1)],['\theta_2 = ',num2str(THETA_2)],['\theta_3 = ',num2str(THETA_3)],['\theta_4 = ',num2str(THETA_4)]};
                annotation('textbox', [0.75, 0.6, 0.1, 0.1], 'String', str, 'FontSize',18)

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
                arm_plot = plot3(ARM_COORDINATES_X, ARM_COORDINATES_Y, ARM_COORDINATES_Z, '-ok');

                plot3([0 0], [0 0], [0 0.077], '-ok'); % Add base stem

                % Plot frames
                plot3([0, 0.021],  [0, 0],     [0, 0],     '-r', 'LineWidth',2);
                plot3([0, 0],      [0, 0.021], [0, 0],     '-g', 'LineWidth',2);
                plot3([0, 0],      [0, 0],     [0, 0.021], '-b', 'LineWidth',2);

                BASE_ROTATION_TM_X = plot3(F_1_X(1, :), F_1_X(2, :), F_1_X(3, :), '-r', 'LineWidth',2);
                BASE_ROTATION_TM_Y = plot3(F_1_Y(1, :), F_1_Y(2, :), F_1_Y(3, :), '-g', 'LineWidth',2);
                BASE_ROTATION_TM_Z = plot3(F_1_Z(1, :), F_1_Z(2, :), F_1_Z(3, :), '-b', 'LineWidth',2);

                SHOULDER_TM_X = plot3(F_2_X(1, :), F_2_X(2, :), F_2_X(3, :), '-r', 'LineWidth',2);
                SHOULDER_TM_Y = plot3(F_2_Y(1, :), F_2_Y(2, :), F_2_Y(3, :), '-g', 'LineWidth',2);
                SHOULDER_TM_Z = plot3(F_2_Z(1, :), F_2_Z(2, :), F_2_Z(3, :), '-b', 'LineWidth',2);

                ELBOW_TM_X = plot3(F_3_X(1, :), F_3_X(2, :), F_3_X(3, :), '-r', 'LineWidth',2);
                ELBOW_TM_Y = plot3(F_3_Y(1, :), F_3_Y(2, :), F_3_Y(3, :), '-g', 'LineWidth',2);
                ELBOW_TM_Z = plot3(F_3_Z(1, :), F_3_Z(2, :), F_3_Z(3, :), '-b', 'LineWidth',2);

                WRIST_TM_X = plot3(F_4_X(1, :), F_4_X(2, :), F_4_X(3, :), '-r', 'LineWidth',2);
                WRIST_TM_Y = plot3(F_4_Y(1, :), F_4_Y(2, :), F_4_Y(3, :), '-g', 'LineWidth',2);
                WRIST_TM_Z = plot3(F_4_Z(1, :), F_4_Z(2, :), F_4_Z(3, :), '-b', 'LineWidth',2);

                EE_TM_X = plot3(F_5_X(1, :), F_5_X(2, :), F_5_X(3, :), '-r', 'LineWidth',2);
                EE_TM_Y = plot3(F_5_Y(1, :), F_5_Y(2, :), F_5_Y(3, :), '-g', 'LineWidth',2);
                EE_TM_Z = plot3(F_5_Z(1, :), F_5_Z(2, :), F_5_Z(3, :), '-b', 'LineWidth',2);

                % Update coordinate
                % scatter3(BASE_ROTATION_TM(1), BASE_ROTATION_TM(2), BASE_ROTATION_TM(3), 'MarkerEdgeColor', 'm', 'Marker', 'o');
                % scatter3(SHOULDER_TM(1), SHOULDER_TM(2), SHOULDER_TM(3), 'MarkerEdgeColor', 'm', 'Marker', 'o');
                % scatter3(ELBOW_TM(1), ELBOW_TM(2), ELBOW_TM(3), 'MarkerEdgeColor', 'm', 'Marker', 'o');
                % scatter3(WRIST_TM(1), WRIST_TM(2), WRIST_TM(3), 'MarkerEdgeColor', 'm', 'Marker', 'o');
                scatter3(EE_TM(1), EE_TM(2), EE_TM(3), 'MarkerEdgeColor', 'r', 'Marker', 'o');

                pause(0.25);

                iteration_count = iteration_count + 1;
            end
        end
    end
end