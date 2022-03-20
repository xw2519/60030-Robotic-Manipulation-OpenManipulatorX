%% --- trajectoryLib --- %%

% This script sets up and handles all trajectory related functionalities

classdef trajectoryLib
    %% --- Class variables --- %%
    properties(Constant)
        % Acrylic board coordinates 
        ACRYLIC_BOARD_Z_COORDS = [ 0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198,    0,      0,      0,      0,      0,   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198;
                                   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198,    0,      0,      0,      0,      0,   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198;
                                   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198,    0,      0,      0,      0,      0,   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198;
                                   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198,    0,      0,      0,      0,      0,   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198;
                                   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198,    0,      0,      0,      0,      0,   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198;
                                   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198;
                                   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198;
                                   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198;
                                   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198;
                                   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198;
                                   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198;
                                   0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198,  0.0198, 0.0198,  0.0198,  0.0198,  0.0198,  0.0198; ];
    
        ACRYLIC_BOARD_Y_COORDS = [ -0.2000,  -0.1750,  -0.1500,  -0.1250,  -0.1000,  -0.0750,    0,      0,      0,      0,      0,    0.0750, 0.1000, 0.1250, 0.1500, 0.1750, 0.2000;
                                   -0.2000,  -0.1750,  -0.1500,  -0.1250,  -0.1000,  -0.0750,    0,      0,      0,      0,      0,    0.0750, 0.1000, 0.1250, 0.1500, 0.1750, 0.2000;
                                   -0.2000,  -0.1750,  -0.1500,  -0.1250,  -0.1000,  -0.0750,    0,      0,      0,      0,      0,    0.0750, 0.1000, 0.1250, 0.1500, 0.1750, 0.2000;
                                   -0.2000,  -0.1750,  -0.1500,  -0.1250,  -0.1000,  -0.0750,    0,      0,      0,      0,      0,    0.0750, 0.1000, 0.1250, 0.1500, 0.1750, 0.2000;
                                   -0.2000,  -0.1750,  -0.1500, -0.1250,  -0.1000,  -0.0750,    0,      0,      0,      0,      0,    0.0750, 0.1000, 0.1250, 0.1500, 0.1750, 0.2000;
                                   -0.2000,  -0.1750, -0.1500,  -0.1250,  -0.1000,  -0.0750, -0.0500, -0.0250,    0,  0.0250, 0.0500, 0.0750, 0.1000, 0.1250, 0.1500, 0.1750, 0.2000;
                                   -0.2000,  -0.1750,  -0.1000,  -0.1250,  -0.1000,  -0.0750, -0.0500, -0.0250,    0,  0.0250, 0.0500, 0.0750, 0.1000, 0.1250, 0.1500, 0.1750, 0.2000;
                                   -0.2000,  -0.1750,  -0.1500,  -0.1250,  -0.1000,  -0.0750, -0.0500, -0.0250,    0,  0.0250, 0.0500, 0.0750, 0.1000, 0.1250, 0.1500, 0.1750, 0.2000;
                                   -0.2000,  -0.1750,  -0.1500,  -0.1250,  -0.1000,  -0.0750, -0.0500, -0.0250,    0,  0.0250, 0.0500, 0.0750, 0.1000, 0.1250, 0.1500, 0.1750, 0.2000;
                                   -0.2000,  -0.1750,  -0.1500,  -0.1250,  -0.1000,  -0.0750, -0.0500, -0.0250,    0,  0.0250, 0.0500, 0.0750, 0.1000, 0.1250, 0.1500, 0.1750, 0.2000;
                                   -0.2000,  -0.1750,  -0.1500,  -0.1250,  -0.1000,  -0.0750, -0.0500, -0.0250,    0,  0.0250, 0.0500, 0.0750, 0.1000, 0.1250, 0.1500, 0.1750, 0.2000;
                                   -0.2000,  -0.1750,  -0.1500,  -0.1250,  -0.1000,  -0.0750, -0.0500, -0.0250,    0,  0.0250, 0.0500, 0.0750, 0.1000, 0.1250, 0.1500, 0.1750, 0.2000; ];

        ACRYLIC_BOARD_X_COORDS = [ -0.0500, -0.0500, -0.0500, -0.0500, -0.0500, -0.0500,   0,      0,      0,      0,      0,    -0.0500, -0.0500, -0.0500, -0.0500, -0.0500, -0.0500;
                                   -0.0250, -0.0250, -0.0250, -0.0250, -0.0250, -0.0250,   0,      0,      0,      0,      0,    -0.0250, -0.0250, -0.0250, -0.0250, -0.0250, -0.0250;
                                      0,       0,       0,       0,       0,       0,      0,      0,      0,      0,      0,       0,       0,       0,       0,       0,       0,  ;  
                                   0.0250,  0.0250,  0.0250,  0.0250,  0.0250,  0.0250,    0,      0,      0,      0,      0,     0.0250,  0.0250,  0.0250,  0.0250,  0.0250,  0.0250;
                                   0.0500,  0.0500,  0.0500,  0.0500,  0.0500,  0.0500,    0,      0,      0,      0,      0,     0.0500,  0.0500,  0.0500,  0.0500,  0.0500,  0.0500;
                                   0.0750,  0.0750,  0.0750,  0.0750,  0.0750,  0.0750, 0.0750, 0.0750, 0.0750, 0.0750, 0.0750,   0.0750,  0.0750,  0.0750,  0.0750,  0.0750,  0.0750;
                                   0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000, 0.1000, 0.1000, 0.1000, 0.1000, 0.1000,   0.1000,  0.1000,  0.1000,  0.1000,  0.1000,  0.1000;
                                   0.1250,  0.1250,  0.1250,  0.1250,  0.1250,  0.1250, 0.1250, 0.1250, 0.1250, 0.1250, 0.1250,   0.1250,  0.1250,  0.1250,  0.1250,  0.1250,  0.1250;
                                   0.1500,  0.1500,  0.1500,  0.1500,  0.1500,  0.1500, 0.1500, 0.1500, 0.1500, 0.1500, 0.1500,   0.1500,  0.1500,  0.1500,  0.1500,  0.1500,  0.1500;
                                   0.1750,  0.1750,  0.1750,  0.1750,  0.1750,  0.1750, 0.1750, 0.1750, 0.1750, 0.1750, 0.1750,   0.1750,  0.1750,  0.1750,  0.1750,  0.1750,  0.1750;
                                   0.2000,  0.2000,  0.2000,  0.2000,  0.2000,  0.2000, 0.2000, 0.2000, 0.2000, 0.2000, 0.2000,   0.2000,  0.2000,  0.2000,  0.2000,  0.2000,  0.2000;
                                   0.2250,  0.2250,  0.2250,  0.2250,  0.2250,  0.2250, 0.2250, 0.2250, 0.2250, 0.2250, 0.2250,   0.2250,  0.2250,  0.2250,  0.2250,  0.2250,  0.2250; ];
    end
    
    properties(GetAccess=private)
        THETA_1_CALIBRATION = 30
        THETA_2_CALIBRATION = 0
        THETA_3_CALIBRATION = 0
        THETA_4_CALIBRATION = 0
    end
    
    %% --- Static Class methods --- %%
    methods(Static)
        %% --- Converters --- %%
        function [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = convert_to_servo_angles(THETA_1, THETA_2, THETA_3, THETA_4)
            SERVO_THETA_1 = (180-THETA_1); % (90+THETA_1);
            SERVO_THETA_2 = (180+(THETA_2-10.61965528));
            SERVO_THETA_3 = (180+(-THETA_3-79.38034472));
            SERVO_THETA_4 = (180-THETA_4);
        end

        function SERVO_ANGLE_ARRAY_CONVERTED = convert_to_servo_angles_array(SERVO_ANGLE_ARRAY)
            SERVO_ANGLE_ARRAY_CONVERTED(length(SERVO_ANGLE_ARRAY), 4) = 0;
            
            for i = 1:length(SERVO_ANGLE_ARRAY)
                SERVO_THETA_1 = (180-SERVO_ANGLE_ARRAY(i, 1)); % (90+SERVO_ANGLE_ARRAY(i, 1));
                SERVO_THETA_2 = (180+(SERVO_ANGLE_ARRAY(i, 2)-10.61965528));
                SERVO_THETA_3 = (180+(-SERVO_ANGLE_ARRAY(i, 3)-79.38034472));
                SERVO_THETA_4 = (180-SERVO_ANGLE_ARRAY(i, 4));
                SERVO_ANGLE_ARRAY_CONVERTED(i) = [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4];
            end
        end
        
        %% --- FK and IK --- %%
        function [P_X, P_Y, P_Z] = FK(SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4)
            % Convert to FK angle format
            THETA_1 = -(180-SERVO_THETA_1);
            THETA_2 = 180-SERVO_THETA_2;
            THETA_3 = 180-SERVO_THETA_3;
            THETA_4 = 180-SERVO_THETA_4;
            
            % Feed angles into FK matrices
            %% --- Waist TM --- %%
            T_0_1 = [   cosd(THETA_1)               -sind(THETA_1)          0           0;
                        sind(THETA_1)*cosd(0)       cosd(THETA_1)*cosd(0)   -sind(0)    -sind(0)*0.077;
                        sind(THETA_1)*sind(0)       cosd(THETA_1)*sind(0)   cosd(0)     cosd(0)*0.077;
                        0                           0                       0           1
                    ];

            %% --- Wrist TM --- %%
            T_1_2 = [   cosd(THETA_2 + 90 - 10.6)                    -sind(THETA_2 + 90 - 10.6)               0           0;
                        sind(THETA_2 + 90 - 10.6)*cosd(90)           cosd(THETA_2 + 90 - 10.6)*cosd(90)       -sind(90)   -sind(90)*0;
                        sind(THETA_2 + 90 - 10.6)*sind(90)           cosd(THETA_2 + 90 - 10.6)*sind(90)       cosd(90)    cosd(90)*0;
                        0                                            0                                        0           1
                    ];

            %% --- Shoulder TM --- %%

            T_2_3 = [   cosd(THETA_3 - 90 + 10.6)               -sind(THETA_3 - 90 + 10.6)          0           0.130;
                        sind(THETA_3 - 90 + 10.6)*cosd(0)      cosd(THETA_3 - 90 + 10.6)*cosd(0)    -sind(0)    -sind(0)*0;
                        sind(THETA_3 - 90 + 10.6)*sind(0)      cosd(THETA_3 - 90 + 10.6)*sind(0)    cosd(0)     cosd(0)*0;
                        0                                       0                                   0           1
                    ];

            %% --- Back Edge TM --- %%
            T_3_4 = [   cosd(THETA_4)                      -sind(THETA_4)          0           0.124;
                        sind(THETA_4)*cosd(0)              cosd(THETA_4)*cosd(0)   -sind(0)    -sind(0)*0;
                        sind(THETA_4)*sind(0)              cosd(THETA_4)*sind(0)   cosd(0)     cosd(0)*0;
                        0                                       0                                   0           1
                    ];

            %% --- Elbow TM --- %%
            T_4_5 = [   cosd(0)                    -sind(0)               0           0.126;
                        sind(0)*cosd(0)            cosd(0)*cosd(0)        -sind(0)    -sind(0)*0;
                        sind(0)*sind(0)            cosd(0)*sind(0)        cosd(0)     cosd(0)*0;
                        0                                0                            0           1
                    ];

            EE_TM = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5;
            
            % Extract X, Y and Z coordinates
            P_X = EE_TM(1,4);
            P_Y = EE_TM(2,4);
            P_Z = EE_TM(3,4);
        end
        
        %% --- Operation Functions --- %%
        function draw_line(START_COORDS, END_COORDS)
            ...
        end

        function draw_arc(ARC_CENTER, RADIUS, ~, START_COORDS, END_COORDS)
            ...
        end
    end
    
    %% --- Non-static Class methods --- %%
    methods
        %% --- FK and IK --- %%
        
        function [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = IK_with_PHI_draw(obj, P_X, P_Y, P_Z, PHI)
            constant = atand(0.024/0.128);
            constant = 90-constant;

            % Constants
            A_2 = 0.130;
            A_3 = 0.124;
            A_4 = 0.116;

            SIGN = -1;

            P_R = abs(sqrt(P_X^2 + P_Y^2));
            R_3 = P_R;
            Z_3 = P_Z - 0.077;

            R_2 = R_3 - A_4*cosd(PHI);
            Z_2 = Z_3 - A_4*sind(PHI);

            % Calculate angles
            % Theta 1 
            if ~(P_X<=0) && ~(P_Y<=0)  
                RAW_THETA_1 = atand(P_Y/P_X);
            elseif (P_X<0) && ~(P_Y<0) 
                RAW_THETA_1 = atand(P_Y/P_X) + 180;
            elseif (P_X<0) && (P_Y<0) 
                RAW_THETA_1 = atand(P_Y/P_X) - 180;  
            else
                RAW_THETA_1 = atand(P_Y/P_X);
            end

            % Theta 3 
            ARGUMENT = ((R_2^2 + Z_2^2 - (A_2^2 + A_3^2))/(2*A_2*A_3));
            RAW_THETA_3 = SIGN*acosd(ARGUMENT);

            % Theta 2
            COSINE_ARGUMENT_1 = (((A_2 + A_3*cosd(RAW_THETA_3))*Z_2) - ((A_3*sind(RAW_THETA_3))*R_2))/ (R_2^2 + Z_2^2);
            SINE_ARGUMENT_1 = (((A_2 + A_3*cosd(RAW_THETA_3))*R_2) + ((A_3*sind(RAW_THETA_3))*Z_2))/ (R_2^2 + Z_2^2);
            RAW_THETA_2 = atand(SINE_ARGUMENT_1/COSINE_ARGUMENT_1);

            % Theta 4
            RAW_THETA_4 = PHI - RAW_THETA_3 - 90 + RAW_THETA_2;
            
            % Converting angles into format required by servo
            if(imag(RAW_THETA_1) < 10^-5 && imag(RAW_THETA_2) < 10^-5 && imag(RAW_THETA_3) < 10^-5 && imag(RAW_THETA_4) < 10^-5)
                SERVO_THETA_1 = 180 + RAW_THETA_1;
                SERVO_THETA_2 = 180 - (-1*(RAW_THETA_2 - (90 - constant)));
                SERVO_THETA_3 = 180 - (RAW_THETA_3 + constant);
                SERVO_THETA_4 = 180 - RAW_THETA_4;  
            else
                assert("Unreachable");
            end
        end
        
        
        function [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = IK_with_PHI_stick(obj, P_X, P_Y, P_Z, PHI)
            constant = atand(0.024/0.128);
            constant = 90-constant;

            % Constants
            A_2 = 0.130;
            A_3 = 0.124;
            A_4 = 0.116;

            SIGN = -1;

            P_R = abs(sqrt(P_X^2 + P_Y^2));
            R_3 = P_R;
            Z_3 = P_Z - 0.077;

            R_2 = R_3 - A_4*cosd(PHI);
            Z_2 = Z_3 - A_4*sind(PHI);

            % Calculate angles
            % Theta 1 
            if ~(P_X<=0) && ~(P_Y<=0)  
                RAW_THETA_1 = atand(P_Y/P_X);
            elseif (P_X<0) && ~(P_Y<0) 
                RAW_THETA_1 = atand(P_Y/P_X) + 180;
            elseif (P_X<0) && (P_Y<0) 
                RAW_THETA_1 = atand(P_Y/P_X) - 180;  
            else
                RAW_THETA_1 = atand(P_Y/P_X);
            end

            % Theta 3 
            ARGUMENT = ((R_2^2 + Z_2^2 - (A_2^2 + A_3^2))/(2*A_2*A_3));
            RAW_THETA_3 = SIGN*acosd(ARGUMENT);

            % Theta 2
            COSINE_ARGUMENT_1 = (((A_2 + A_3*cosd(RAW_THETA_3))*Z_2) - ((A_3*sind(RAW_THETA_3))*R_2))/ (R_2^2 + Z_2^2);
            SINE_ARGUMENT_1 = (((A_2 + A_3*cosd(RAW_THETA_3))*R_2) + ((A_3*sind(RAW_THETA_3))*Z_2))/ (R_2^2 + Z_2^2);
            RAW_THETA_2 = atand(SINE_ARGUMENT_1/COSINE_ARGUMENT_1);

            % Theta 4
            RAW_THETA_4 = PHI - RAW_THETA_3 - 90 + RAW_THETA_2;
            
            % Converting angles into format required by servo
            if(imag(RAW_THETA_1) < 10^-5 && imag(RAW_THETA_2) < 10^-5 && imag(RAW_THETA_3) < 10^-5 && imag(RAW_THETA_4) < 10^-5)
                SERVO_THETA_1 = 180 + RAW_THETA_1;
                SERVO_THETA_2 = 180 - (-1*(RAW_THETA_2 - (90 - constant)));
                SERVO_THETA_3 = 180 - (RAW_THETA_3 + constant);
                SERVO_THETA_4 = 180 - RAW_THETA_4;  
            else
                assert("Unreachable");
            end
        end
        
        
        function [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = IK_with_PHI_tip(obj, P_X, P_Y, P_Z, PHI)
            constant = atand(0.024/0.128);
            constant = 90-constant;

            % Constants
            A_2 = 0.130;
            A_3 = 0.124;
            A_4 = 0.126 + 0.02;

            SIGN = -1;

            P_R = abs(sqrt(P_X^2 + P_Y^2));
            R_3 = P_R;
            Z_3 = P_Z - 0.077;

            R_2 = R_3 - A_4*cosd(PHI);
            Z_2 = Z_3 - A_4*sind(PHI);

            % Calculate angles
            % Theta 1 
            if ~(P_X<=0) && ~(P_Y<=0)  
                RAW_THETA_1 = atand(P_Y/P_X);
            elseif (P_X<0) && ~(P_Y<0) 
                RAW_THETA_1 = atand(P_Y/P_X) + 180;
            elseif (P_X<0) && (P_Y<0) 
                RAW_THETA_1 = atand(P_Y/P_X) - 180;  
            else
                RAW_THETA_1 = atand(P_Y/P_X);
            end

            % Theta 3 
            ARGUMENT = ((R_2^2 + Z_2^2 - (A_2^2 + A_3^2))/(2*A_2*A_3));
            RAW_THETA_3 = SIGN*acosd(ARGUMENT);

            % Theta 2
            COSINE_ARGUMENT_1 = (((A_2 + A_3*cosd(RAW_THETA_3))*Z_2) - ((A_3*sind(RAW_THETA_3))*R_2))/ (R_2^2 + Z_2^2);
            SINE_ARGUMENT_1 = (((A_2 + A_3*cosd(RAW_THETA_3))*R_2) + ((A_3*sind(RAW_THETA_3))*Z_2))/ (R_2^2 + Z_2^2);
            RAW_THETA_2 = atand(SINE_ARGUMENT_1/COSINE_ARGUMENT_1);

            % Theta 4
            RAW_THETA_4 = PHI - RAW_THETA_3 - 90 + RAW_THETA_2;
            
            % Converting angles into format required by servo
            if(imag(RAW_THETA_1) < 10^-5 && imag(RAW_THETA_2) < 10^-5 && imag(RAW_THETA_3) < 10^-5 && imag(RAW_THETA_4) < 10^-5)
                SERVO_THETA_1 = 180 + RAW_THETA_1;
                SERVO_THETA_2 = 180 - (-1*(RAW_THETA_2 - (90 - constant)));
                SERVO_THETA_3 = 180 - (RAW_THETA_3 + constant);
                SERVO_THETA_4 = 180 - RAW_THETA_4;  
            else
                assert("Unreachable");
            end
        end
        
        function [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = IK_with_PHI(obj, P_X, P_Y, P_Z, PHI)
            constant = atand(0.024/0.128);
            constant = 90-constant;

            % Constants
            A_2 = 0.130;
            A_3 = 0.124;
            A_4 = 0.126;

            SIGN = -1;

            P_R = abs(sqrt(P_X^2 + P_Y^2));
            R_3 = P_R;
            Z_3 = P_Z - 0.077;

            R_2 = R_3 - A_4*cosd(PHI);
            Z_2 = Z_3 - A_4*sind(PHI);

            % Calculate angles
            % Theta 1 
            if ~(P_X<=0) && ~(P_Y<=0)  
                RAW_THETA_1 = atand(P_Y/P_X);
            elseif (P_X<0) && ~(P_Y<0) 
                RAW_THETA_1 = atand(P_Y/P_X) + 180;
            elseif (P_X<0) && (P_Y<0) 
                RAW_THETA_1 = atand(P_Y/P_X) - 180;  
            else
                RAW_THETA_1 = atand(P_Y/P_X);
            end

            % Theta 3 
            ARGUMENT = ((R_2^2 + Z_2^2 - (A_2^2 + A_3^2))/(2*A_2*A_3));
            RAW_THETA_3 = SIGN*acosd(ARGUMENT);

            % Theta 2
            COSINE_ARGUMENT_1 = (((A_2 + A_3*cosd(RAW_THETA_3))*Z_2) - ((A_3*sind(RAW_THETA_3))*R_2))/ (R_2^2 + Z_2^2);
            SINE_ARGUMENT_1 = (((A_2 + A_3*cosd(RAW_THETA_3))*R_2) + ((A_3*sind(RAW_THETA_3))*Z_2))/ (R_2^2 + Z_2^2);
            RAW_THETA_2 = atand(SINE_ARGUMENT_1/COSINE_ARGUMENT_1);

            % Theta 4
            RAW_THETA_4 = PHI - RAW_THETA_3 - 90 + RAW_THETA_2;
            
            % Converting angles into format required by servo
            if(imag(RAW_THETA_1) < 10^-5 && imag(RAW_THETA_2) < 10^-5 && imag(RAW_THETA_3) < 10^-5 && imag(RAW_THETA_4) < 10^-5)
                SERVO_THETA_1 = 180 + RAW_THETA_1;
                SERVO_THETA_2 = 180 - (-1*(RAW_THETA_2 - (90 - constant)));
                SERVO_THETA_3 = 180 - (RAW_THETA_3 + constant);
                SERVO_THETA_4 = 180 - RAW_THETA_4 - 0.3;  
            else
                SERVO_THETA_1 = 1000;
                SERVO_THETA_2 = 1000;
                SERVO_THETA_3 = 1000;
                SERVO_THETA_4 = 1000;
            end
        end
        
        function ANGLE_ARRAY = IK_array_with_PHI(obj, COORD_ARRAY)
            ANGLE_ARRAY = [];

            % Constants
            A_2 = 0.130;
            A_3 = 0.124;
            A_4 = 0.126;

            SIGN = -1;

            for i = 1:length(COORD_ARRAY)
                P_X = COORD_ARRAY(i,1);
                P_Y = COORD_ARRAY(i,2);
                P_Z = COORD_ARRAY(i,3);
                PHI = COORD_ARRAY(i,4);
                
                P_R = abs(sqrt(P_X^2 + P_Y^2));
                R_3 = P_R;
                Z_3 = P_Z - 0.077;
                
                R_2 = R_3 - A_4*cosd(PHI);
                Z_2 = Z_3 - A_4*sind(PHI);
            
                % Calculate angles
                % Theta 1 
                if ~(P_X<=0) && ~(P_Y<=0)  
                    RAW_THETA_1 = atand(P_Y/P_X);
                elseif (P_X<0) && ~(P_Y<0) 
                    RAW_THETA_1 = atand(P_Y/P_X) + 180;
                elseif (P_X<0) && (P_Y<0) 
                    RAW_THETA_1 = atand(P_Y/P_X) - 180;  
                else
                    RAW_THETA_1 = atand(P_Y/P_X);
                end

                % Theta 3 
                ARGUMENT = ((R_2^2 + Z_2^2 - (A_2^2 + A_3^2))/(2*A_2*A_3));

                % THETA_3 = atan2d(ARGUMENT_2, ARGUMENT);

                RAW_THETA_3 = SIGN*acosd(ARGUMENT);

                % Theta 2

                COSINE_ARGUMENT_1 = (((A_2 + A_3*cosd(RAW_THETA_3))*Z_2) - ((A_3*sind(RAW_THETA_3))*R_2))/ (R_2^2 + Z_2^2);

                SINE_ARGUMENT_1 = (((A_2 + A_3*cosd(RAW_THETA_3))*R_2) + ((A_3*sind(RAW_THETA_3))*Z_2))/ (R_2^2 + Z_2^2);

                RAW_THETA_2 = atand(SINE_ARGUMENT_1/COSINE_ARGUMENT_1);

                % Theta 4
                RAW_THETA_4 = PHI - RAW_THETA_3 - 90 + RAW_THETA_2;

                % Converting angles into format required by servo
                if(imag(RAW_THETA_1) < 10^-5 && imag(RAW_THETA_2) < 10^-5 && imag(RAW_THETA_3) < 10^-5 && imag(RAW_THETA_4) < 10^-5)
                    SERVO_THETA_1 = real((180 - RAW_THETA_1)+obj.THETA_1_CALIBRATION);
                    SERVO_THETA_2 = real((180 + (RAW_THETA_2-10.61965528)) + obj.THETA_2_CALIBRATION);
                    SERVO_THETA_3 = real((180+(-RAW_THETA_3-79.38034472)) + obj.THETA_3_CALIBRATION);
                    SERVO_THETA_4 = real((180-RAW_THETA_4) + obj.THETA_4_CALIBRATION);
                else
                    assert("Unreachable");
                end

                % Store into array
                ANGLE_ARRAY = [ANGLE_ARRAY; [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4]];
            end
        end
        
        %% --- Acrylic board functions --- %%
        function [P_X, P_Y, P_Z] = get_board_location(obj, ROW, COLUMN, FORWARD_OFFSET, Z_OFFSET)
            P_X = obj.ACRYLIC_BOARD_X_COORDS(ROW, COLUMN);
            P_Y = obj.ACRYLIC_BOARD_Y_COORDS(ROW, COLUMN);
            P_Z = obj.ACRYLIC_BOARD_Z_COORDS(ROW, COLUMN) + Z_OFFSET;
            degree = atand(P_Y/P_X);
            
            y_offset = (FORWARD_OFFSET) * sind(degree);
            x_offset = (FORWARD_OFFSET) * cosd(degree);
            P_X = P_X + x_offset;
            P_Y = P_Y + y_offset;
        end        
    end
end