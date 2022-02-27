function [BASE_ROTATION_TM, SHOULDER_TM, ELBOW_TM, WRIST_TM, EE_TM] = FK(THETA_1, THETA_2, THETA_3, THETA_4)
    % Constants 
    constant = atand(0.024/0.128);
    constant = 90 - constant;

    % Base rotation TM 
    T_0_1 = [   cosd(THETA_1)                       -sind(THETA_1)                      0           0;
                sind(THETA_1)*cosd(0)               cosd(THETA_1)*cosd(0)               -sind(0)    -sind(0)*0.077;
                sind(THETA_1)*sind(0)               cosd(THETA_1)*sind(0)               cosd(0)     cosd(0)*0.077;
                0                                   0                                   0           1
            ];

    % Shoulder TM
    T_1_2 = [   cosd(THETA_2 + constant)            -sind(THETA_2 + constant)           0           0;
                sind(THETA_2 + constant)*cosd(90)   cosd(THETA_2 + constant)*cosd(90)   -sind(90)   -sind(90)*0;
                sind(THETA_2 + constant)*sind(90)   cosd(THETA_2 + constant)*sind(90)   cosd(90)    cosd(90)*0;
                0                                   0                                   0           1
            ];

    % Elbow TM 

    T_2_3 = [   cosd(THETA_3 - constant)            -sind(THETA_3 - constant)           0           0.130;
                sind(THETA_3 - constant)*cosd(0)    cosd(THETA_3 - constant)*cosd(0)    -sind(0)    -sind(0)*0;
                sind(THETA_3 - constant)*sind(0)    cosd(THETA_3 - constant)*sind(0)    cosd(0)     cosd(0)*0;
                0                                   0                                   0           1
            ];

    % Wrist TM 
    T_3_4 = [   cosd(THETA_4)                       -sind(THETA_4)                      0           0.124;
                sind(THETA_4)*cosd(0)               cosd(THETA_4)*cosd(0)               -sind(0)    -sind(0)*0;
                sind(THETA_4)*sind(0)               cosd(THETA_4)*sind(0)               cosd(0)     cosd(0)*0;
                0                                   0                                   0           1
            ];

    % EE TM 
    T_4_5 = [   cosd(0)                             -sind(0)                            0           0.126;
                sind(0)*cosd(0)                     cosd(0)*cosd(0)                     -sind(0)    -sind(0)*0;
                sind(0)*sind(0)                     cosd(0)*sind(0)                     cosd(0)     cosd(0)*0;
                0                                   0                                   0           1
            ];
        
    BASE_ROTATION_TM = T_0_1;
    BASE_ROTATION_TM = BASE_ROTATION_TM([1,2,3], 4);
    
    SHOULDER_TM      = T_0_1*T_1_2;
    SHOULDER_TM = SHOULDER_TM([1,2,3], 4);
    
    ELBOW_TM         = T_0_1*T_1_2*T_2_3;
    ELBOW_TM = ELBOW_TM([1,2,3], 4);
    
    WRIST_TM         = T_0_1*T_1_2*T_2_3*T_3_4;
    WRIST_TM = WRIST_TM([1,2,3], 4);
    
    EE_TM            = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5;
    EE_TM = EE_TM([1,2,3], 4);
end