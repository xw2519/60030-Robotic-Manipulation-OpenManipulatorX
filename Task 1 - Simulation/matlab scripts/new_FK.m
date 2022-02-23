%     Frame     alpha_{i-1}     a_{i-1}     d_i         theta_i      
% ----------------------------------------------------------------
%   1 (Waist)        0             0         0.077         0
%   2 (Shoulder)     90            0         0             90     
%   3 (Back Edge)    0             0.128     0             -90   
%   4 (Elbow)        0             0.024     0             0
%   5 (Wrist)        0             0.124     0             0
%   6 (EE)           0             0.126     0             0

%% --- Constants --- %%
THETA_3 = 0;
THETA_6 = 0;

THETA_1 = 0;
THETA_2 = 0;
THETA_4 = 0;
THETA_5 = 0;

%% --- Waist TM --- %%

T_0_1 = [   cosd(THETA_1)               -sind(THETA_1)          0           0;
            sind(THETA_1)*cosd(0)       cosd(THETA_1)*cosd(0)   -sind(0)    -sind(0)*0.077;
            sind(THETA_1)*sind(0)       cosd(THETA_1)*sind(0)   cosd(0)     cosd(0)*0.077;
            0                           0                       0           1
        ];

%% --- Shoulder TM --- %%

T_1_2 = [   cosd(THETA_2)               -sind(THETA_2)          0           0;
            sind(THETA_2)*cosd(90)      cosd(THETA_2)*cosd(90)  -sind(90)   -sind(90)*0;
            sind(THETA_2)*sind(90)      cosd(THETA_2)*sind(90)  cosd(90)    cosd(90)*0;
            0                                0                            0           1
        ];
    
%% --- Back Edge TM --- %%
T_2_3 = [   cosd(THETA_3 - 90)               -sind(THETA_3 - 90)          0           0.128;
            sind(THETA_3 - 90)*cosd(0)       cosd(THETA_3 - 90)*cosd(0)   -sind(0)    -sind(0)*0;
            sind(THETA_3 - 90)*sind(0)       cosd(THETA_3 - 90)*sind(0)   cosd(0)     cosd(0)*0;
            0                                0                            0           1
        ];
  
%% --- Elbow TM --- %%
T_3_4 = [   cosd(THETA_4 + 90)                    -sind(THETA_4 + 90)               0           0.024;
            sind(THETA_4 + 90)*cosd(0)            cosd(THETA_4 + 90)*cosd(0)        -sind(0)    -sind(0)*0;
            sind(THETA_4 + 90)*sind(0)            cosd(THETA_4 + 90)*sind(0)        cosd(0)     cosd(0)*0;
            0                                     0                                 0           1
        ];

%% --- Wrist TM --- %%
T_4_5 = [   cosd(THETA_5)                    -sind(THETA_5)               0           0.124;
            sind(THETA_5)*cosd(0)            cosd(THETA_5)*cosd(0)        -sind(0)    -sind(0)*0;
            sind(THETA_5)*sind(0)            cosd(THETA_5)*sind(0)        cosd(0)     cosd(0)*0;
            0                                0                            0           1
        ];
    
%% --- EE TM --- %%
T_5_6 = [   cosd(THETA_6)                    -sind(THETA_6)               0           0.126;
            sind(THETA_6)*cosd(0)            cosd(THETA_6)*cosd(0)        -sind(0)    -sind(0)*0;
            sind(THETA_6)*sind(0)            cosd(THETA_6)*sind(0)        cosd(0)     cosd(0)*0;
            0                                0                            0           1
        ];

%% --- Testing --- %%
T_0_6 = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6;

disp(T_0_6);