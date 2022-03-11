% Frame     alpha_{i-1}     a_{i-1}     d_i           theta_i 
% -----------------------------------------------------------------
%   1            0             0       0.077          theta_1
%   2            90            0         0      theta_2 + 90 - 10.6    
%   3            0           0.130       0      theta_3 - 90 + 10.6
%   4            0           0.124       0            theta_4
%   5            0           0.126       0               0

%% -- Constant -- %%
theta_1 = 0;
theta_2 = -40.42;
theta_3 = 42.57;
theta_4 = -92.15;  

%% --- Waist TM --- %%

T_0_1 = [   cosd(theta_1)               -sind(theta_1)          0           0;
            sind(theta_1)*cosd(0)       cosd(theta_1)*cosd(0)   -sind(0)    -sind(0)*0.077;
            sind(theta_1)*sind(0)       cosd(theta_1)*sind(0)   cosd(0)     cosd(0)*0.077;
            0                           0                       0           1
        ];

%% --- Wrist TM --- %%
T_1_2 = [   cosd(theta_2 + 90 - 10.6)                    -sind(theta_2 + 90 - 10.6)               0           0;
            sind(theta_2 + 90 - 10.6)*cosd(90)           cosd(theta_2 + 90 - 10.6)*cosd(90)       -sind(90)   -sind(90)*0;
            sind(theta_2 + 90 - 10.6)*sind(90)           cosd(theta_2 + 90 - 10.6)*sind(90)       cosd(90)    cosd(90)*0;
            0                                            0                                        0           1
        ];
    
%% --- Shoulder TM --- %%

T_2_3 = [   cosd(theta_3 - 90 + 10.6)               -sind(theta_3 - 90 + 10.6)          0           0.130;
            sind(theta_3 - 90 + 10.6)*cosd(0)      cosd(theta_3 - 90 + 10.6)*cosd(0)    -sind(0)    -sind(0)*0;
            sind(theta_3 - 90 + 10.6)*sind(0)      cosd(theta_3 - 90 + 10.6)*sind(0)    cosd(0)     cosd(0)*0;
            0                                       0                                   0           1
        ];
    
%% --- Back Edge TM --- %%
T_3_4 = [   cosd(theta_4)                      -sind(theta_4)          0           0.124;
            sind(theta_4)*cosd(0)              cosd(theta_4)*cosd(0)   -sind(0)    -sind(0)*0;
            sind(theta_4)*sind(0)              cosd(theta_4)*sind(0)   cosd(0)     cosd(0)*0;
            0                                       0                                   0           1
        ];
  
%% --- Elbow TM --- %%
T_4_5 = [   cosd(0)                    -sind(0)               0           0.126;
            sind(0)*cosd(0)            cosd(0)*cosd(0)        -sind(0)    -sind(0)*0;
            sind(0)*sind(0)            cosd(0)*sind(0)        cosd(0)     cosd(0)*0;
            0                                0                            0           1
        ];

%% --- Testing --- %%
T_0_5 = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5;

disp(T_0_5);
