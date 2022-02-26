% Frame     alpha_{i-1}     a_{i-1}     d_i           theta_i 
% -----------------------------------------------------------------
%   1            0             0       0.077          theta_1
%   2            90            0         0      theta_2 + 90 - 10.6    
%   3            0           0.130       0      theta_3 - 90 + 10.6
%   4            0           0.124       0            theta_4
%   5            0           0.126       0               0

% a_2 = 0.130
% a_3 = 0.124
% a_4 = 0.126

%% --- Desired coordinates --- %%
P_X = 0.2400; 
P_Y = 0.0;
P_Z = 0.0250;

%% --- Constants
A_2 = 0.130;
A_3 = 0.124;
A_4 = 0.126;

constant = atand(0.024/0.128);
constant = 90-constant;

SIGN = -1;

P_R = abs(sqrt(P_X^2 + P_Y^2));
R_3 = P_R;
Z_3 = P_Z - 0.077;

PHI = -90;

R_2 = R_3 - A_4*cosd(PHI);
Z_2 = Z_3 - A_4*sind(PHI);

%% --- Theta 1 --- %%
% Find correct value 
if ~(P_X<=0) && ~(P_Y<=0)  
    THETA_1 = atand(P_Y/P_X);
elseif (P_X<0) && ~(P_Y<0) 
    THETA_1 = atand(P_Y/P_X) + 180;
elseif (P_X<0) && (P_Y<0) 
    THETA_1 = atand(P_Y/P_X) - 180;  
else
    THETA_1 = atand(P_Y/P_X);
end


%% --- Theta 3 --- %%
ARGUMENT = ((R_2^2 + Z_2^2 - (A_2^2 + A_3^2))/(2*A_2*A_3));

% THETA_3 = atan2d(ARGUMENT_2, ARGUMENT);

THETA_3 = SIGN*acosd(ARGUMENT);

%% --- Theta 2 --- %%

COSINE_ARGUMENT_1 = (((A_2 + A_3*cosd(THETA_3))*Z_2) - ((A_3*sind(THETA_3))*R_2))/ (R_2^2 + Z_2^2);

SINE_ARGUMENT_1 = (((A_2 + A_3*cosd(THETA_3))*R_2) + ((A_3*sind(THETA_3))*Z_2))/ (R_2^2 + Z_2^2);

THETA_2 = atand(SINE_ARGUMENT_1/COSINE_ARGUMENT_1);

%% --- Theta 4 --- %%
THETA_4 = PHI - THETA_3 - 90 + THETA_2;

disp('Raw Theta 1')
disp(THETA_1)

disp('Raw Theta 2')
disp(THETA_2)

disp('Raw Theta 3')
disp(THETA_3)

disp('Raw Theta 4')
disp(THETA_4)

% Converting raw thetas into format required by FK
THETA_2 =  THETA_2 - (90 - constant);
THETA_2 = -1*THETA_2;

THETA_3 = THETA_3 + constant;

%% -- Constant -- %%
theta_1 = THETA_1;
theta_2 = THETA_2;
theta_3 = THETA_3;
theta_4 = THETA_4;  

disp('Theta 1')
disp(THETA_1)

disp('Theta 2')
disp(THETA_2)

disp('Theta 3')
disp(THETA_3)

disp('Theta 4')
disp(THETA_4)

%% --- Waist TM --- %%

T_0_1 = [   cosd(theta_1)               -sind(theta_1)          0           0;
            sind(theta_1)*cosd(0)       cosd(theta_1)*cosd(0)   -sind(0)    -sind(0)*0.077;
            sind(theta_1)*sind(0)       cosd(theta_1)*sind(0)   cosd(0)     cosd(0)*0.077;
            0                           0                       0           1
        ];

%% --- Wrist TM --- %%
T_1_2 = [   cosd(theta_2 + constant)                    -sind(theta_2 + constant)               0           0;
            sind(theta_2 + constant)*cosd(90)           cosd(theta_2 + constant)*cosd(90)       -sind(90)   -sind(90)*0;
            sind(theta_2  + constant)*sind(90)           cosd(theta_2 + constant)*sind(90)       cosd(90)    cosd(90)*0;
            0                                            0                                        0           1
        ];
    
%% --- Shoulder TM --- %%

T_2_3 = [   cosd(theta_3  - constant)               -sind(theta_3 - constant)          0           0.130;
            sind(theta_3  - constant)*cosd(0)      cosd(theta_3 - constant)*cosd(0)    -sind(0)    -sind(0)*0;
            sind(theta_3  - constant)*sind(0)      cosd(theta_3 - constant)*sind(0)    cosd(0)     cosd(0)*0;
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
T_0_1 = T_0_1;
T_0_2 = T_0_1*T_1_2;
T_0_3 = T_0_1*T_1_2*T_2_3;
T_0_4 = T_0_1*T_1_2*T_2_3*T_3_4;
T_0_5 = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5;

disp('T_0_1')
disp(T_0_1);

disp('T_0_2')
disp(T_0_2);

disp('T_0_3')
disp(T_0_3);

disp('T_0_4')
disp(T_0_4);

disp('T_0_5')
disp(T_0_5);
