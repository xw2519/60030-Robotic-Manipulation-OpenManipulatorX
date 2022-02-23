% Frame     d       theta            r       alpha 
% ----------------------------------------------------
%   1       0.077   theta_1          0        90
%   2       0       theta_2 - 11     0.130    0
%   3       0       theta_3 + 11     0.135    0
%   4       0       theta_4          0.126    0

% a_2 = 0.130
% a_3 = 0.124
% a_4 = 0.126

%% --- Desired coordinates --- %%
P_X = 0.1; 
P_Y = 0;
P_Z = 0.1 + 0.126;

R = sqrt(P_X^2 + P_Y^2);

%% --- Constants
L_1 = 0.077;
L_2 = 0.130;
L_3 = 0.124;
L_4 = 0.126;

PHI = 0;

%% --- Theta 1 --- %%
% Find correct value 
THETA_1 = atan2d(P_Y, P_X);

disp('Theta 1')
disp(THETA_1)


%% --- Theta 3 --- %%
CosTheta3 = (R^2 + P_Z^2 - (L_2)^2 - (L_3)^2)/(2*(L_2)*(L_3));
SineTheta3 = -sqrt(1 - CosTheta3^2);
THETA_3 = atan2d(SineTheta3, CosTheta3);

disp('Theta 3')
disp(THETA_3)


%% --- Theta 2 --- %%
K2 = L_3*SineTheta3;
K1 = L_2 + (L_3)*CosTheta3;

THETA_2 = atan2d(R, P_Z) - atan2d(K2, K1);

disp('Theta 2')
disp(THETA_2)

%% --- Theta 4 --- %%
THETA_4 = -(90 + THETA_2 - abs(THETA_3));

disp('Theta 4')
disp(THETA_4)
