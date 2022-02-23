% Frame     d       theta            r       alpha 
% ----------------------------------------------------
%   1       0.077   theta_1          0        90
%   2       0       theta_1 - 11     0.130    0
%   3       0       theta_3 + 11     0.135    0
%   4       0       theta_4          0.126    0

% a_2 = 0.130
% a_3 = 0.124
% a_4 = 0.126

%% --- Desired coordinates --- %%
P_X = 0.2740; 
P_Y = 0;
P_Z = 0.2050;

%% --- Constants
A_2 = 0.130;
A_3 = 0.124;
A_4 = 0.126;


SIGN = 1;

P_R = abs(sqrt(P_X^2 + P_Y^2));
R_3 = P_R;
Z_3 = P_Z - 0.077;

PHI = atan2d(Z_3, R_3);

disp(PHI);

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

disp('Theta 1')
disp(THETA_1)

%% --- Theta 3 --- %%
ARGUMENT = ((R_2^2 + Z_2^2 - (A_2^2 + A_3^2))/(2*A_2*A_3));
ARGUMENT_2 = SIGN*sqrt((1 - ARGUMENT^2));

% THETA_3 = atan2d(ARGUMENT_2, ARGUMENT);

THETA_3 = SIGN*acosd(ARGUMENT);

disp('Theta 3')
disp(THETA_3)


%% --- Theta 2 --- %%

SINE_ARGUMENT_1 = ((A_2 + A_3*cosd(THETA_3))*Z_2 + (A_3*sind(THETA_3))*R_2) / (R_2^2 + Z_2^2);

COSINE_ARGUMENT_1 = ((A_2 + A_3*cosd(THETA_3)*R_2) + (A_3*sind(THETA_3))*Z_2) / (R_2^2 + Z_2^2);

THETA_2 = atand(SINE_ARGUMENT_1/COSINE_ARGUMENT_1);

disp('Theta 2')
disp(THETA_2)

%% --- Theta 4 --- %%
THETA_4 = PHI - (THETA_2 + THETA_3);

disp('Theta 4')
disp(THETA_4)

