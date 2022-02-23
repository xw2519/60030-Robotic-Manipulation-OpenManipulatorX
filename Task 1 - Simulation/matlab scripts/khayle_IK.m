%% --- Desired coordinates --- %%
P_X = 0; 
P_Y = 0;
P_Z = 0;

%% --- Constants
S_4 = 


%% --- Theta_1
if ~(P_Y<=0) && ~(P_X<=0)  
    theta_1 = atan(P_Y/P_X);
elseif (P_Y<0) && ~(P_X<0) 
    theta_1 = atan(P_Y/P_X) + 180;  
elseif (P_Y<0) && (P_X<0) 
    theta_1 = atan(P_Y/P_X) - 180;  
else
    theta_1 = atan(P_Y/P_X);
end

disp(theta_1);

%% --- Theta_2


%% --- Theta_3


%% --- Theta_4


