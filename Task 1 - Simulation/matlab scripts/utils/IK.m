% function [THETA_1, THETA_2, THETA_3, THETA_4] = IK(P_X, P_Y, P_Z)

function [THETA_1, THETA_2, THETA_3, THETA_4] = IK(P_X, P_Y, P_Z)
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

    PHI = -45;

    R_2 = R_3 - A_4*cosd(PHI);
    Z_2 = Z_3 - A_4*sind(PHI);

    %% --- Theta 1 
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
    
    RAW_THETA_1 = THETA_1;

    %% --- Theta 3 
    ARGUMENT = ((R_2^2 + Z_2^2 - (A_2^2 + A_3^2))/(2*A_2*A_3));

    % THETA_3 = atan2d(ARGUMENT_2, ARGUMENT);

    THETA_3 = SIGN*acosd(ARGUMENT);
    
    RAW_THETA_3 = THETA_3;

    %% --- Theta 2

    COSINE_ARGUMENT_1 = (((A_2 + A_3*cosd(THETA_3))*Z_2) - ((A_3*sind(THETA_3))*R_2))/ (R_2^2 + Z_2^2);

    SINE_ARGUMENT_1 = (((A_2 + A_3*cosd(THETA_3))*R_2) + ((A_3*sind(THETA_3))*Z_2))/ (R_2^2 + Z_2^2);

    THETA_2 = atand(SINE_ARGUMENT_1/COSINE_ARGUMENT_1);

    %% --- Theta 4
    THETA_4 = PHI - THETA_3 - 90 + THETA_2;
    
    RAW_THETA_4 = THETA_4;

    %% --- Converting raw thetas into format required by FK
    RAW_THETA_2 = THETA_2;
    
    THETA_2 =  THETA_2 - (90 - constant);
    THETA_2 = -1*THETA_2;

    THETA_3 = THETA_3 + constant;
end