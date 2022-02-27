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
%--------------------------------------------------------------------
%% - Retrieve current theta values - %%
%Insert Start point
%P_X_start = 0.274; 
%P_Y_start = 0;
%P_Z_start = 0.205;
%Current Theta Values
T_1 = 0;
T_2 = -0.0192;
T_3 = 0.1276;
T_4 = -0.1084;
%Insert End Point
P_X_end = 0.38;
P_Y_end = 0;
P_Z_end = 0.090;

% --- Constants
A_2 = 0.130;
A_3 = 0.124;
A_4 = 0.126;

constant = atand(0.024/0.128);
constant = 90-constant;

P_R = abs(sqrt(P_X_end^2 + P_Y_end^2));
R_3 = P_R;
Z_3 = P_Z_end - 0.077;

T1_BOT_OPT = 0;
T2_BOT_OPT= 0;
T3_BOT_OPT = 0;
T4_BOT_OPT = 0;
T1_TOP_OPT = 0;
T2_TOP_OPT= 0;
T3_TOP_OPT = 0;
T4_TOP_OPT = 0;
OPT_DIST_TOP = 0;
OPT_DIST_BOT = 0;
%% - Do IK for values of phi btw -90 and 90 for elbow down - %%
for PHI = -90:1:90 %phi = (90-t2)+t3+T4
    disp("PHI IS:")
    disp(PHI);
    SIGN = -1; %ELBOW DOWN
    
    R_2 = R_3 - A_4*cosd(PHI);
    disp(R_2)
    Z_2 = Z_3 - A_4*sind(PHI);
    disp(Z_2)
    % --- Theta 1 --- %
    % Find correct value 
    if ~(P_X_end<=0) && ~(P_Y_end<=0)  
        THETA_1 = atand(P_Y_end/P_X_end);
    elseif (P_X_end<0) && ~(P_Y_end<0) 
        THETA_1 = atand(P_Y_end/P_X_end) + 180;
    elseif (P_X_end<0) && (P_Y_end<0) 
        THETA_1 = atand(P_Y_end/P_X_end) - 180;  
    else
        THETA_1 = atand(P_Y_end/P_X_end);
    end


    % --- Theta 3 --- %
    ARGUMENT = ((R_2^2 + Z_2^2 - (A_2^2 + A_3^2))/(2*A_2*A_3));

    % THETA_3 = atan2d(ARGUMENT_2, ARGUMENT);

    THETA_3 = SIGN*acosd(ARGUMENT);

    % --- Theta 2 --- %

    COSINE_ARGUMENT_1 = (((A_2 + A_3*cosd(THETA_3))*Z_2) - ((A_3*sind(THETA_3))*R_2))/ (R_2^2 + Z_2^2);

    SINE_ARGUMENT_1 = (((A_2 + A_3*cosd(THETA_3))*R_2) + ((A_3*sind(THETA_3))*Z_2))/ (R_2^2 + Z_2^2);

    THETA_2 = atand(SINE_ARGUMENT_1/COSINE_ARGUMENT_1);

    % --- Theta 4 --- %
    THETA_4 = PHI - THETA_3 - 90 + THETA_2;

    disp('Raw Theta 1')
    disp(THETA_1)

    disp('Raw Theta 2')
    disp(THETA_2)

    disp('Raw Theta 3')
    disp(THETA_3)

    disp('Raw Theta 4')
    disp(THETA_4)

  %% -- Optimal Theta -- %% 
     if (imag(THETA_3)<10^(-5) && imag(THETA_4)<10^(-5))
        DIST_T1 = real(THETA_1) - T_1;
        DIST_T2 = real(THETA_2) - T_2;
        DIST_T3 = imag(THETA_3) - T_3;
        DIST_T4 = imag(THETA_4) - T_4; 
        EUCL_DIST = sqrt(DIST_T1^2 + DIST_T2^2 + DIST_T3^2 +DIST_T4^2);
        if (OPT_DIST_TOP == 0)
            OPT_DIST_TOP = EUCL_DIST;
        elseif(EUCL_DIST < OPT_DIST_TOP)
            OPT_DIST_TOP= EUCL_DIST;
            T1_TOP_OPT = real(THETA_1);
            T2_TOP_OPT = real(THETA_2);
            T3_TOP_OPT = imag(THETA_3);
            T4_TOP_OPT = imag(THETA_4);
        end
    end
    if isreal(THETA_1) && isreal(THETA_2) && isreal(THETA_3) && isreal(THETA_4)
        DIST_T1 = THETA_1 - T_1;
        DIST_T2 = THETA_2 - T_2;
        DIST_T3 = THETA_3 - T_3;
        DIST_T4 = THETA_4 - T_4; 
        EUCL_DIST = sqrt(DIST_T1^2 + DIST_T2^2 + DIST_T3^2 +DIST_T4^2);
        if (OPT_DIST_BOT == 0)
            OPT_DIST_BOT = EUCL_DIST;
        elseif (EUCL_DIST < OPT_DIST_BOT)
            PT_DIST_BOT= EUCL_DIST;
            T1_BOT_OPT = THETA_1;
            T2_BOT_OPT = THETA_2;
            T3_BOT_OPT = THETA_3;
            T4_BOT_OPT = THETA_4;
        end
    end
end
%% - Do IK for values of phi btw -90 and 90 for elbow up - %%

for PHI = -90:1:90 %phi = (90-t2)+t3+T4 
    disp(PHI);
    SIGN = 1; %ELBOW UP
    
    R_2 = R_3 - A_4*cosd(PHI);
    disp(R_2)
    Z_2 = Z_3 - A_4*sind(PHI);
    disp(Z_2)
    % --- Theta 1 --- %
    % Find correct value 
    if ~(P_X_end<=0) && ~(P_Y_end<=0)  
        THETA_1 = atand(P_Y_end/P_X_end);
    elseif (P_X_end<0) && ~(P_Y_end<0) 
        THETA_1 = atand(P_Y_end/P_X_end) + 180;
    elseif (P_X_end<0) && (P_Y_end<0) 
        THETA_1 = atand(P_Y_end/P_X_end) - 180;  
    else
        THETA_1 = atand(P_Y_end/P_X_end);
    end


    % --- Theta 3 --- %
    ARGUMENT = ((R_2^2 + Z_2^2 - (A_2^2 + A_3^2))/(2*A_2*A_3));

    % THETA_3 = atan2d(ARGUMENT_2, ARGUMENT);

    THETA_3 = SIGN*acosd(ARGUMENT);

    % --- Theta 2 --- %

    COSINE_ARGUMENT_1 = (((A_2 + A_3*cosd(THETA_3))*Z_2) - ((A_3*sind(THETA_3))*R_2))/ (R_2^2 + Z_2^2);

    SINE_ARGUMENT_1 = (((A_2 + A_3*cosd(THETA_3))*R_2) + ((A_3*sind(THETA_3))*Z_2))/ (R_2^2 + Z_2^2);

    THETA_2 = atand(SINE_ARGUMENT_1/COSINE_ARGUMENT_1);

    % --- Theta 4 --- %
    THETA_4 = PHI - THETA_3 - 90 + THETA_2;

    disp('Raw Theta 1')
    disp(THETA_1)

    disp('Raw Theta 2')
    disp(THETA_2)

    disp('Raw Theta 3')
    disp(THETA_3)

    disp('Raw Theta 4')
    disp(THETA_4)
    
    %% - Compare distance with current thetas - %%
    if ( imag(THETA_3)<10^(-5)&&imag(THETA_4)<10^(-5))
        DIST_T1 = real(THETA_1) - T_1;
        DIST_T2 = real(THETA_2) - T_2;
        DIST_T3 = imag(THETA_3) - T_3;
        DIST_T4 = imag(THETA_4) - T_4; 
        EUCL_DIST = sqrt(DIST_T1^2 + DIST_T2^2 + DIST_T3^2 +DIST_T4^2);
        if (OPT_DIST_TOP == 0)
            OPT_DIST_TOP = EUCL_DIST;
        elseif(EUCL_DIST < OPT_DIST_TOP)
            OPT_DIST_TOP= EUCL_DIST;
            T1_TOP_OPT = real(THETA_1);
            T2_TOP_OPT = real(THETA_2);
            T3_TOP_OPT = imag(THETA_3);
            T4_TOP_OPT = imag(THETA_4);
        end
    end
    if (isreal(THETA_1) && isreal(THETA_2) && isreal(THETA_3) && isreal(THETA_4))
        DIST_T1 = THETA_1 - T_1;
        DIST_T2 = THETA_2 - T_2;
        DIST_T3 = THETA_3 - T_3;
        DIST_T4 = THETA_4 - T_4; 
        EUCL_DIST = sqrt(DIST_T1^2 + DIST_T2^2 + DIST_T3^2 +DIST_T4^2);
        if (OPT_DIST_TOP == 0)
            OPT_DIST_TOP = EUCL_DIST;
        elseif(EUCL_DIST < OPT_DIST_TOP)
            OPT_DIST_TOP= EUCL_DIST;
            T1_TOP_OPT = THETA_1;
            T2_TOP_OPT = THETA_2;
            T3_TOP_OPT = THETA_3;
            T4_TOP_OPT = THETA_4;
        end
    end
end
disp("BEST BOT OPTIMAL THETAS:");
disp(T1_BOT_OPT);
disp(T2_BOT_OPT);
disp(T3_BOT_OPT);
disp(T4_BOT_OPT);
disp(OPT_DIST_BOT);
disp("BEST TOP OPTIMAL THETAS:");
disp(T1_TOP_OPT);
disp(T2_TOP_OPT);
disp(T3_TOP_OPT);
disp(T4_TOP_OPT);
disp(OPT_DIST_TOP);
%% Compare outputed optimal thetas from elbow and up and return best thetas
if (OPT_DIST_TOP > OPT_DIST_BOT)
    theta_1 = T1_BOT_OPT;
    theta_2 = -1*(T2_BOT_OPT-(90-constant));
    theta_3 = T3_BOT_OPT + constant;
    theta_4 = T4_BOT_OPT; 
elseif (OPT_DIST_TOP < OPT_DIST_BOT)
    theta_1 = T1_TOP_OPT;
    theta_2 = -1*(T2_TOP_OPT-(90-constant));
    theta_3 = T3_TOP_OPT + constant;
    theta_4 = T4_TOP_OPT; 
end 

disp('Theta 1')
disp(theta_1)

disp('Theta 2')
disp(theta_2)

disp('Theta 3')
disp(theta_3)

disp('Theta 4')
disp(theta_4)

%% -- VERIFY ANGLES MATCH END POINT -- %%  

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