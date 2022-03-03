function [theta_1, theta_2, theta_3, theta_4, phi, RAW_OPT_T1, RAW_OPT_T2, RAW_OPT_T3, RAW_OPT_T4] = IK_Optimal(P_X, P_Y, P_Z, T_1, T_2, T_3, T_4)
    %% --- Constants
    A_2 = 0.130;
    A_3 = 0.124;
    A_4 = 0.126;
    
    OPT_DIST_BOT = 999;
    OPT_DIST_TOP = 999;

    constant = atand(0.024/0.128);
    constant = 90-constant;


    P_R = abs(sqrt(P_X^2 + P_Y^2));
    R_3 = P_R;
    Z_3 = P_Z - 0.077;
    %% - Do IK for values of phi btw -90 and 90 for elbow down - %%
    for PHI = -90:1:90 %phi = (90-t2)+t3+T4
        SIGN = -1; %ELBOW DOWN
    
        R_2 = R_3 - A_4*cosd(PHI);
        Z_2 = Z_3 - A_4*sind(PHI);
        % --- Theta 1 --- %
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

        THETA_3 = SIGN*acosd(ARGUMENT);;

       %% --- Theta 2 --- %%

        COSINE_ARGUMENT_1 = (((A_2 + A_3*cosd(THETA_3))*Z_2) - ((A_3*sind(THETA_3))*R_2))/ (R_2^2 + Z_2^2);

        SINE_ARGUMENT_1 = (((A_2 + A_3*cosd(THETA_3))*R_2) + ((A_3*sind(THETA_3))*Z_2))/ (R_2^2 + Z_2^2);

        THETA_2 = atand(SINE_ARGUMENT_1/COSINE_ARGUMENT_1);

        %% --- Theta 4 --- %%
        THETA_4 = PHI - THETA_3 - 90 + THETA_2;

      %% -- Optimal Theta -- %% 
        if isreal(THETA_1) && isreal(THETA_2) && isreal(THETA_3) && isreal(THETA_4)
            DIST_T1 = THETA_1 - T_1;
            DIST_T2 = THETA_2 - T_2;
            DIST_T3 = THETA_3 - T_3;
            DIST_T4 = THETA_4 - T_4; 
            EUCL_DIST = sqrt(DIST_T1^2 + DIST_T2^2 + DIST_T3^2 +DIST_T4^2);
            if (EUCL_DIST < OPT_DIST_BOT)
                OPT_DIST_BOT = EUCL_DIST;
                T1_BOT_OPT = THETA_1;
                T2_BOT_OPT = THETA_2;
                T3_BOT_OPT = THETA_3;
                T4_BOT_OPT = THETA_4;
            end
        elseif (imag(THETA_1) <= 10^-5) && (imag(THETA_2) <= 10^-5) && (imag(THETA_3) <= 10^-5) && (imag(THETA_4) <= 10^-5) 
            DIST_T1 = real(THETA_1)- T_1;
            DIST_T2 = real(THETA_2) - T_2;
            DIST_T3 = real(THETA_3) - T_3;
            DIST_T4 = real(THETA_4) - T_4; 
            EUCL_DIST = sqrt(DIST_T1^2 + DIST_T2^2 + DIST_T3^2 +DIST_T4^2);
            if (EUCL_DIST < OPT_DIST_BOT)
                OPT_DIST_BOT = EUCL_DIST;
                T1_BOT_OPT = real(THETA_1);
                T2_BOT_OPT = real(THETA_2);
                T3_BOT_OPT = real(THETA_3);
                T4_BOT_OPT = real(THETA_4);
            end
        end
    end
    phi_bot_opt = T2_BOT_OPT+T3_BOT_OPT+T4_BOT_OPT;
    %% - Do IK for values of phi btw -90 and 90 for elbow up - %%
    for PHI = -90:1:90 %phi = (90-t2)+t3+T4 
        SIGN = 1; %ELBOW UP

        R_2 = R_3 - A_4*cosd(PHI);
        Z_2 = Z_3 - A_4*sind(PHI);
        % --- Theta 1 --- %
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

        %% - Compare distance with current thetas - %%
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
        elseif (imag(THETA_1) <= 10^-5) && (imag(THETA_2) <= 10^-5) && (imag(THETA_3) <= 10^-5) && (imag(THETA_4) <= 10^-5) 
            DIST_T1 = real(THETA_1)- T_1;
            DIST_T2 = real(THETA_2) - T_2;
            DIST_T3 = real(THETA_3) - T_3;
            DIST_T4 = real(THETA_4) - T_4; 
            EUCL_DIST = sqrt(DIST_T1^2 + DIST_T2^2 + DIST_T3^2 +DIST_T4^2);
            if (EUCL_DIST < OPT_DIST_TOP)
                OPT_DIST_TOP = EUCL_DIST;
                T1_TOP_OPT = real(THETA_1);
                T2_TOP_OPT = real(THETA_2);
                T3_TOP_OPT = real(THETA_3);
                T4_TOP_OPT = real(THETA_4);
            end
        end
    end
    phi_top_opt = T2_TOP_OPT+T3_TOP_OPT+T4_TOP_OPT;
    %% Compare outputed optimal thetas from elbow and up and return best thetas
    if (OPT_DIST_TOP > OPT_DIST_BOT)
        T2_BOT_OPT =  T2_BOT_OPT - (90 - constant);
        theta_1 = T1_BOT_OPT;
        theta_2 = -1*T2_BOT_OPT;
        theta_3 = T3_BOT_OPT + constant;
        theta_4 = T4_BOT_OPT; 
        phi = theta_2 + theta_3 + theta_4;
        disp(phi)
        RAW_OPT_T1 = T1_BOT_OPT;
        RAW_OPT_T2 = T2_BOT_OPT;
        RAW_OPT_T3 = T3_BOT_OPT;
        RAW_OPT_T4 = T4_BOT_OPT;
    elseif (OPT_DIST_TOP <= OPT_DIST_BOT)
        T2_BOT_OPT =  T2_BOT_OPT - (90 - constant);
        theta_1 = T1_TOP_OPT;
        theta_2 = -1*T2_BOT_OPT;
        theta_3 = T3_TOP_OPT + constant;
        theta_4 = T4_TOP_OPT; 
        phi = theta_2 + theta_3 + theta_4;
        RAW_OPT_T1 = T1_TOP_OPT;
        RAW_OPT_T2 = T2_TOP_OPT;
        RAW_OPT_T3 = T3_TOP_OPT;
        RAW_OPT_T4 = T4_TOP_OPT;
    end 