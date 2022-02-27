function [F_1_X, F_1_Y, F_1_Z, F_2_X, F_2_Y, F_2_Z, F_3_X, F_3_Y, F_3_Z, F_4_X, F_4_Y, F_4_Z, F_5_X, F_5_Y, F_5_Z] = FK_coordinate_frames(THETA_1, THETA_2, THETA_3, THETA_4)
    %% -- Constant -- %%
    constant = atand(0.024/0.128);
    constant = 90-constant;

    %% --- Waist TM --- %%
    T_0_1 = [   cosd(THETA_1)                           -sind(THETA_1)                      0           0;
                sind(THETA_1)*cosd(0)                   cosd(THETA_1)*cosd(0)               -sind(0)    -sind(0)*0.077;
                sind(THETA_1)*sind(0)                   cosd(THETA_1)*sind(0)               cosd(0)     cosd(0)*0.077;
                0                                       0                                   0           1
            ];

    T_0_1_X = [ cosd(THETA_1)                           -sind(THETA_1)                      0           0.021;
                sind(THETA_1)*cosd(0)                   cosd(THETA_1)*cosd(0)               -sind(0)    0;
                sind(THETA_1)*sind(0)                   cosd(THETA_1)*sind(0)               cosd(0)     0;
                0                                       0                                   0           1
              ];
          
    T_0_1_Y = [ cosd(THETA_1)                           -sind(THETA_1)                      0           0;
                sind(THETA_1)*cosd(0)                   cosd(THETA_1)*cosd(0)               -sind(0)    0.021;
                sind(THETA_1)*sind(0)                   cosd(THETA_1)*sind(0)               cosd(0)     0;
                0                                       0                                   0           1
              ];
          
    T_0_1_Z = [ cosd(THETA_1)                           -sind(THETA_1)                      0           0;
                sind(THETA_1)*cosd(0)                   cosd(THETA_1)*cosd(0)               -sind(0)    0;
                sind(THETA_1)*sind(0)                   cosd(THETA_1)*sind(0)               cosd(0)     0.021;
                0                                       0                                   0           1
              ];


    %% --- Wrist TM --- %%
    T_1_2 = [   cosd(THETA_2 + constant)                -sind(THETA_2 + constant)           0           0;
                sind(THETA_2 + constant)*cosd(90)       cosd(THETA_2 + constant)*cosd(90)   -sind(90)   -sind(90)*0;
                sind(THETA_2  + constant)*sind(90)      cosd(THETA_2 + constant)*sind(90)   cosd(90)    cosd(90)*0;
                0                                       0                                   0           1
            ];

    T_1_2_X = [ cosd(THETA_2 + constant)                -sind(THETA_2 + constant)           0           0.021;
                sind(THETA_2 + constant)*cosd(90)       cosd(THETA_2 + constant)*cosd(90)   -sind(90)   0;
                sind(THETA_2  + constant)*sind(90)      cosd(THETA_2 + constant)*sind(90)   cosd(90)    0;
                0                                       0                                   0           1
            ];

    T_1_2_Y = [ cosd(THETA_2 + constant)                -sind(THETA_2 + constant)           0           0;
                sind(THETA_2 + constant)*cosd(90)       cosd(THETA_2 + constant)*cosd(90)   -sind(90)   0.021 ;
                sind(THETA_2  + constant)*sind(90)      cosd(THETA_2 + constant)*sind(90)   cosd(90)    0;
                0                                       0                                   0           1
            ];

    T_1_2_Z = [ cosd(THETA_2 + constant)                -sind(THETA_2 + constant)           0           0;
                sind(THETA_2 + constant)*cosd(90)       cosd(THETA_2 + constant)*cosd(90)   -sind(90)   0;
                sind(THETA_2  + constant)*sind(90)      cosd(THETA_2 + constant)*sind(90)   cosd(90)    0.021;
                0                                       0                                   0           1
            ];

    %% --- Shoulder TM --- %%

    T_2_3 = [   cosd(THETA_3  - constant)               -sind(THETA_3 - constant)           0           0.130;
                sind(THETA_3  - constant)*cosd(0)       cosd(THETA_3 - constant)*cosd(0)    -sind(0)    -sind(0)*0;
                sind(THETA_3  - constant)*sind(0)       cosd(THETA_3 - constant)*sind(0)    cosd(0)     cosd(0)*0;
                0                                       0                                   0           1
            ];

    T_2_3_X = [ cosd(THETA_3  - constant)               -sind(THETA_3 - constant)           0           0.021;
                sind(THETA_3  - constant)*cosd(0)       cosd(THETA_3 - constant)*cosd(0)    -sind(0)    0;
                sind(THETA_3  - constant)*sind(0)       cosd(THETA_3 - constant)*sind(0)    cosd(0)     0;
                0                                       0                                   0           1
            ];

    T_2_3_Y = [ cosd(THETA_3  - constant)               -sind(THETA_3 - constant)           0           0;
                sind(THETA_3  - constant)*cosd(0)       cosd(THETA_3 - constant)*cosd(0)    -sind(0)    0.021;
                sind(THETA_3  - constant)*sind(0)       cosd(THETA_3 - constant)*sind(0)    cosd(0)     0;
                0                                       0                                   0           1
            ];

    T_2_3_Z = [ cosd(THETA_3  - constant)               -sind(THETA_3 - constant)           0           0;
                sind(THETA_3  - constant)*cosd(0)       cosd(THETA_3 - constant)*cosd(0)    -sind(0)    0;
                sind(THETA_3  - constant)*sind(0)       cosd(THETA_3 - constant)*sind(0)    cosd(0)     0.021;
                0                                       0                                   0           1
            ];

    %% --- Back Edge TM --- %%
    T_3_4 = [   cosd(THETA_4)                           -sind(THETA_4)                      0           0.124;
                sind(THETA_4)*cosd(0)                   cosd(THETA_4)*cosd(0)               -sind(0)    -sind(0)*0;
                sind(THETA_4)*sind(0)                   cosd(THETA_4)*sind(0)               cosd(0)     cosd(0)*0;
                0                                       0                                   0           1
            ];

    T_3_4_X = [ cosd(THETA_4)                           -sind(THETA_4)                      0           0.021;
                sind(THETA_4)*cosd(0)                   cosd(THETA_4)*cosd(0)               -sind(0)    0;
                sind(THETA_4)*sind(0)                   cosd(THETA_4)*sind(0)               cosd(0)     0;
                0                                       0                                   0           1
            ];

    T_3_4_Y = [ cosd(THETA_4)                           -sind(THETA_4)                      0           0;
                sind(THETA_4)*cosd(0)                   cosd(THETA_4)*cosd(0)               -sind(0)    0.021;
                sind(THETA_4)*sind(0)                   cosd(THETA_4)*sind(0)               cosd(0)     0;
                0                                       0                                   0           1
            ];

    T_3_4_Z = [ cosd(THETA_4)                           -sind(THETA_4)                      0           0;
                sind(THETA_4)*cosd(0)                   cosd(THETA_4)*cosd(0)               -sind(0)    0;
                sind(THETA_4)*sind(0)                   cosd(THETA_4)*sind(0)               cosd(0)     0.021;
                0                                       0                                   0           1
            ];

    %% --- Elbow TM --- %%
    T_4_5 = [   cosd(0)                                 -sind(0)                            0           0.126;
                sind(0)*cosd(0)                         cosd(0)*cosd(0)                     -sind(0)    -sind(0)*0;
                sind(0)*sind(0)                         cosd(0)*sind(0)                     cosd(0)     cosd(0)*0;
                0                                       0                                   0           1
            ];

    T_4_5_X = [ cosd(0)                                 -sind(0)                            0           0.021;
                sind(0)*cosd(0)                         cosd(0)*cosd(0)                     -sind(0)    0;
                sind(0)*sind(0)                         cosd(0)*sind(0)                     cosd(0)     0;
                0                                       0                                   0           1
            ];

    T_4_5_Y = [ cosd(0)                                 -sind(0)                            0           0;
                sind(0)*cosd(0)                         cosd(0)*cosd(0)                     -sind(0)    0.021;
                sind(0)*sind(0)                         cosd(0)*sind(0)                     cosd(0)     0;
                0                                       0                                   0           1
            ];

    T_4_5_Z = [ cosd(0)                                 -sind(0)                            0           0;
                sind(0)*cosd(0)                         cosd(0)*cosd(0)                     -sind(0)    0;
                sind(0)*sind(0)                         cosd(0)*sind(0)                     cosd(0)     0.021;
                0                                       0                                   0           1
            ];

    %% --- Testing --- %%
    T_0_1 = T_0_1;
    T_0_2 = T_0_1*T_1_2;
    T_0_3 = T_0_1*T_1_2*T_2_3;
    T_0_4 = T_0_1*T_1_2*T_2_3*T_3_4;
    T_0_5 = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5;


    F_1_X = (T_0_1*T_0_1_X);  
    F_1_X = F_1_X([1,2,3], 4);
    F_1_Y = T_0_1*T_0_1_Y;  
    F_1_Y = F_1_Y([1,2,3], 4);
    F_1_Z = T_0_1*T_0_1_Z;
    F_1_Z = F_1_Z([1,2,3], 4);
    
    F_2_X = T_0_2*T_1_2_X;
    F_2_X = F_2_X([1,2,3], 4);
    F_2_Y = T_0_2*T_1_2_Y;
    F_2_Y = F_2_Y([1,2,3], 4);
    F_2_Z = T_0_2*T_1_2_Z;
    F_2_Z = F_2_Z([1,2,3], 4);
    
    F_3_X = T_0_3*T_2_3_X;
    F_3_X = F_3_X([1,2,3], 4);
    F_3_Y = T_0_3*T_2_3_Y;
    F_3_Y = F_3_Y([1,2,3], 4);
    F_3_Z = T_0_3*T_2_3_Z;
    F_3_Z = F_3_Z([1,2,3], 4);
    
    F_4_X = T_0_4*T_3_4_X;
    F_4_X = F_4_X([1,2,3], 4);
    F_4_Y = T_0_4*T_3_4_Y;
    F_4_Y = F_4_Y([1,2,3], 4);
    F_4_Z = T_0_4*T_3_4_Z;
    F_4_Z = F_4_Z([1,2,3], 4);
    
    F_5_X = T_0_5*T_4_5_X;
    F_5_X = F_5_X([1,2,3], 4);
    F_5_Y = T_0_5*T_4_5_Y;
    F_5_Y = F_5_Y([1,2,3], 4);
    F_5_Z = T_0_5*T_4_5_Z;
    F_5_Z = F_5_Z([1,2,3], 4);
end