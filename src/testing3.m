
T_1 = 0;
T_2 = 10.6388;
T_3 = -79.2528;
T_4 = -0.1084;
phi = -90;

[THETA_1, THETA_2, THETA_3, THETA_4, PHI, RAW_THETA1, RAW_THETA2, RAW_THETA3, RAW_THETA4] = IK_Optimal(0.175, 0.05, 0.025, T_1, T_2, T_3, T_4);
[A_THETA_1, A_THETA_2, A_THETA_3, A_THETA_4] = IK(0.175, 0.05, 0.025, phi);

disp("Raw theta1")
disp(A_THETA_1)
disp("Raw theta2")
disp(A_THETA_2)
disp("Raw theta3")
disp(A_THETA_3)
disp("Raw theta4")
disp(A_THETA_4)
