
T_1 = 0;
T_2 = 10.6388;
T_3 = -79.2528;
T_4 = -0.1084;

[THETA_1, THETA_2, THETA_3, THETA_4, PHI, RAW_THETA1, RAW_THETA2, RAW_THETA3, RAW_THETA4] = IK_Optimal(0.175, 0.05, 0.015, T_1, T_2, T_3, T_4);

disp("Raw theta1")
disp(RAW_THETA1)
disp("Raw theta2")
disp(RAW_THETA2)
disp("Raw theta3")
disp(RAW_THETA3)
disp("Raw theta4")
disp(RAW_THETA4)
