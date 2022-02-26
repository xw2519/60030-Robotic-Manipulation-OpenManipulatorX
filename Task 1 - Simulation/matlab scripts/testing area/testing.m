%% --- Coordinates of joints
Base          = [0,     0, 0];
Base_Rotation = [0,     0, 0.077];
Shoulder      = [0,     0, 0.077];
Elbow         = [0.024, 0, 0.205];
Wrist         = [0.148, 0, 0.205];
EE            = [0.274, 0, 0.205];

THETA_1 = 0;
THETA_2 = -53.8541;
THETA_3 = 62.1913;
THETA_4 = -98.3372;

tiledlayout(2,2)

ax1 = nexttile;
grid on;

ax2 = nexttile;
grid on;

ax3 = nexttile;
grid on;

ax4 = nexttile;
grid on;
%% --- Initial state
x = [0, 0,     0.1173,  0.2400,   0.2400  ];
y = [0, 0,     0,       0,        0       ];
z = [0, 0.077, 0.1330,  0.1510,   0.0250  ];

arm = plot3(ax1, x, y, z, '-ok');
hold on;

arm = copy(arm);
plot3(ax2, arm);
hold on;

% Add coordinate frames 
% Base rotation
plot3([0, 0.021], [0, 0],     [0, 0], '-r', 'LineWidth',2);
plot3([0, 0],     [0, 0.021], [0, 0], '-g', 'LineWidth',2);
plot3([0, 0],     [0, 0],     [0, 0.021], '-b', 'LineWidth',2);

% Shoulder
plot3([0, 0.021], [0, 0],     [0.077, 0.077], '-r', 'LineWidth',2);
plot3([0, 0],     [0, -0.021], [0.077, 0.077], '-b', 'LineWidth',2);
plot3([0, 0],     [0, 0],     [0.077, 0.077 + 0.021], '-g', 'LineWidth',2);

plot3([0, 0.0190],     [0, 0],     [0.077, 0.0860], '-r', 'LineWidth',2);
plot3([0, -0.0090],     [0, 0],     [0.077, 0.0960], '-g', 'LineWidth',2);
plot3([0, 0],          [0, -0.021], [0.077, 0.077], '-b', 'LineWidth',2);

xlabel('x');
ylabel('y');
zlabel('z');

axis equal;
axis([0 0.5 -0.5 0.5 0 0.5])

hold off;
