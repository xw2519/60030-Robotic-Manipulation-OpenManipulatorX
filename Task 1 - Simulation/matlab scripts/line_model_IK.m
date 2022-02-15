%% --- 4DOF Inverse Kinematic Solver --- %%

% This script handles the inverse kinematic calculations for a 4DOF robotic
% arm including the openMANIPULATOR-X.

% Outputs a list of angles (in Deg) representing the desired servo angles
% to reach the inputted (X,Y) coordinates

%% --- Specify the DH matrix --- %%

dhparams = [0   	0	    0.018  	  0;
            0       0       0.077     0 
            0.024   0	    0.128	  0;
            0.124  	0	    0    	  0;
            0.126   0   	0   	  0];

%% --- Initialise robotic components --- %%

robot = rigidBodyTree;

% Base servo
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');

% 
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');

setFixedTransform(jnt1,dhparams(1,:),'dh');
setFixedTransform(jnt2,dhparams(2,:),'dh');
setFixedTransform(jnt3,dhparams(3,:),'dh');
setFixedTransform(jnt4,dhparams(4,:),'dh');
setFixedTransform(jnt5,dhparams(5,:),'dh');

body1.Joint = jnt1;
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;

addBody(robot,body1,'base')
addBody(robot,body2,'body1')
addBody(robot,body3,'body2')
addBody(robot,body4,'body3')
addBody(robot,body5,'body4')

%% --- Display robot --- %%
showdetails(robot)

show(robot);
axis([-1,1,-1,1,-1,1])
axis on