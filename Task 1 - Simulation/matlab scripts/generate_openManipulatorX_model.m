%% --- OpenMANIPULATOR-X model generation using Robotics System Toolbox --- %%

% This script generates the OpenMANIPULATOR-X model and its coordinate
% frames

%% --- Load and display robot --- %%

clear
clc

addpath(genpath(strcat(pwd,'\Dependencies')))
robot = createRigidBodyTree;
axes = show(robot);
axes.CameraPositionMode = 'auto';
