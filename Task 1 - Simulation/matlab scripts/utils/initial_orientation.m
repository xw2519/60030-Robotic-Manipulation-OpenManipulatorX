%% --- initial_orientation --- %%

% This script sets the robot arm to a initial pre-determined position

%% --- Clear terminal and load libraries --- %%
clc;
clear all;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
end

%% --- Load setup script and save variables --- %
% Saves the environment variables into env.mat file
setup_robotic_arm

%% --- Set actuators into position control mode and torque mode --- %%
% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BaseRotation, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_Shoulder, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_Elbow, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_Wrist, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_Gripper, ADDR_PRO_OPERATING_MODE, 3);

% Enable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BaseRotation, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_Shoulder, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_Elbow, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_Wrist, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_Gripper, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

%% --- Move into initial position --- %%

dxl_1_goal_position = [2049, 1517, 2307, 2474, 3498];   

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BaseRotation, ADDR_PRO_GOAL_POSITION, typecast(int32(dxl_1_goal_position(1)), 'uint32'));
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_Elbow, ADDR_PRO_GOAL_POSITION, typecast(int32(dxl_1_goal_position(3)), 'uint32'));

if getLastTxRxResult(port_num, PROTOCOL_VERSION) ~= COMM_SUCCESS
    printTxRxResult(PROTOCOL_VERSION, getLastTxRxResult(port_num, PROTOCOL_VERSION));
elseif getLastRxPacketError(port_num, PROTOCOL_VERSION) ~= 0
    printRxPacketError(PROTOCOL_VERSION, getLastRxPacketError(port_num, PROTOCOL_VERSION));
end


%% --- Disable robotic arm --- %
disable_robotic_arm
