%% --- disable_robotic_arm --- %%

% This script disables the robotic arm by disabling dynamixel torque 
% and closing the port

%% --- Move robot to closing position -- %%
disp('Moving robotic arm to closing position')

dxl_1_closing_positions = [2025, 733, 3019, 2434, 3498];

write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BaseRotation, ADDR_PRO_GOAL_POSITION, typecast(int32(dxl_1_closing_positions(1)), 'uint32'));
pause(1)
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_Shoulder, ADDR_PRO_GOAL_POSITION, typecast(int32(dxl_1_closing_positions(2)), 'uint32'));
pause(1)
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_Elbow, ADDR_PRO_GOAL_POSITION, typecast(int32(dxl_1_closing_positions(3)), 'uint32'));
pause(1)
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_Wrist, ADDR_PRO_GOAL_POSITION, typecast(int32(dxl_1_closing_positions(4)), 'uint32'));
pause(1)
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_Gripper, ADDR_PRO_GOAL_POSITION, typecast(int32(dxl_1_closing_positions(5)), 'uint32'));
pause(1)

%% --- Disable robot settings and connection --- %%
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BaseRotation, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_Shoulder, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_Elbow, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_Wrist, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_Gripper, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);          

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

pause(2)

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

close all;
clear all;