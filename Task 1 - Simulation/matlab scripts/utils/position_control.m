%% --- Position_control --- %%

% This script sets the robot arm to position control 

%% --- Load setup script and save variables --- %
% Saves the environment variables into env.mat file
setup_robotic_arm

%% --- Cancellation button --- %%
hWaitbar = waitbar(0, 'Reading encoder values', 'Name', 'Encoder reading in progress', 'CreateCancelBtn', 'delete(gcbf)');

%% ---- Switch Servo Torque Off to Allow Manual Tracking---- %%

% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BaseRotation, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_Shoulder, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_Elbow, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_Wrist, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_Gripper, ADDR_PRO_OPERATING_MODE, 3);

% Disable Dynamixel Torque
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
else
    fprintf('Dynamixel has been successfully connected \n');
end

%% ---- Track Servo Position and Convert Into Angles and Radians ---- %%
while 1
    if ~ishandle(hWaitbar)
        % Stop the if cancel button was pressed
        disp('Stopped by user');
        break;
    else
        % Read present servo position
        dxl_ID1_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID1_BaseRotation, ADDR_PRO_PRESENT_POSITION);
        dxl_ID2_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID2_Shoulder, ADDR_PRO_PRESENT_POSITION);
        dxl_ID3_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID3_Elbow, ADDR_PRO_PRESENT_POSITION);
        dxl_ID4_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID4_Wrist, ADDR_PRO_PRESENT_POSITION);
        dxl_ID5_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID5_Gripper, ADDR_PRO_PRESENT_POSITION);

        dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
        dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

        if dxl_comm_result ~= COMM_SUCCESS
            fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        elseif dxl_error ~= 0
            fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
        end

        % Convert to angles in degrees and radians
        % 0.088 [°] <------> 0 ~ 4,095(1 rotation)
        % Round to nearest integer
        dxl_ID1_angle_degree = (0.088 * typecast(single(dxl_ID1_present_position), 'single'));
        dxl_ID2_angle_degree = (0.088 * typecast(single(dxl_ID2_present_position), 'single'));
        dxl_ID3_angle_degree = (0.088 * typecast(single(dxl_ID3_present_position), 'single'));
        dxl_ID4_angle_degree = (0.088 * typecast(single(dxl_ID4_present_position), 'single'));
        dxl_ID5_angle_degree = (0.088 * typecast(single(dxl_ID5_present_position), 'single'));
        
        dxl_ID1_angle_radian = deg2rad(dxl_ID1_angle_degree);
        dxl_ID2_angle_radian = deg2rad(dxl_ID2_angle_degree);
        dxl_ID3_angle_radian = deg2rad(dxl_ID3_angle_degree);
        dxl_ID4_angle_radian = deg2rad(dxl_ID4_angle_degree);
        dxl_ID5_angle_radian = deg2rad(dxl_ID5_angle_degree);

        % Print out readings and conversions
        fprintf('[ID:%03d] Position: %.1f - Angle(Deg): %.4f - Angle(Rad): %.4f\n', DXL_ID1_BaseRotation, typecast(uint32(dxl_ID1_present_position), 'int32'), dxl_ID1_angle_degree, dxl_ID1_angle_radian);
        fprintf('[ID:%03d] Position: %.1f - Angle(Deg): %.4f - Angle(Rad): %.4f\n', DXL_ID2_Shoulder, typecast(uint32(dxl_ID2_present_position), 'int32'), dxl_ID2_angle_degree, dxl_ID2_angle_radian);
        fprintf('[ID:%03d] Position: %.1f - Angle(Deg): %.4f - Angle(Rad): %.4f\n', DXL_ID3_Elbow, typecast(uint32(dxl_ID3_present_position), 'int32'), dxl_ID3_angle_degree, dxl_ID3_angle_radian);
        fprintf('[ID:%03d] Position: %.1f - Angle(Deg): %.4f - Angle(Rad): %.4f\n', DXL_ID4_Wrist, typecast(uint32(dxl_ID4_present_position), 'int32'), dxl_ID4_angle_degree, dxl_ID4_angle_radian);
        fprintf('[ID:%03d] Position: %.1f - Angle(Deg): %.4f - Angle(Rad): %.4f\n', DXL_ID5_Gripper, typecast(uint32(dxl_ID5_present_position), 'int32'), dxl_ID5_angle_degree, dxl_ID5_angle_radian);
    end
end

%% --- Disable robotic arm --- %
disable_robotic_arm
