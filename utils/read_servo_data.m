%% --- Position_control --- %%

% This script sets the robot arm to position control 

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
%% ---- Load Control Table Addresses ---- %%
% Control table address is different in every Dynamixel model

ADDR_PRO_TORQUE_ENABLE       = 64;          % Torque Enable(64) determines Torque ON/OFF. Writing ‘1’ to Toque Enable’s address will turn on the Torque           
ADDR_PRO_GOAL_POSITION       = 116;         % Sets desired position
ADDR_PRO_PRESENT_POSITION    = 132;         % Present position of the servo
ADDR_PRO_OPERATING_MODE      = 11;          % Sets between Current, Velocity, Position Control Mode 
ADDR_PRO_VELOCITY            = 112;         % Controls the velocity of the servos

%% ---- Load Dynamixel Devices ---- %%
DXL_ID1_BaseRotation                = 11;          
DXL_ID2_Shoulder                    = 12;    
DXL_ID3_Elbow                       = 13;          
DXL_ID4_Wrist                       = 14;    
DXL_ID5_Gripper                     = 15;          

%% ---- Load Other Settings ---- %%
% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
BAUDRATE                    = 115200;
DEVICENAME                  = 'COM3';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'      
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold
DXL_VELOCITY                = 20000;

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% --- Initialize PortHandler Structs --- %%
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;             % Communication result

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position

% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

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
    dxl_ID1_angle_degree = (0.088 * typecast(double(dxl_ID1_present_position), 'double'));
    dxl_ID2_angle_degree = (0.088 * typecast(double(dxl_ID2_present_position), 'double'));
    dxl_ID3_angle_degree = (0.088 * typecast(double(dxl_ID3_present_position), 'double'));
    dxl_ID4_angle_degree = (0.088 * typecast(double(dxl_ID4_present_position), 'double'));
    dxl_ID5_angle_degree = (0.088 * typecast(double(dxl_ID5_present_position), 'double'));

    dxl_ID1_angle_radian = deg2rad(dxl_ID1_angle_degree);
    dxl_ID2_angle_radian = deg2rad(dxl_ID2_angle_degree);
    dxl_ID3_angle_radian = deg2rad(dxl_ID3_angle_degree);
    dxl_ID4_angle_radian = deg2rad(dxl_ID4_angle_degree);
    dxl_ID5_angle_radian = deg2rad(dxl_ID5_angle_degree);

    % Print out readings and conversions
    fprintf('[ID:%03d] Raw encoding: %.1f - Angle(Deg): %.4f - Angle(Rad): %.4f\n', DXL_ID1_BaseRotation, typecast(uint32(dxl_ID1_present_position), 'int32'), dxl_ID1_angle_degree, dxl_ID1_angle_radian);
    fprintf('[ID:%03d] Raw encoding: %.1f - Angle(Deg): %.4f - Angle(Rad): %.4f\n', DXL_ID2_Shoulder, typecast(uint32(dxl_ID2_present_position), 'int32'), dxl_ID2_angle_degree, dxl_ID2_angle_radian);
    fprintf('[ID:%03d] Raw encoding: %.1f - Angle(Deg): %.4f - Angle(Rad): %.4f\n', DXL_ID3_Elbow, typecast(uint32(dxl_ID3_present_position), 'int32'), dxl_ID3_angle_degree, dxl_ID3_angle_radian);
    fprintf('[ID:%03d] Raw encoding: %.1f - Angle(Deg): %.4f - Angle(Rad): %.4f\n', DXL_ID4_Wrist, typecast(uint32(dxl_ID4_present_position), 'int32'), dxl_ID4_angle_degree, dxl_ID4_angle_radian);
    fprintf('[ID:%03d] Raw encoding: %.1f - Angle(Deg): %.4f - Angle(Rad): %.4f\n', DXL_ID5_Gripper, typecast(uint32(dxl_ID5_present_position), 'int32'), dxl_ID5_angle_degree, dxl_ID5_angle_radian);
end
