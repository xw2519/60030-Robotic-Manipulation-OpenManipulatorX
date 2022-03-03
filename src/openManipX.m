%% --- openManipX --- %%

% This script sets up the robotic arm class for interactions

classdef openManipX
    %% --- Class variables --- %%
    properties(Constant)
        %% --- Dynamixel addresses --- %%
        ADDR_PRO_TORQUE_ENABLE      = 64;           % Torque Enable(64) determines Torque ON/OFF. Writing ‘1’ to Toque Enable’s address will turn on the Torque           
        ADDR_PRO_GOAL_POSITION      = 116;          % Sets desired position
        ADDR_PRO_PRESENT_POSITION   = 132;          % Present position of the servo
        ADDR_PRO_OPERATING_MODE     = 11;           % Sets between Current, Velocity, Position Control Mode 
        ADDR_PRO_VELOCITY           = 112;          % Controls the velocity of the servos

        %% ---- Load Dynamixel Devices ---- %%
        DXL_ID1_BaseRotation        = 11;          
        DXL_ID2_Shoulder            = 12;    
        DXL_ID3_Elbow               = 13;          
        DXL_ID4_Wrist               = 14;    
        DXL_ID5_Gripper             = 15;   
        
        %% ---- Load Other Settings ---- %%
        % Protocol version
        PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

        % Default setting
        BAUDRATE                    = 1000000;
        DEVICENAME                  = 'COM6';       % Check which port is being used on your controller
                                                    % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'      
        TORQUE_ENABLE               = 1;            % Value for enabling the torque
        TORQUE_DISABLE              = 0;            % Value for disabling the torque
        DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
        DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold
        DXL_VELOCITY                = 20000;

        COMM_SUCCESS                = 0;            % Communication Success result value
        COMM_TX_FAIL                = -1001;        % Communication Tx Failed

        BYTE_LENGTH                 = 4;
    end
    
    properties(GetAccess=private)
        PORT_NUM
        LIB_NAME
        GROUPWRITE_NUM
    end
    
    % properties
    %     ID1_BaseRotation_Encoder_Value {mustBeNumeric}
    %     ID2_Shoulder_Encoder_Value {mustBeNumeric}
    %     ID3_Elbow_Encoder_Value {mustBeNumeric}
    %     ID4_Wrist_Encoder_Value {mustBeNumeric}
    %     ID5_Gripper_Encoder_Value {mustBeNumeric}
    % end
    
    %% --- Class methods --- %%
    methods
        %% --- Class constructor and deconstructor --- %%
        function obj = openManipX()
            logger(mfilename, "Log: Activating robotic arm")
            
            % Load object library
            obj.LIB_NAME = '';

            if strcmp(computer, 'PCWIN')
              obj.LIB_NAME = 'dxl_x86_c';
            elseif strcmp(computer, 'PCWIN64')
              obj.LIB_NAME = 'dxl_x64_c';
            elseif strcmp(computer, 'GLNX86')
              obj.LIB_NAME = 'libdxl_x86_c';
            elseif strcmp(computer, 'GLNXA64')
              obj.LIB_NAME = 'libdxl_x64_c';
            elseif strcmp(computer, 'MACI64')
              obj.LIB_NAME = 'libdxl_mac_c';
            end

            % Load Libraries
            if ~libisloaded(obj.LIB_NAME)
                [notfound, warnings] = loadlibrary(obj.LIB_NAME, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
            end

            % Set the port path
            % Get methods and members of PortHandlerLinux or PortHandlerWindows
            obj.PORT_NUM = portHandler(obj.DEVICENAME);

            % Initialize PacketHandler Structs
            packetHandler();

            % Open port
            if (openPort(obj.PORT_NUM))
                fprintf('Port Open\n');
            else
                unloadlibrary(obj.LIB_NAME);
                fprintf('Failed to open the port\n');
                input('Press any key to terminate...\n');
                return;
            end

            % Set port baudrate
            if (setBaudRate(obj.PORT_NUM, obj.BAUDRATE))
                fprintf('Baudrate Set\n');
            else
                unloadlibrary(obj.LIB_NAME);
                fprintf('Failed to change the baudrate!\n');
                input('Press any key to terminate...\n');
                return;
            end
            
            % Ping servos to verify connection
            DXL_ID1_MODEL_NUMBER = pingGetModelNum(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            fprintf('Log: [ID:%03d] ping Succeeded. Dynamixel model number : %d\n', obj.DXL_ID1_BaseRotation, DXL_ID1_MODEL_NUMBER);
            
            DXL_ID2_MODEL_NUMBER = pingGetModelNum(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            fprintf('Log: [ID:%03d] ping Succeeded. Dynamixel model number : %d\n', obj.DXL_ID2_Shoulder, DXL_ID2_MODEL_NUMBER);
            
            DXL_ID3_MODEL_NUMBER = pingGetModelNum(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            fprintf('Log: [ID:%03d] ping Succeeded. Dynamixel model number : %d\n', obj.DXL_ID3_Elbow, DXL_ID3_MODEL_NUMBER);
            
            DXL_ID4_MODEL_NUMBER = pingGetModelNum(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            fprintf('Log: [ID:%03d] ping Succeeded. Dynamixel model number : %d\n', obj.DXL_ID4_Wrist, DXL_ID4_MODEL_NUMBER);
            
            DXL_ID5_MODEL_NUMBER = pingGetModelNum(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            fprintf('Log: [ID:%03d] ping Succeeded. Dynamixel model number : %d\n', obj.DXL_ID5_Gripper, DXL_ID5_MODEL_NUMBER);
            
            % Move robotic arm initial starting position
            % write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 11, obj.ADDR_PRO_GOAL_POSITION, 2045);
            % pause(0.5)
            % write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 14, obj.ADDR_PRO_GOAL_POSITION, 2482);
            % pause(0.5)
            % write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 13, obj.ADDR_PRO_GOAL_POSITION, 3015);
            % pause(0.5)
            % write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 12, obj.ADDR_PRO_GOAL_POSITION, 715);
            % pause(3)
            
            % if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
            %     printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            % elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
            %     printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            % end
            
            % Initiate group write object
            obj.GROUPWRITE_NUM = groupSyncWrite(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.ADDR_PRO_GOAL_POSITION, BYTE_LENGTH);
            
            logger(mfilename, "Log: Robotic arm activated and ready for operations")
        end
        
        function delete(obj)
            logger(mfilename, "Log: Deactivating robotic arm")
            
            % Move to resting position
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 11, obj.ADDR_PRO_GOAL_POSITION, 2045);
            pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 14, obj.ADDR_PRO_GOAL_POSITION, 2482);
            pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 13, obj.ADDR_PRO_GOAL_POSITION, 3015);
            pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 12, obj.ADDR_PRO_GOAL_POSITION, 715);
            pause(2)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            % Disable Dynamixel Torque
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 11, obj.ADDR_PRO_TORQUE_ENABLE, obj.TORQUE_DISABLE);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully disconnected \n', 11);
            end

            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 12, obj.ADDR_PRO_TORQUE_ENABLE, obj.TORQUE_DISABLE);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully disconnected \n', 12);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 13, obj.ADDR_PRO_TORQUE_ENABLE, obj.TORQUE_DISABLE);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully disconnected \n', 13);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 14, obj.ADDR_PRO_TORQUE_ENABLE, obj.TORQUE_DISABLE);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully disconnected \n', 14);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 15, obj.ADDR_PRO_TORQUE_ENABLE, obj.TORQUE_DISABLE);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully disconnected \n', 15);
            end
            
            pause(2)

            % Close port
            closePort(obj.PORT_NUM);
            fprintf('Port Closed \n');

            % Unload Library
            unloadlibrary(obj.LIB_NAME);

            close all;
            clear;
            
            logger(mfilename, "Log: Robotic arm deactivated")
        end
        
        %% --- Servo settings --- %%
        function position_control_mode(obj)    
            logger(mfilename, "Log: Setting Position Control mode") 
            
            % Put actuators into Position Control Mode
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_OPERATING_MODE, 3);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully set to Position Control Mode \n', obj.DXL_ID1_BaseRotation);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_OPERATING_MODE, 3);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully set to Position Control Mode \n', obj.DXL_ID2_Shoulder);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_OPERATING_MODE, 3);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully set to Position Control Mode \n', obj.DXL_ID3_Elbow);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_OPERATING_MODE, 3);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully set to Position Control Mode \n', obj.DXL_ID4_Wrist);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_OPERATING_MODE, 3);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully set to Position Control Mode \n', obj.DXL_ID5_Gripper);
            end
            
            logger(mfilename, "Log: Position Control mode activated")
        end
        
        function toggle_torque(obj, torque_toggle)
            logger(mfilename, "Log: Toggling Torque Mode")
            
           % Enable/Disable Dynamixel Torque
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully set to torque mode \n', obj.DXL_ID1_BaseRotation);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully set to torque mode \n', obj.DXL_ID2_Shoulder);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully set to torque mode \n', obj.DXL_ID3_Elbow);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully set to torque mode \n', obj.DXL_ID4_Wrist);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d has been successfully set to torque mode \n', obj.DXL_ID5_Gripper);
            end
            
            logger(mfilename, "Log: Torque Mode toggled successfully")
        end
   
        function set_servo_speed_limits(obj, DXL_VELOCITY)
            logger(mfilename, "Log: Servo speeds limited") 
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_VELOCITY, DXL_VELOCITY);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d speed has been successfully limited \n', obj.DXL_ID1_BaseRotation);
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_VELOCITY, DXL_VELOCITY);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d speed has been successfully limited \n', obj.DXL_ID2_Shoulder);
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_VELOCITY, DXL_VELOCITY);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d speed has been successfully limited \n', obj.DXL_ID3_Elbow);
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_VELOCITY, DXL_VELOCITY);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d speed has been successfully limited \n', obj.DXL_ID4_Wrist);
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_VELOCITY, DXL_VELOCITY); 
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d speed has been successfully limited \n', obj.DXL_ID5_Gripper);
            end
            
            logger(mfilename, "Log: Servo speeds limited") %
        end
    
        %% --- Write and read functions --- %%
        function write_raw_encoder(obj, SERVO_ID, ENCODER_VAL)
            msg = append('Log: Writing encoder value: ', string(ENCODER_VAL), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg)

            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, SERVO_ID, obj.ADDR_PRO_GOAL_POSITION, ENCODER_VAL);

            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            msg = append('Log: Completed writing encoder value: ', string(ENCODER_VAL), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg)
        end
        
        function write_angles(obj, SERVO_ID, ANGLE)
            msg = append('Log: Writing angle: ', string(ANGLE), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg)
            
            % Convert degree to raw encoder value
            ENCODER_VAL = ANGLE / 0.088;
            
            % Write to servo
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, SERVO_ID, obj.ADDR_PRO_GOAL_POSITION, ENCODER_VAL);

            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            msg = append('Log: Completed writing encoder value: ', string(ENCODER_VAL), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg)
        end
        
        function write_angles_to_all_servos(obj, ID1_ANGLE, ID2_ANGLE, ID3_ANGLE, ID4_ANGLE, ID5_ANGLE)
            msg = append('Writing encoder value: ', string(ENCODER_VAL), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg) % Log
            
            % Convert to raw encoder values
            ID1_VALUE = ID1_ANGLE / 0.088;
            ID2_VALUE = ID2_ANGLE / 0.088;
            ID3_VALUE = ID3_ANGLE / 0.088;
            ID4_VALUE = ID4_ANGLE / 0.088;
            ID5_VALUE = ID5_ANGLE / 0.088;

            % Add goal position values to the Syncwrite storage
            dxl_addparam_result = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID1_BaseRotation, typecast(int32(ID1_VALUE), 'uint32'), obj.LEN_PRO_GOAL_POSITION);
            if dxl_addparam_result ~= true
                fprintf('Log: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID1_BaseRotation);
                return;
            end
            
            dxl_addparam_result = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID2_Shoulder, typecast(int32(ID2_VALUE), 'uint32'), obj.LEN_PRO_GOAL_POSITION);
            if dxl_addparam_result ~= true
                fprintf('Log: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID2_Shoulder);
                return;
            end
            
            dxl_addparam_result = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID3_Elbow, typecast(int32(ID3_VALUE), 'uint32'), obj.LEN_PRO_GOAL_POSITION);
            if dxl_addparam_result ~= true
                fprintf('Log: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID3_Elbow);
                return;
            end
            
            dxl_addparam_result = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID4_Wrist, typecast(int32(ID4_VALUE), 'uint32'), obj.LEN_PRO_GOAL_POSITION);
            if dxl_addparam_result ~= true
                fprintf('Log: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID4_Wrist);
                return;
            end
            
            dxl_addparam_result = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID5_Gripper, typecast(int32(ID5_VALUE), 'uint32'), obj.LEN_PRO_GOAL_POSITION);
            if dxl_addparam_result ~= true
                fprintf('Log: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID5_Gripper);
                return;
            end
            
            logger(mfilename, "Log: All angles converted and stored. Attempting to write simultaneously to all servos")
            
            groupSyncWriteTxPacket(obj.GROUPWRITE_NUM); % Syncwrite goal position
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            % Clear syncwrite parameter storage
            groupSyncWriteClearParam(obj.GROUPWRITE_NUM);
                
            logger(mfilename, "Log: Successfully written to all servos")
        end
        
        function [ID11_Value, ID12_Value, ID13_Value, ID14_Value, ID15_Value] = read_all_servo_raw_encoder(obj)
            logger(mfilename, "Log: Reading all servo raw encoder values")
            
            ID11_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            ID12_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            ID13_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            ID14_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            ID15_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            logger(mfilename, "Log: Successfully read all servo raw encoder values")
        end
        
        function [ID11_Angle, ID12_Angle, ID13_Angle, ID14_Angle, ID15_Angle] = read_all_servo_angles(obj)
            logger(mfilename, "Log: Reading all servo raw encoder values")
            
            ID11_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_PRESENT_POSITION);
            ID12_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_PRESENT_POSITION);
            ID13_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_PRESENT_POSITION);
            ID14_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_PRESENT_POSITION);
            ID15_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_PRESENT_POSITION);
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            ID11_Angle = (0.088 * typecast(single(ID11_Value), 'single'));
            ID12_Angle = (0.088 * typecast(single(ID12_Value), 'single'));
            ID13_Angle = (0.088 * typecast(single(ID13_Value), 'single'));
            ID14_Angle = (0.088 * typecast(single(ID14_Value), 'single'));
            ID15_Angle = (0.088 * typecast(single(ID15_Value), 'single'));
            
            logger(mfilename, "Log: Successfully read all servo raw encoder values")
        end

        %% --- Gripper functions --- %%
        function open_gripper(obj)
            logger(mfilename, "Log: Opening gripper")
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_GOAL_POSITION, 4280.0);
            % pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            logger(mfilename, "Log: Gripper opened") 
        end
        
        function close_gripper(obj)
            logger(mfilename, "Closing gripper")
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_GOAL_POSITION, 3481.0);
            % pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            logger(mfilename, "Gripper closed") 
        end
    end
end