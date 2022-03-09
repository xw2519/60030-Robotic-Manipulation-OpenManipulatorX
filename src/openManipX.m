%% --- openManipX --- %%

% This script sets up the robotic arm class for interactions

classdef openManipX
    %% --- Class variables --- %%
    properties(Constant)
        %% --- Dynamixel addresses --- %%
        ADDR_PRO_ACCELERATION       = 108;
        ADDR_PRO_TORQUE_ENABLE      = 64;           % Torque Enable(64) determines Torque ON/OFF. Writing ‘1’ to Toque Enable’s address will turn on the Torque           
        ADDR_PRO_GOAL_POSITION      = 116;          % Sets desired position
        ADDR_PRO_PRESENT_POSITION   = 132;          % Present position of the servo
        ADDR_PRO_OPERATING_MODE     = 11;           % Sets between Current, Velocity, Position Control Mode 
        ADDR_PRO_VELOCITY           = 112;          % Controls the velocity of the servos
        ADDR_MAX_POS                = 48; 
        ADDR_MIN_POS                = 52; 
        
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

        BYTE_LENGTH                 = 4;
        
        MIN_POS                     = [600, 600, 600, 600, 600]; % Min positions for each servo(in raw encoding format)
        MAX_POS                     = [3400, 3400, 3400, 3400, 3400]; % Max positions for each servo(in raw encoding format)
    end
    
    properties(GetAccess = private)
        DXL_ADDPARAM_RESULT
        DXL_GETDATA_RESULT
        GROUPREAD_NUM
        GROUPWRITE_NUM
        LIB_NAME
        PORT_NUM
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
            
            % Initiate group write object
            obj.DXL_ADDPARAM_RESULT = false;
            obj.DXL_GETDATA_RESULT = false;
            
            obj.GROUPREAD_NUM = groupSyncRead(obj.PORT_NUM, obj.PROTOCOL_VERSION, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION);
            obj.GROUPWRITE_NUM = groupSyncWrite(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.ADDR_PRO_GOAL_POSITION, obj.BYTE_LENGTH);

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
            % Try to broadcast ping the Dynamixel
            broadcastPing(obj.PORT_NUM, obj.PROTOCOL_VERSION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end

            fprintf('Detected Dynamixel : \n');
            for SERVO_ID = 0 : MAX_ID
              if getBroadcastPingResult(obj.PORT_NUM, obj.PROTOCOL_VERSION, SERVO_ID)
                fprintf('[ID:%03d]\n', SERVO_ID);
              end
            end
            
            % Move robotic arm initial starting position
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 15, obj.ADDR_PRO_GOAL_POSITION, 2382.0);
            pause(0.5)
            
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
            
            logger(mfilename, "Log: Robotic arm activated and ready for operations")
        end
        
        function delete(obj)
            logger(mfilename, "Log: Deactivating robotic arm")
            
            % Move to resting position
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 15, obj.ADDR_PRO_GOAL_POSITION, 1943.18);
            pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 14, obj.ADDR_PRO_GOAL_POSITION, 2048);
            pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 12, obj.ADDR_PRO_GOAL_POSITION, 2048);
            pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 13, obj.ADDR_PRO_GOAL_POSITION, 2000);
            pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, 11, obj.ADDR_PRO_GOAL_POSITION, 2048);
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
        
        %% --- Servo modes --- %%
        function position_control_mode(obj)    
            logger(mfilename, "Log: Setting Position Control mode") 
            
            % Put actuators into Position Control Mode
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_OPERATING_MODE, 3);            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_OPERATING_MODE, 3);            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_OPERATING_MODE, 3);            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_OPERATING_MODE, 3);           
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_OPERATING_MODE, 3);
            
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
        
        %% --- Servo settings --- %%
        function set_servo_speed_limit(obj, ID, DXL_VELOCITY)
            logger(mfilename, "Log: Setting servo speed limit") 
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_VELOCITY, DXL_VELOCITY);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d speed has been successfully limited \n', ID);
            end
            
            logger(mfilename, "Log: Servo speeds limited") 
        end
        
        function set_all_servo_speed_limits(obj, DXL_VELOCITY)
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
            
            logger(mfilename, "Log: Servo speeds limited") 
        end
        
        function set_servo_acceleration(obj, ID, DXL_ACCELERATION)
            % Assuming Time-based Profile
            % Range: 0 ~ 32737 where '0' represents an infinite acceleration time('0 [msec]').
            % Profile Acceleration(108, Acceleration time) will not exceed 50% of Profile Velocity (112, the time span to reach the velocity of the Profile) value.
            logger(mfilename, "Log: Setting servo acceleration limit") 
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_ACCELERATION, DXL_ACCELERATION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d speed has been successfully limited \n', ID);
            end
            
            logger(mfilename, "Log: Servo acceleration limit set")
        end
        
        function set_all_servo_acceleration_limits(obj, DXL_ACCELERATION)
            logger(mfilename, "Log: Setting servo acceleration limit") 
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_ACCELERATION, DXL_ACCELERATION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d speed has been successfully limited \n', obj.DXL_ID1_BaseRotation);
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_ACCELERATION, DXL_ACCELERATION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d speed has been successfully limited \n', obj.DXL_ID2_Shoulder);
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_ACCELERATION, DXL_ACCELERATION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d speed has been successfully limited \n', obj.DXL_ID3_Elbow);
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_ACCELERATION, DXL_ACCELERATION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d speed has been successfully limited \n', obj.DXL_ID4_Wrist);
            end
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_ACCELERATION, DXL_ACCELERATION); 
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('Log: Dynamixel #%d speed has been successfully limited \n', obj.DXL_ID5_Gripper);
            end
            
            logger(mfilename, "Log: Servo acceleration limit set") 
        end
        
        function set_all_servo_range_motion(obj)
           logger(mfilename, "Log: Servo range motion limiting initiated") 
            
           % Limiting max range
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_MAX_POS, obj.MAX_POS(1)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_MAX_POS, obj.MAX_POS(2)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_MAX_POS, obj.MAX_POS(3)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_MAX_POS, obj.MAX_POS(4)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_MAX_POS, obj.MAX_POS(5)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end

           % Limiting min range
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_MIN_POS, obj.MIN_POS(1)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_MIN_POS, obj.MIN_POS(2)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_MIN_POS, obj.MIN_POS(3)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_MIN_POS, obj.MIN_POS(4)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_MIN_POS, obj.MIN_POS(5)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('Log: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           logger(mfilename, "Log: Servo range motion limiting completed") 
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
        
        function write_angles_to_all_servos(obj, ID1_ANGLE, ID2_ANGLE, ID3_ANGLE, ID4_ANGLE)
            msg = append('Writing encoder value: ', string(ENCODER_VAL), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg) % Log
            constant = 45/512;
            % Convert to raw encoder values
            ID1_VALUE = ID1_ANGLE / constant;
            ID2_VALUE = ID2_ANGLE / constant;
            ID3_VALUE = ID3_ANGLE / constant;
            ID4_VALUE = ID4_ANGLE / constant;

            % Add goal position values to the Syncwrite storage
            obj.DXL_ADDPARAM_RESULT = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID1_BaseRotation, typecast(int32(ID1_VALUE), 'uint32'), obj.LEN_PRO_GOAL_POSITION);
            if obj.DXL_ADDPARAM_RESULT ~= true
                fprintf('Log: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID1_BaseRotation);
                return;
            end
            
            obj.DXL_ADDPARAM_RESULT = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID2_Shoulder, typecast(int32(ID2_VALUE), 'uint32'), obj.LEN_PRO_GOAL_POSITION);
            if obj.DXL_ADDPARAM_RESULT ~= true
                fprintf('Log: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID2_Shoulder);
                return;
            end
            
            obj.DXL_ADDPARAM_RESULT = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID3_Elbow, typecast(int32(ID3_VALUE), 'uint32'), obj.LEN_PRO_GOAL_POSITION);
            if obj.DXL_ADDPARAM_RESULT ~= true
                fprintf('Log: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID3_Elbow);
                return;
            end
            
            obj.DXL_ADDPARAM_RESULT = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID4_Wrist, typecast(int32(ID4_VALUE), 'uint32'), obj.LEN_PRO_GOAL_POSITION);
            if obj.DXL_ADDPARAM_RESULT ~= true
                fprintf('Log: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID4_Wrist);
                return;
            end
            
            logger(mfilename, "Log: All angles converted and stored. Attempting to write simultaneously to all servos")
            
            % Write in sync to all servo
            groupSyncWriteTxPacket(obj.GROUPWRITE_NUM); 
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
        end

        %% --- Gripper functions --- %%
        function open_gripper(obj)
            logger(mfilename, "Log: Opening gripper")
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_GOAL_POSITION, 1943.18);
            pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            logger(mfilename, "Log: Gripper opened") 
        end
        
        function close_gripper(obj)
            logger(mfilename, "Closing gripper")
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_GOAL_POSITION, 2382.0);
            pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            logger(mfilename, "Gripper closed") 
        end
        
        %% --- Task functions --- %%
        function pick_up_cube_at_coord(obj, P_X, P_Y, P_Z, PHI)            
            % Calculate IK
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = IK_with_PHI(P_X, P_Y, (P_Z + 0.025), PHI);
            [SERVO_THETA_1_GRIP, SERVO_THETA_2_GRIP, SERVO_THETA_3_GRIP, SERVO_THETA_4_GRIP] = IK_with_PHI(P_X, P_Y, P_Z, PHI);
            
            % Verify angles are not complex i.e. reachable angles
            assert(isreal(SERVO_THETA_1_ABOVE_CUBE), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_ABOVE_CUBE), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_ABOVE_CUBE), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_ABOVE_CUBE), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_GRIP), "Fatal error: THETA 1 angle to grip cube is unreachable");
            assert(isreal(SERVO_THETA_2_GRIP), "Fatal error: THETA 2 angle to grip cube is unreachable");
            assert(isreal(SERVO_THETA_3_GRIP), "Fatal error: THETA 3 angle to grip cube is unreachable");
            assert(isreal(SERVO_THETA_4_GRIP), "Fatal error: THETA 4 angle to grip cube is unreachable");
            
            % Move to coordinate directly above cube by 0.025 m
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            open_gripper(obj);
            
            pause(3);
            
            % Move arm down 
            write_angles_to_all_servos(obj, SERVO_THETA_1_GRIP, SERVO_THETA_2_GRIP, SERVO_THETA_3_GRIP, SERVO_THETA_4_GRIP);
            
            % Grip the cube 
            close_gripper(obj); 
            
            % Move arm back up
            % write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            % gonna try to not have this for now so we know what coordinate
            % we are on
            pause(3);
        end
        
        function pick_up_cube_at(arm_obj, trajectory_obj, ROW, COLUMN, PHI)
            % Get board location specified 
            [P_X, P_Y, P_Z] = get_board_location(trajectory_obj, ROW, COLUMN);
            
            P_Z_WITH_CUBE = P_Z + 0.0125;
            
            % Modify P_Z height to grip the cube at 3/4 the gripper's
            % length to avoid gripper the platform as well
            % Shift the designated height upward by 0.00965
            % P_Z_CALIBRATED = P_Z_WITH_CUBE + 0.00965; 
            P_Z_CALIBRATED = P_Z_WITH_CUBE;
            
            % Calculate IK
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = IK_with_PHI(P_X, P_Y, (P_Z_CALIBRATED + 0.025), PHI);
            [SERVO_THETA_1_GRIP, SERVO_THETA_2_GRIP, SERVO_THETA_3_GRIP, SERVO_THETA_4_GRIP] = IK_with_PHI(P_X, P_Y, P_Z_CALIBRATED, PHI);
            
            % Verify angles are not complex i.e. reachable angles
            assert(isreal(SERVO_THETA_1_ABOVE_CUBE), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_ABOVE_CUBE), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_ABOVE_CUBE), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_ABOVE_CUBE), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_GRIP), "Fatal error: THETA 1 angle to grip cube is unreachable");
            assert(isreal(SERVO_THETA_2_GRIP), "Fatal error: THETA 2 angle to grip cube is unreachable");
            assert(isreal(SERVO_THETA_3_GRIP), "Fatal error: THETA 3 angle to grip cube is unreachable");
            assert(isreal(SERVO_THETA_4_GRIP), "Fatal error: THETA 4 angle to grip cube is unreachable");
            
            % Move to coordinate directly above cube by 0.025 m
            write_angles_to_all_servos(arm_obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            open_gripper(obj);
            
            pause(3);
            
            % Move arm down 
            write_angles_to_all_servos(arm_obj, SERVO_THETA_1_GRIP, SERVO_THETA_2_GRIP, SERVO_THETA_3_GRIP, SERVO_THETA_4_GRIP);
            
            % Grip the cube 
            close_gripper(obj);
            
            % Move arm back up
            write_angles_to_all_servos(arm_obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            pause(3);
        end
        
        function drop_cube_at_coord(obj, P_X, P_Y, P_Z, PHI) 
            % Calculate IK
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = IK_with_PHI(P_X, P_Y, (P_Z + 0.025), PHI);
            [SERVO_THETA_1_DROP, SERVO_THETA_2_DROP, SERVO_THETA_3_DROP, SERVO_THETA_4_DROP] = IK_with_PHI(P_X, P_Y, P_Z, PHI);
            
            % Verify angles are not complex i.e. reachable angles
            assert(isreal(SERVO_THETA_1_ABOVE_CUBE), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_ABOVE_CUBE), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_ABOVE_CUBE), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_ABOVE_CUBE), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_DROP), "Fatal error: THETA 1 angle to drop cube is unreachable");
            assert(isreal(SERVO_THETA_2_DROP), "Fatal error: THETA 2 angle to drop cube is unreachable");
            assert(isreal(SERVO_THETA_3_DROP), "Fatal error: THETA 3 angle to drop cube is unreachable");
            assert(isreal(SERVO_THETA_4_DROP), "Fatal error: THETA 4 angle to drop cube is unreachable");
            
            % Move to coordinate directly above cube by 0.025 m
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            
            % Move arm down 
            write_angles_to_all_servos(obj, SERVO_THETA_1_DROP, SERVO_THETA_2_DROP, SERVO_THETA_3_DROP, SERVO_THETA_4_DROP);
            
            % Drop the cube 
            open_gripper(obj);
            
            pause(3);
            
            % Move arm back up
            % write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            % don't want to do this step as we won't know what coords the
            % arm is at
        end

        function drop_cube_at(arm_obj, trajectory_obj, ROW, COLUMN, PHI)
            % Get board location specified 
            [P_X, P_Y, P_Z] = get_board_location(trajectory_obj, ROW, COLUMN);
            
            % Modify P_Z to include height of cube
            P_Z_WITH_CUBE = P_Z + 0.0125;
            
            % Modify P_Z height to grip the cube at 3/4 the gripper's
            % length to avoid gripper the platform as well
            % Shift the designated height upward by 0.00965
            % P_Z_CALIBRATED = P_Z_WITH_CUBE + 0.00965; 
            P_Z_CALIBRATED = P_Z_WITH_CUBE;
            
            % Calculate IK
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = IK_with_PHI(P_X, P_Y, (P_Z_CALIBRATED + 0.025), PHI);
            [SERVO_THETA_1_DROP, SERVO_THETA_2_DROP, SERVO_THETA_3_DROP, SERVO_THETA_4_DROP] = IK_with_PHI(P_X, P_Y, P_Z_CALIBRATED, PHI);
            
            % Verify angles are not complex i.e. reachable angles
            assert(isreal(SERVO_THETA_1_ABOVE_CUBE), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_ABOVE_CUBE), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_ABOVE_CUBE), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_ABOVE_CUBE), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_DROP), "Fatal error: THETA 1 angle to drop cube is unreachable");
            assert(isreal(SERVO_THETA_2_DROP), "Fatal error: THETA 2 angle to drop cube is unreachable");
            assert(isreal(SERVO_THETA_3_DROP), "Fatal error: THETA 3 angle to drop cube is unreachable");
            assert(isreal(SERVO_THETA_4_DROP), "Fatal error: THETA 4 angle to drop cube is unreachable");
            
            % Move to coordinate directly above cube by 0.025 m
            write_angles_to_all_servos(arm_obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            
            % Move arm down 
            write_angles_to_all_servos(arm_obj, SERVO_THETA_1_DROP, SERVO_THETA_2_DROP, SERVO_THETA_3_DROP, SERVO_THETA_4_DROP);
            
            % Drop the cube 
            open_gripper(obj);
            
            % Move arm back up
            write_angles_to_all_servos(arm_obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
        end
        
        function move_cube_coord(obj, P_X1, P_Y1, P_Z1, P_X2, P_Y2, P_Z2)
            % Modify P_Z height to grip the cube at 3/4 the gripper's
            % length to avoid gripper the platform as well
            % Shift the designated height upward by 0.00965
            %P_Z1 = P_Z1 + 0.00965;
            %P_Z2 = P_Z2 + 0.00965;
            
            % Assuming we are inputting cube coord centres then we have to
            % shift up by 0.00025 to allow the gripper to not grip the cube
            % holder
            P_Z1 = P_Z1 + 0.00025;
            P_Z2 = P_Z2 + 0.00025;
            phi = -90;
            pick_up_cube_at_coord(obj, P_X1, P_Y1, P_Z1, phi);
            drop_cube_at_coord(obj, P_X2, P_Y2, P_Z2, phi);
        end
        
        function rotate_cube_forward_at_coord(obj, P_X, P_Y, P_Z)
            phi = 0;
            [SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z + 0.025, phi);
            [SERVO_THETA_1_PRE_ROTATION, SERVO_THETA_2_PRE_ROTATION, SERVO_THETA_3_PRE_ROTATION, SERVO_THETA_4_PRE_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z, phi);
            phi = -90;
            [SERVO_THETA_1_ABOVE_POST_ROTATION, SERVO_THETA_2_ABOVE_POST_ROTATION, SERVO_THETA_3_ABOVE_POST_ROTATION, SERVO_THETA_4_ABOVE_POST_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z + 0.025, phi);
            [SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z, phi);
            
            
            % Verify angles are not complex i.e. reachable angles
            assert(isreal(SERVO_THETA_1_ABOVE_PRE_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_ABOVE_PRE_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_ABOVE_PRE_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_ABOVE_PRE_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_PRE_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_PRE_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_PRE_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_PRE_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_ABOVE_POST_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_ABOVE_POST_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_ABOVE_POST_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_ABOVE_POST_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_POST_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_POST_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_POST_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_POST_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            % Move to coordinate directly above cube by 0.025 m
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION);
            pause(3);
            open_gripper(obj);
            pause(3);
            % - Pick up cube at position with phi = 0 
            write_angles_to_all_servos(obj, SERVO_THETA_1_PRE_ROTATION, SERVO_THETA_2_PRE_ROTATION, SERVO_THETA_3_PRE_ROTATION, SERVO_THETA_4_PRE_ROTATION);
            pause(3);
            close_gripper(obj);
            pause(3);
            % - Move upward 
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION);
            paise(3);
            % - Move cube to the exact same position with phi = -90
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_POST_ROTATION, SERVO_THETA_2_ABOVE_POST_ROTATION, SERVO_THETA_3_ABOVE_POST_ROTATION, SERVO_THETA_4_ABOVE_POST_ROTATION);
            pause(3);
            % Drop cube at intended position
            write_angles_to_all_servos(obj, SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION);
            pause(3);
            open_gripper(obj);
            pause(3);
        end
        
        function rotate_cube_backward_at_coord(obj, P_X, P_Y, P_Z)
            phi = -90;
            [SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z + 0.025, phi);
            [SERVO_THETA_1_PRE_ROTATION, SERVO_THETA_2_PRE_ROTATION, SERVO_THETA_3_PRE_ROTATION, SERVO_THETA_4_PRE_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z, phi);
            phi = 0;
            [SERVO_THETA_1_ABOVE_POST_ROTATION, SERVO_THETA_2_ABOVE_POST_ROTATION, SERVO_THETA_3_ABOVE_POST_ROTATION, SERVO_THETA_4_ABOVE_POST_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z + 0.025, phi);
            [SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z, phi);
            
            
            % Verify angles are not complex i.e. reachable angles
            assert(isreal(SERVO_THETA_1_ABOVE_PRE_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_ABOVE_PRE_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_ABOVE_PRE_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_ABOVE_PRE_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_PRE_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_PRE_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_PRE_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_PRE_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_ABOVE_POST_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_ABOVE_POST_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_ABOVE_POST_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_ABOVE_POST_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_POST_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_POST_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_POST_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_POST_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            % Move to coordinate directly above cube by 0.025 m
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION);
            pause(3);
            open_gripper(obj);
            pause(3);
            % - Pick up cube at position with phi = 0 
            write_angles_to_all_servos(obj, SERVO_THETA_1_PRE_ROTATION, SERVO_THETA_2_PRE_ROTATION, SERVO_THETA_3_PRE_ROTATION, SERVO_THETA_4_PRE_ROTATION);
            pause(3);
            close_gripper(obj);
            pause(3);
            % - Move upward 
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION);
            paise(3);
            % - Move cube to the exact same position with phi = -90
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_POST_ROTATION, SERVO_THETA_2_ABOVE_POST_ROTATION, SERVO_THETA_3_ABOVE_POST_ROTATION, SERVO_THETA_4_ABOVE_POST_ROTATION);
            pause(3);
            % Drop cube at intended position
            write_angles_to_all_servos(obj, SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION);
            pause(3);
            open_gripper(obj);
            pause(3);
        end
        
        function rotate_cube_forward_at(obj, ROW, COLUMN)
            [P_X, P_Y, P_Z] = get_board_location(trajectory_obj, ROW, COLUMN);
            
            P_Z_WITH_CUBE = P_Z + 0.0125;
            
            % Modify P_Z height to grip the cube at 3/4 the gripper's
            % length to avoid gripper the platform as well
            % Shift the designated height upward by 0.00965
            % P_Z_CALIBRATED = P_Z_WITH_CUBE + 0.00965; 
            P_Z = P_Z_WITH_CUBE;
            
            
            phi = 0;
            [SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z + 0.025, phi);
            [SERVO_THETA_1_PRE_ROTATION, SERVO_THETA_2_PRE_ROTATION, SERVO_THETA_3_PRE_ROTATION, SERVO_THETA_4_PRE_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z, phi);
            phi = -90;
            [SERVO_THETA_1_ABOVE_POST_ROTATION, SERVO_THETA_2_ABOVE_POST_ROTATION, SERVO_THETA_3_ABOVE_POST_ROTATION, SERVO_THETA_4_ABOVE_POST_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z + 0.025, phi);
            [SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z, phi);
            
            
            % Verify angles are not complex i.e. reachable angles
            assert(isreal(SERVO_THETA_1_ABOVE_PRE_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_ABOVE_PRE_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_ABOVE_PRE_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_ABOVE_PRE_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_PRE_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_PRE_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_PRE_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_PRE_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_ABOVE_POST_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_ABOVE_POST_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_ABOVE_POST_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_ABOVE_POST_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_POST_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_POST_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_POST_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_POST_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            % Move to coordinate directly above cube by 0.025 m
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION);
            pause(3);
            open_gripper(obj);
            pause(3);
            % - Pick up cube at position with phi = 0 
            write_angles_to_all_servos(obj, SERVO_THETA_1_PRE_ROTATION, SERVO_THETA_2_PRE_ROTATION, SERVO_THETA_3_PRE_ROTATION, SERVO_THETA_4_PRE_ROTATION);
            pause(3);
            close_gripper(obj);
            pause(3);
            % - Move upward 
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION);
            paise(3);
            % - Move cube to the exact same position with phi = -90
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_POST_ROTATION, SERVO_THETA_2_ABOVE_POST_ROTATION, SERVO_THETA_3_ABOVE_POST_ROTATION, SERVO_THETA_4_ABOVE_POST_ROTATION);
            pause(3);
            % Drop cube at intended position
            write_angles_to_all_servos(obj, SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION);
            pause(3);
            open_gripper(obj);
            pause(3); 
        end
        
        function rotate_cube_backward_at(obj, ROW, COLUMN)
            [P_X, P_Y, P_Z] = get_board_location(trajectory_obj, ROW, COLUMN);
            
            P_Z_WITH_CUBE = P_Z + 0.0125;
            
            % Modify P_Z height to grip the cube at 3/4 the gripper's
            % length to avoid gripper the platform as well
            % Shift the designated height upward by 0.00965
            % P_Z_CALIBRATED = P_Z_WITH_CUBE + 0.00965; 
            P_Z = P_Z_WITH_CUBE;
            
            
            phi = -90;
            [SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z + 0.025, phi);
            [SERVO_THETA_1_PRE_ROTATION, SERVO_THETA_2_PRE_ROTATION, SERVO_THETA_3_PRE_ROTATION, SERVO_THETA_4_PRE_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z, phi);
            phi = 0;
            [SERVO_THETA_1_ABOVE_POST_ROTATION, SERVO_THETA_2_ABOVE_POST_ROTATION, SERVO_THETA_3_ABOVE_POST_ROTATION, SERVO_THETA_4_ABOVE_POST_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z + 0.025, phi);
            [SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION] = IK_with_PHI(P_X, P_Y, P_Z, phi);
            
            
            % Verify angles are not complex i.e. reachable angles
            assert(isreal(SERVO_THETA_1_ABOVE_PRE_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_ABOVE_PRE_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_ABOVE_PRE_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_ABOVE_PRE_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_PRE_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_PRE_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_PRE_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_PRE_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_ABOVE_POST_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_ABOVE_POST_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_ABOVE_POST_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_ABOVE_POST_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            assert(isreal(SERVO_THETA_1_POST_ROTATION), "Fatal error: THETA 1 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_2_POST_ROTATION), "Fatal error: THETA 2 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_3_POST_ROTATION), "Fatal error: THETA 3 angle above cube is unreachable");
            assert(isreal(SERVO_THETA_4_POST_ROTATION), "Fatal error: THETA 4 angle above cube is unreachable");
            
            % Move to coordinate directly above cube by 0.025 m
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION);
            pause(3);
            open_gripper(obj);
            pause(3);
            % - Pick up cube at position with phi = 0 
            write_angles_to_all_servos(obj, SERVO_THETA_1_PRE_ROTATION, SERVO_THETA_2_PRE_ROTATION, SERVO_THETA_3_PRE_ROTATION, SERVO_THETA_4_PRE_ROTATION);
            pause(3);
            close_gripper(obj);
            pause(3);
            % - Move upward 
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION);
            paise(3);
            % - Move cube to the exact same position with phi = -90
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_POST_ROTATION, SERVO_THETA_2_ABOVE_POST_ROTATION, SERVO_THETA_3_ABOVE_POST_ROTATION, SERVO_THETA_4_ABOVE_POST_ROTATION);
            pause(3);
            % Drop cube at intended position
            write_angles_to_all_servos(obj, SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION);
            pause(3);
            open_gripper(obj);
            pause(3); 
        end
    end
end