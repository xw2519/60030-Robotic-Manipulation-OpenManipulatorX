%% --- openManipX --- %%

% This script setup the robotic arm class for interactions

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
        DEVICENAME                  = 'COM4';       % Check which port is being used on your controller
                                                    % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'      
        TORQUE_ENABLE               = 1;            % Value for enabling the torque
        TORQUE_DISABLE              = 0;            % Value for disabling the torque
        DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
        DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold
        DXL_VELOCITY                = 20000;

        COMM_SUCCESS                = 0;            % Communication Success result value
        COMM_TX_FAIL                = -1001;        % Communication Tx Failed
    end
    
    properties(GetAccess=private)
        PORT_NUM
    end
    
    %% --- Class methods --- %%
    methods
        %% --- Class constructor and deconstructor --- %%
        function obj = openManipX()
            % Clear terminal and load libraries
            clc;
            clear all;
            
            % Load object library
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

            % Set the port path
            % Get methods and members of PortHandlerLinux or PortHandlerWindows
            obj.PORT_NUM = portHandler(obj.DEVICENAME);

            % Initialize PacketHandler Structs
            packetHandler();

            % Open port
            if (openPort(obj.PORT_NUM))
                fprintf('Port Open\n');
            else
                unloadlibrary(lib_name);
                fprintf('Failed to open the port\n');
                input('Press any key to terminate...\n');
                return;
            end

            % Set port baudrate
            if (setBaudRate(obj.PORT_NUM, obj.BAUDRATE))
                fprintf('Baudrate Set\n');
            else
                unloadlibrary(lib_name);
                fprintf('Failed to change the baudrate!\n');
                input('Press any key to terminate...\n');
                return;
            end
        end
        
        function delete(obj)
           ...
        end
        
        %% --- Servo settings --- %%
        function position_control_mode(obj)    
            logger(mfilename, "Setting Position control") % Log
            
            % Put actuators into Position Control Mode
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_OPERATING_MODE, 3);
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_OPERATING_MODE, 3);
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_OPERATING_MODE, 3);
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_OPERATING_MODE, 3);
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_OPERATING_MODE, 3);
            
            % Error handling
            dxl_comm_result = getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION);

            if dxl_comm_result ~= obj.COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(obj.PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(obj.PROTOCOL_VERSION, dxl_error));
            end
            
            logger(mfilename, "Position control set") % Log
        end
        
        function toggle_torque(obj, torque_toggle)
            logger(mfilename, "Toggling torque mode") % Log
            
           % Enable/Disable Dynamixel Torque
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
        
            % Error handling
            dxl_comm_result = getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION);
            dxl_error = getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION);

            if dxl_comm_result ~= obj.COMM_SUCCESS
                fprintf('%s\n', getTxRxResult(obj.PROTOCOL_VERSION, dxl_comm_result));
            elseif dxl_error ~= 0
                fprintf('%s\n', getRxPacketError(obj.PROTOCOL_VERSION, dxl_error));
            end
            
            logger(mfilename, "Toggling torque mode successful") % Log
        end
   
        function set_servo_limits(obj, SERVO_ID, MIN_POS, MAX_POS)
            ...
        end
    
        %% --- Write and read functions --- %%
        function write(obj, SERVO_ID, ENCODER_VAL)
            msg = append('Writing encoder value: ', string(ENCODER_VAL), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg) % Log

            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, SERVO_ID, obj.ADDR_PRO_GOAL_POSITION, ENCODER_VAL);

            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            msg = append('Completed writing encoder value: ', string(ENCODER_VAL), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg) % Log
        end
        
        function write_to_all(obj, ENCODER_VAL)
            msg = append('Writing encoder value: ', string(ENCODER_VAL), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg) % Log

            ...
                
            logger(mfilename, msg) % Log
        end
        
        function raw_encoder_value = read(obj, SERVO_ID)
            ...
        end
    end
end