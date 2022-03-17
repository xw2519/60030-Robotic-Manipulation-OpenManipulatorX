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
        DEVICENAME                  = 'COM9';       % Check which port is being used on your controller
                                                    % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'      
        TORQUE_ENABLE               = 1;            % Value for enabling the torque
        TORQUE_DISABLE              = 0;            % Value for disabling the torque
        DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
        DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        DXL_MOVING_STATUS_THRESHOLD = 100;           % Dynamixel moving status threshold
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
        LEN_PRO_PRESENT_POSITION = 4
        LIB_NAME
        PORT_NUM
        
        % Gripper calibrate values 
        GRIPPER_CLOSE = 2470
        GRIPPER_OPEN = 1870
        
        % Pick up and drop cubes calibration values 
        ABOVE_CUBE_OFFSET = 0.09
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
            logger(mfilename, "Activating robotic arm")
            
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
                [notfound, warnings] = loadlibrary(obj.LIB_NAME, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', 'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h');
            end

            % Set the port path
            % Get methods and members of PortHandlerLinux or PortHandlerWindows
            obj.PORT_NUM = portHandler(obj.DEVICENAME);

            % Initialize PacketHandler Structs
            packetHandler();
            
            % Initiate group write object
            obj.DXL_ADDPARAM_RESULT = false;
            obj.DXL_GETDATA_RESULT = false;
            
            obj.GROUPREAD_NUM = groupSyncRead(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.ADDR_PRO_PRESENT_POSITION, 4);
            obj.GROUPWRITE_NUM = groupSyncWrite(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.ADDR_PRO_GOAL_POSITION, 4);

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
            for SERVO_ID = 11:15
              if getBroadcastPingResult(obj.PORT_NUM, obj.PROTOCOL_VERSION, SERVO_ID)
                fprintf('[ID:%03d]\n', SERVO_ID);
              end
            end
            
            % Move robotic arm initial starting position
            
            logger(mfilename, "Robotic arm activated and ready for operations")
        end
        
        function delete(obj)
            logger(mfilename, "Deactivating robotic arm")
            
            % Move to resting position
            obj.write_angles_to_all_servos(180, 180, 180, 180);
            %check_position_reached(obj, 180, 180, 180, 180);
            % obj.check_position_reached((180/0.088), (180/0.088), (180/0.088), (180/0.088));
            
            obj.write_angles_to_all_servos(180, 130, 216, 198);
            %check_position_reached(obj, 180, 130, 216, 198);
            % obj.check_position_reached((180/0.088), (130/0.088), (216/0.088), (198/0.088));
            
            obj.write_angles_to_all_servos(180, 77.70, 245.04, 219.90);
            %check_position_reached(obj, 180, 77.70, 245.04, 219.90);
            % obj.check_position_reached((180/0.088), (77.70/0.088), (245.04/0.088), (219.90/0.088));
            
            obj.write_angles_to_all_servos(180, 62.93, 265.87, 218.67);
            %check_position_reached(obj, 180, 62.93, 265.87, 218.67);
            % obj.check_position_reached((180/0.088), (62.93/0.088), (265.87/0.088), (218.67/0.088));
            
            % Disable Dynamixel Torque
            toggle_torque(obj, obj.TORQUE_DISABLE)
            
            % Close port
            closePort(obj.PORT_NUM);
            fprintf('Port Closed \n');

            % Unload Library
            unloadlibrary(obj.LIB_NAME);

            close all;
            clear;
            
            logger(mfilename, "Robotic arm successfully deactivated")
        end
        
        %% --- Check position 
        function check_position_reached(obj, goal_pos_1, goal_pos_2, goal_pos_3, goal_pos_4)
            constant = 360/4096;
            goal_pos_1 = typecast(uint32(goal_pos_1/constant), 'int32');
            goal_pos_2 = typecast(uint32(goal_pos_2/constant), 'int32'); 
            goal_pos_3 = typecast(uint32(goal_pos_3/constant), 'int32'); 
            goal_pos_4 = typecast(uint32(goal_pos_4/constant), 'int32'); 
            
            logger(mfilename, "Checking if servo positions reached")
            
            % Start timer and only adjust if a certain time has passed            
            % Loop until position is reached 
            tic; 
            
            while 1 
                dxl_1_present_position = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_PRESENT_POSITION);
                if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                    printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                    printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                end

                dxl_2_present_position = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_PRESENT_POSITION);
                if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                    printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                    printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                end
                
                dxl_3_present_position = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_PRESENT_POSITION);
                if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                    printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                    printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                end
                
                dxl_4_present_position = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_PRESENT_POSITION);
                if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                    printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                    printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                end

                if ~((abs(goal_pos_1 - typecast(uint32(dxl_1_present_position), 'int32')) > obj.DXL_MOVING_STATUS_THRESHOLD) || (abs(goal_pos_2 - typecast(uint32(dxl_2_present_position), 'int32')) > obj.DXL_MOVING_STATUS_THRESHOLD) || (abs(goal_pos_3 - typecast(uint32(dxl_3_present_position), 'int32')) > obj.DXL_MOVING_STATUS_THRESHOLD || (abs(goal_pos_4 - typecast(uint32(dxl_4_present_position), 'int32')) > obj.DXL_MOVING_STATUS_THRESHOLD)))
                    logger(mfilename, "Goal position reached")
                    fprintf('[Log]: Pos: #%d - #%d - #%d - #%d \n', goal_pos_1, goal_pos_2, goal_pos_3, goal_pos_4);
                    fprintf('[Log]: Pos: #%d - #%d - #%d - #%d \n', dxl_1_present_position, dxl_2_present_position, dxl_3_present_position, dxl_4_present_position);
                    break;
                end
            end
            
        end
        
        %% --- Servo modes --- %%
        function position_control_mode(obj)    
            logger(mfilename, "Setting Position Control mode") 
            
            % Put actuators into Position Control Mode
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_OPERATING_MODE, 3);            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_OPERATING_MODE, 3);            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_OPERATING_MODE, 3);            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_OPERATING_MODE, 3);           
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_OPERATING_MODE, 3);
            
            logger(mfilename, "Position Control mode activated")
        end
                
        function toggle_torque(obj, torque_toggle)
            logger(mfilename, "Toggling Torque Mode")
            
           % Enable/Disable Dynamixel Torque
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d disconnection failed \n', obj.DXL_ID1_BaseRotation);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d disconnection failed \n', obj.DXL_ID1_BaseRotation);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('[Log]: Dynamixel #%d has been successfully toggled \n', obj.DXL_ID1_BaseRotation);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d disconnection failed \n', obj.DXL_ID2_Shoulder);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d disconnection failed \n', obj.DXL_ID2_Shoulder);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('[Log]: Dynamixel #%d has been successfully toggled \n', obj.DXL_ID2_Shoulder);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d disconnection failed \n', obj.DXL_ID3_Elbow);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d disconnection failed \n', obj.DXL_ID3_Elbow);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('[Log]: Dynamixel #%d has been successfully toggled \n', obj.DXL_ID3_Elbow);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d disconnection failed \n', obj.DXL_ID4_Wrist);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d disconnection failed \n', obj.DXL_ID4_Wrist);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('[Log]: Dynamixel #%d has been successfully toggled \n', obj.DXL_ID4_Wrist);
            end
            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_TORQUE_ENABLE, torque_toggle);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d disconnection failed \n', obj.DXL_ID5_Gripper);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d disconnection failed \n', obj.DXL_ID5_Gripper);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('[Log]: Dynamixel #%d has been successfully toggled \n', obj.DXL_ID5_Gripper);
            end
            
            logger(mfilename, "Torque Mode successfully toggled")
        end
        
        %% --- Servo settings --- %%
        function set_servo_speed_limit(obj, ID, DXL_VELOCITY)
            logger(mfilename, "Setting servo speed limit") 
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_VELOCITY, DXL_VELOCITY);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d speed could not be limited \n', ID);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Log]: Dynamixel #%d speed could not be limited \n', ID);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('[Log]: Dynamixel #%d speed has been successfully limited \n', ID);
            end
            
            logger(mfilename, "Servo speeds limited") 
        end
        
        function set_all_servo_speed_limits(obj, DXL_VELOCITY)
            logger(mfilename, "Servo speeds limited") 
            
            set_servo_speed_limit(obj, obj.DXL_ID1_BaseRotation, DXL_VELOCITY);
            set_servo_speed_limit(obj, obj.DXL_ID2_Shoulder, DXL_VELOCITY);
            set_servo_speed_limit(obj, obj.DXL_ID3_Elbow, DXL_VELOCITY);
            set_servo_speed_limit(obj, obj.DXL_ID4_Wrist, DXL_VELOCITY);
            set_servo_speed_limit(obj, obj.DXL_ID5_Gripper, DXL_VELOCITY);
      
            logger(mfilename, "Servo speeds limited") 
        end
        
        function set_servo_acceleration(obj, ID, DXL_ACCELERATION)
            % Assuming Time-based Profile
            % Range: 0 ~ 32737 where '0' represents an infinite acceleration time('0 [msec]').
            % Profile Acceleration(108, Acceleration time) will not exceed 50% of Profile Velocity (112, the time span to reach the velocity of the Profile) value.
            logger(mfilename, "Setting servo acceleration limit") 
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, ID, obj.ADDR_PRO_ACCELERATION, DXL_ACCELERATION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d acceleration limit failed to be set \n', ID);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d acceleration limit failed to be set \n', ID);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            else
                fprintf('[Log]: Dynamixel #%d acceleration limit has been successfully limited \n', ID);
            end
            
            logger(mfilename, "Servo acceleration limit set")
        end
        
        function set_all_servo_acceleration_limits(obj, DXL_ACCELERATION)
            logger(mfilename, "Setting servo acceleration limit") 
            
            set_servo_acceleration(obj, obj.DXL_ID1_BaseRotation, DXL_ACCELERATION);
            set_servo_acceleration(obj, obj.DXL_ID2_Shoulder, DXL_ACCELERATION);
            set_servo_acceleration(obj, obj.DXL_ID3_Elbow, DXL_ACCELERATION);
            set_servo_acceleration(obj, obj.DXL_ID4_Wrist, DXL_ACCELERATION);
            set_servo_acceleration(obj, obj.DXL_ID5_Gripper, DXL_ACCELERATION);
           
            logger(mfilename, "Servo acceleration limit set") 
        end
        
        function set_all_servo_range_motion(obj)
           logger(mfilename, "Servo range motion limiting initiated") 
            
           % Limiting max range
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_MAX_POS, obj.MAX_POS(1)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_MAX_POS, obj.MAX_POS(2)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID2_Shoulder);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID2_Shoulder);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_MAX_POS, obj.MAX_POS(3)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID3_Elbow);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID3_Elbow);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_MAX_POS, obj.MAX_POS(4)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID4_Wrist);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID4_Wrist);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_MAX_POS, obj.MAX_POS(5)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID5_Gripper);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID5_Gripper);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end

           % Limiting min range
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_MIN_POS, obj.MIN_POS(1)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID1_BaseRotation);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_MIN_POS, obj.MIN_POS(2)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID2_Shoulder);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID2_Shoulder);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_MIN_POS, obj.MIN_POS(3)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID3_Elbow);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID3_Elbow);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_MIN_POS, obj.MIN_POS(4)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID4_Wrist);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID4_Wrist);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_MIN_POS, obj.MIN_POS(5)); 
           if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID5_Gripper);
               printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
               fprintf('[Error]: Dynamixel #%d motion failed to be limited \n', obj.DXL_ID5_Gripper);
               printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
           end
           
           logger(mfilename, "Servo range motion limiting completed") 
        end
    
        %% --- Write and read functions --- %%
        function write_raw_encoder(obj, SERVO_ID, ENCODER_VAL)
            msg = append('Writing encoder value: ', string(ENCODER_VAL), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg)

            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, SERVO_ID, obj.ADDR_PRO_GOAL_POSITION, ENCODER_VAL);

            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d raw encoder cannot be written \n', SERVO_ID);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d raw encoder cannot be written \n', SERVO_ID);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            msg = append('Completed writing encoder value: ', string(ENCODER_VAL), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg)
        end
        
        function write_angles(obj, SERVO_ID, ANGLE)
            msg = append('Writing angle: ', string(ANGLE), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg)
            
            % Convert degree to raw encoder value
            ENCODER_VAL = ANGLE / 0.088;
            
            % Write to servo
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, SERVO_ID, obj.ADDR_PRO_GOAL_POSITION, ENCODER_VAL);

            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d angle cannot be written \n', SERVO_ID);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d angle cannot be written \n', SERVO_ID);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            msg = append('[Log]: Completed writing encoder value: ', string(ENCODER_VAL), ' to servo: ', string(SERVO_ID));
            logger(mfilename, msg)
        end
        
        function write_angles_to_all_servos(obj, ID1_ANGLE, ID2_ANGLE, ID3_ANGLE, ID4_ANGLE)
            msg = append('Writing angles: ', string(ID1_ANGLE), ' ', string(ID2_ANGLE), ' ', string(ID3_ANGLE), ' ', string(ID4_ANGLE));
            logger(mfilename, msg) % [Log]
            
            % Convert to raw encoder values
            constant = 360/4096;
            ID1_VALUE = ID1_ANGLE / constant;
            ID2_VALUE = ID2_ANGLE / constant;
            ID3_VALUE = ID3_ANGLE / constant;
            ID4_VALUE = ID4_ANGLE / constant;

            % Add goal position values to the Syncwrite storage
            obj.DXL_ADDPARAM_RESULT = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID1_BaseRotation, typecast(int32(ID1_VALUE), 'uint32'), 4);
            if obj.DXL_ADDPARAM_RESULT ~= true
                fprintf('[Log]: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID1_BaseRotation);
                return;
            end
            
            obj.DXL_ADDPARAM_RESULT = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID2_Shoulder, typecast(int32(ID2_VALUE), 'uint32'), 4);
            if obj.DXL_ADDPARAM_RESULT ~= true
                fprintf('[Log]: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID2_Shoulder);
                return;
            end
            
            obj.DXL_ADDPARAM_RESULT = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID3_Elbow, typecast(int32(ID3_VALUE), 'uint32'), 4);
            if obj.DXL_ADDPARAM_RESULT ~= true
                fprintf('[Log]: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID3_Elbow);
                return;
            end
            
            obj.DXL_ADDPARAM_RESULT = groupSyncWriteAddParam(obj.GROUPWRITE_NUM, obj.DXL_ID4_Wrist, typecast(int32(ID4_VALUE), 'uint32'), 4);
            if obj.DXL_ADDPARAM_RESULT ~= true
                fprintf('[Log]: [ID:%03d] groupSyncWrite addparam failed', obj.DXL_ID4_Wrist);
                return;
            end
            
            logger(mfilename, "All angles converted and stored. Attempting to write simultaneously to all servos")
            
            % Write in sync to all servo
            groupSyncWriteTxPacket(obj.GROUPWRITE_NUM); 
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Failed to write simultaneously to all servos \n');
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            % Check if reached
            check_position_reached(obj, ID1_ANGLE, ID2_ANGLE, ID3_ANGLE, ID4_ANGLE);
            
            % Clear syncwrite parameter storage
            groupSyncWriteClearParam(obj.GROUPWRITE_NUM);
                
            logger(mfilename, "Successfully written to all servos")
        end
        
        function [ID11_Value, ID12_Value, ID13_Value, ID14_Value, ID15_Value] = read_all_servo_raw_encoder(obj)
            logger(mfilename, "Reading all servo raw encoder values")
            
            ID11_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d raw encoder cannot be read \n', obj.DXL_ID1_BaseRotation);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d raw encoder cannot be read \n', obj.DXL_ID1_BaseRotation);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            ID12_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d raw encoder cannot be read \n', obj.DXL_ID2_Shoulder);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d raw encoder cannot be read \n', obj.DXL_ID2_Shoulder);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            ID13_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d raw encoder cannot be read \n', obj.DXL_ID3_Elbow);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d raw encoder cannot be read \n', obj.DXL_ID3_Elbow);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            ID14_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d raw encoder cannot be read \n', obj.DXL_ID4_Wrist);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d raw encoder cannot be read \n', obj.DXL_ID4_Wrist);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            ID15_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d raw encoder cannot be read \n', obj.DXL_ID5_Gripper);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d raw encoder cannot be read \n', obj.DXL_ID5_Gripper);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            logger(mfilename, "Successfully read all servo raw encoder values")
        end
        
        function [ID11_Angle, ID12_Angle, ID13_Angle, ID14_Angle, ID15_Angle] = read_all_servo_angles(obj)
            logger(mfilename, "Reading all servo raw encoder values")
            
            ID11_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.DXL_ID1_BaseRotation));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.DXL_ID1_BaseRotation));
            end
            
            ID12_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.DXL_ID2_Shoulder));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.DXL_ID2_Shoulder));
            end
            
            ID13_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.DXL_ID3_Elbow));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.DXL_ID3_Elbow));
            end
            
            ID14_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.DXL_ID4_Wrist));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.DXL_ID4_Wrist));
            end
            
            ID15_Value = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_PRESENT_POSITION);
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.DXL_ID5_Gripper));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.DXL_ID5_Gripper));
            end
            
            ID11_Angle = (0.088 * typecast(single(ID11_Value), 'single'));
            ID12_Angle = (0.088 * typecast(single(ID12_Value), 'single'));
            ID13_Angle = (0.088 * typecast(single(ID13_Value), 'single'));
            ID14_Angle = (0.088 * typecast(single(ID14_Value), 'single'));
            ID15_Angle = (0.088 * typecast(single(ID15_Value), 'single'));
        end

         %% --- Gripper functions --- %%
        function open_gripper(obj)
            logger(mfilename, "Opening gripper")
            % 1943.18 -> R02
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_GOAL_POSITION, obj.GRIPPER_OPEN);
            pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d: Gripper failed to open \n', obj.DXL_ID5_Gripper);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d: Gripper failed to open \n', obj.DXL_ID5_Gripper);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            % Loop until position is reached 
            while 1
                dxl_present_position = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_PRESENT_POSITION);
                if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                    printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                    printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                end
                
                if ~((abs(obj.GRIPPER_OPEN - typecast(uint32(dxl_present_position), 'int32')) > obj.DXL_MOVING_STATUS_THRESHOLD))
                    logger(mfilename, "Goal position reached")
                    fprintf('[Log]: Pos: #%d - #%d - #%d - #%d \n', obj.GRIPPER_OPEN);
                    fprintf('[Log]: Pos: #%d - #%d - #%d - #%d \n', dxl_present_position);
                    break;
                end
            end
            
            logger(mfilename, "Gripper opened") 
        end
        
        function close_gripper(obj)
            logger(mfilename, "Closing gripper")
            % 2382.0 -> R02
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_GOAL_POSITION, obj.GRIPPER_CLOSE);
            pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d: Gripper failed to close \n', obj.DXL_ID5_Gripper);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d: Gripper failed to close \n', obj.DXL_ID5_Gripper);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            % Loop until position is reached 
            while 1
                dxl_present_position = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_PRESENT_POSITION);
                if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                    printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                    printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                end
                
                if ~((abs(obj.GRIPPER_CLOSE - typecast(uint32(dxl_present_position), 'int32')) > obj.DXL_MOVING_STATUS_THRESHOLD))
                    logger(mfilename, "Goal position reached")
                    fprintf('[Log]: Pos: #%d - #%d - #%d - #%d \n', obj.GRIPPER_CLOSE);
                    fprintf('[Log]: Pos: #%d - #%d - #%d - #%d \n', dxl_present_position);
                    break;
                end
            end
            
            logger(mfilename, "Gripper closed") 
        end
        
        function close_gripper_pen(obj)
            logger(mfilename, "Closing gripper")
            % 2382.0 -> R02
            
            write4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_GOAL_POSITION, obj.GRIPPER_CLOSE + 150);
            pause(0.5)
            
            if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                fprintf('[Error]: Dynamixel #%d: Gripper failed to close \n', obj.DXL_ID5_Gripper);
                printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                fprintf('[Error]: Dynamixel #%d: Gripper failed to close \n', obj.DXL_ID5_Gripper);
                printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
            end
            
            % Loop until position is reached 
            while 1
                dxl_present_position = read4ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_PRESENT_POSITION);
                if getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= obj.COMM_SUCCESS
                    printTxRxResult(obj.PROTOCOL_VERSION, getLastTxRxResult(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                elseif getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION) ~= 0
                    printRxPacketError(obj.PROTOCOL_VERSION, getLastRxPacketError(obj.PORT_NUM, obj.PROTOCOL_VERSION));
                end
                
                if ~((abs(obj.GRIPPER_CLOSE - typecast(uint32(dxl_present_position), 'int32')) > obj.DXL_MOVING_STATUS_THRESHOLD))
                    logger(mfilename, "Goal position reached")
                    fprintf('[Log]: Pos: #%d - #%d - #%d - #%d \n', obj.GRIPPER_CLOSE);
                    fprintf('[Log]: Pos: #%d - #%d - #%d - #%d \n', dxl_present_position);
                    break;
                end
            end
            
            logger(mfilename, "Gripper closed") 
        end
        
        %% --- Task functions --- %%
        function pick_up_cube_at_coord(obj, trajectoryLib, P_X, P_Y, P_Z, PHI)            
            % Calculate IK
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = trajectoryLib.IK_with_PHI(P_X, P_Y, (P_Z + obj.ABOVE_CUBE_OFFSET), 0);

            % Move to coordinate directly above cube by 0.05 m
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, 180, 180, 180);
            open_gripper(obj);
            
            Z_COORDS = linspace((P_Z + obj.ABOVE_CUBE_OFFSET), P_Z, 10);
            PHI_TRANSITION = 0;
            
            %for i = 1:10
            %    [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, Z_COORDS(i), PHI_TRANSITION);
            %    write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
                
            %    PHI_TRANSITION = PHI_TRANSITION -9;
            %end
            
            [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z, -90);
            write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
            % Move arm down and grip the cube 
            pause(0.5);
            close_gripper(obj); 

            % Move arm back up
            Z_COORDS = linspace(P_Z, (P_Z + obj.ABOVE_CUBE_OFFSET), 10);
            PHI_TRANSITION = -90;
            
            %for i = 1:10
            %    [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, Z_COORDS(i), PHI_TRANSITION);
            %    write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
                
            %    PHI_TRANSITION = PHI_TRANSITION + 9;
            %end

            
            

            dist = sqrt(P_X^2 + P_Y^2);
            disp(dist);
            if (dist > 0.1375)
                [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z + obj.ABOVE_CUBE_OFFSET, 0);
                write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
            else
                [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z + obj.ABOVE_CUBE_OFFSET, -90);
                write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
            end
            write_angles_to_all_servos(obj, SERVO_THETA_1, 180, 180, 180);
            %write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
        end
        
        function pick_up_cube_at(obj, trajectoryLib, trajectory_obj, ROW, COLUMN, PHI)
            % Get board location specified 
            [P_X, P_Y, P_Z] = get_board_location(trajectory_obj, ROW, COLUMN);
            
            P_Z_WITH_CENTER_CUBE = P_Z + 0.0125;
            
            % Modify P_Z height to grip the cube at 3/4 the gripper's
            % length to avoid gripper the platform as well
            % Shift the designated height upward by 0.00865
            if PHI == 0
                P_Z_CALIBRATED = P_Z_WITH_CENTER_CUBE + 0.00375;
            elseif PHI == -90
                P_Z_CALIBRATED = P_Z_WITH_CENTER_CUBE + 0.00865;
            end 
            
            % Calculate IK
            [SERVO_THETA_1_GRIP, SERVO_THETA_2_GRIP, SERVO_THETA_3_GRIP, SERVO_THETA_4_GRIP] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z_CALIBRATED, PHI);
            
            % Process
            open_gripper(obj);
            
            % Move arm down 
            write_angles_to_all_servos(obj, SERVO_THETA_1_GRIP, SERVO_THETA_2_GRIP, SERVO_THETA_3_GRIP, SERVO_THETA_4_GRIP);
            
            % Grip the cube 
            close_gripper(obj);
        end
        
        function drop_cube_at_coord(obj, trajectoryLib, P_X, P_Y, P_Z, PHI) 
            % Calculate IK
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = trajectoryLib.IK_with_PHI(P_X, P_Y, (P_Z + obj.ABOVE_CUBE_OFFSET), 0);
            
            % Move to coordinate directly above cube by 0.05 m
            %write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            
            %Z_COORDS = linspace((P_Z + obj.ABOVE_CUBE_OFFSET), P_Z, 10);
            %PHI_TRANSITION = -9;
            
            %for i = 1:10
            %    [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, Z_COORDS(i), PHI_TRANSITION);
            %    write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
                
            %    PHI_TRANSITION = PHI_TRANSITION -9;
            %end
            
            
            [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z-0.001, -90);
            write_angles_to_all_servos(obj, SERVO_THETA_1, 180, 180, 180);
            write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
            
            % Drop the cube 
            pause(1);
            open_gripper(obj);

            % Move arm back up
            Z_COORDS = linspace(P_Z, (P_Z + obj.ABOVE_CUBE_OFFSET), 10);
            PHI_TRANSITION = -90;
            
            %for i = 1:10
            %    [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, Z_COORDS(i), PHI_TRANSITION);
            %    write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
                
            %    PHI_TRANSITION = PHI_TRANSITION + 9;
            %end
            
            [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z + obj.ABOVE_CUBE_OFFSET, 0);
            write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
        end

        function drop_cube_at(obj, trajectoryLib, trajectory_obj, ROW, COLUMN, PHI)
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
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = trajectoryLib.IK_with_PHI(P_X, P_Y, (P_Z_CALIBRATED + 0.05), PHI);
            [SERVO_THETA_1_DROP, SERVO_THETA_2_DROP, SERVO_THETA_3_DROP, SERVO_THETA_4_DROP] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z_CALIBRATED, PHI);
            
            % Move to coordinate directly above cube by 0.025 m
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            
            % Move arm down 
            write_angles_to_all_servos(obj, SERVO_THETA_1_DROP, SERVO_THETA_2_DROP, SERVO_THETA_3_DROP, SERVO_THETA_4_DROP);
            
            % Drop the cube 
            open_gripper(obj);
            
            % Move arm back up
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
        end
        
        function move_cube_coord(obj, trajectoryLib, P_X1, P_Y1, P_Z1, P_X2, P_Y2, P_Z2, forward_offset, height_offset)
            PHI = -90;
            PICKING_HEIGHT = 0.03;
            
            degree = atand(P_Y2/P_X2);
            
            y_offset = (forward_offset) * sind(degree);
            x_offset = (forward_offset) * cosd(degree);
            
            % Linear interpolation:
            % - Arm will move around at 0.06 height and PHI = 0
            % Step 1: Get current location and move to 0.1 height
            [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = read_all_servo_angles(obj);
            [P_X, P_Y, P_Z] = trajectoryLib.FK(SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
            
            P_Z_MOVING_HEIGHT = 0.15;
            %[SERVO_THETA_1_MOVE, SERVO_THETA_2_MOVE, SERVO_THETA_3_MOVE, SERVO_THETA_4_MOVE] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z_MOVING_HEIGHT, 0);
            %write_angles_to_all_servos(obj, SERVO_THETA_1_MOVE, SERVO_THETA_2_MOVE, SERVO_THETA_3_MOVE, SERVO_THETA_4_MOVE);
            
            % Step 2: Linear interpolate to coordinate directly 0.1 m above
            X_COORDS = linspace(P_X, P_X1, 5);
            Y_COORDS = linspace(P_Y, P_Y1, 5);
            
            %for i = 1:5
            %    [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(X_COORDS(1), Y_COORDS(2), P_Z_MOVING_HEIGHT, 0);
            %    write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
            %end
            
            % Step 3: Pick up cube            
            pick_up_cube_at_coord(obj, trajectoryLib, P_X1, P_Y1, P_Z1, PHI);
            
            % Step 4: Go back up to 0.1m traversing height
            [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = read_all_servo_angles(obj);
            [P_X, P_Y, ~] = trajectoryLib.FK(SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
            
            [SERVO_THETA_1_MOVE, SERVO_THETA_2_MOVE, SERVO_THETA_3_MOVE, SERVO_THETA_4_MOVE] = trajectoryLib.IK_with_PHI(P_X2 + x_offset, P_Y2 + y_offset, P_Z_MOVING_HEIGHT + height_offset, 0);
            %write_angles_to_all_servos(obj, SERVO_THETA_1_MOVE, SERVO_THETA_2_MOVE, SERVO_THETA_3_MOVE, SERVO_THETA_4_MOVE);
            
            % Step 5: Linear interpolate to location 
            X_COORDS = linspace(P_X1, P_X2, 5);
            Y_COORDS = linspace(P_Y1, P_Y2, 5);
           
            % Step 6: Drop the cube        
            drop_cube_at_coord(obj, trajectoryLib, P_X2 + x_offset, P_Y2 + y_offset, P_Z2 + height_offset, PHI);
        end
        
        function rotate_cube_forward_at_coord(obj, trajectoryLib, P_X, P_Y, P_Z)
            % Calculate IK
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = trajectoryLib.IK_with_PHI(P_X, P_Y, (P_Z + obj.ABOVE_CUBE_OFFSET), 0);

            % Move to coordinate directly above cube by 0.05 m
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            open_gripper(obj);
            
            Z_COORDS = linspace((P_Z + obj.ABOVE_CUBE_OFFSET), P_Z, 10);
            
            for i = 1:10
                [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI_tip(P_X, P_Y, Z_COORDS(i), 0);
                write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
            end
            
            % Move arm down and grip the cube 
            close_gripper(obj); 
            
            % Move arm back up
            Z_COORDS = linspace(P_Z, (P_Z + 0.1), 0);
            
            for i = 1:10
                [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, Z_COORDS(i), 0);
                write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
            end
            
            PHI_TRANSITION = 0;
            
            for i = 1:10
                [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, Z_COORDS(i), PHI_TRANSITION);
                write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
                
                PHI_TRANSITION = PHI_TRANSITION -9;
            end
            open_gripper(obj);
        end
        
        function pick_up_pen(obj, trajectoryLib, P_X, P_Y, P_Z)
            open_gripper(obj);
            
            % Calculate IK
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = trajectoryLib.IK_with_PHI_draw(P_X, P_Y, P_Z, 0);

            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            
            close_gripper_pen(obj);
            pause(1);
            
            % Move arm back up
            Z_COORDS = linspace(P_Z, (P_Z + 0.05), 10);
            
            for i = 1:10
                [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, Z_COORDS(i),  0);
                write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
                
            end
            
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, 180, 180, SERVO_THETA_4_ABOVE_CUBE);
        end
        
        function drop_pen(obj, trajectoryLib, P_X, P_Y, P_Z)
            
            % Calculate IK
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = trajectoryLib.IK_with_PHI_draw(P_X, P_Y, P_Z+0.055, 0);

            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = trajectoryLib.IK_with_PHI_draw(P_X, P_Y, P_Z, 0);

            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            open_gripper(obj);
            pause(1);
            
            % Move arm back up
            Z_COORDS = linspace(P_Z, (P_Z + 0.05), 10);
            
            for i = 1:10
                [SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4] = trajectoryLib.IK_with_PHI(P_X, P_Y, Z_COORDS(i),  0);
                write_angles_to_all_servos(obj, SERVO_THETA_1, SERVO_THETA_2, SERVO_THETA_3, SERVO_THETA_4);
                
            end
            
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, 180, 180, SERVO_THETA_4_ABOVE_CUBE);
        end
                  
        function rotate_cube_backward_at_coord(obj, trajectoryLib, P_X, P_Y, P_Z, forward_offset, z_offset)
            
            constant = 0.00000115;
            constant2 = 0.00000205;
            constant3 = -0.005;
            offset = 1/(P_X^2 + P_Y^2 + P_Z^2)^3 * constant;
            offset2 = 1/(P_X^2 + P_Y^2 + P_Z^2)^2 * constant2;
            dist = (P_X^2 + P_Y^2 + P_Z^2);
            
            phi = -90;
            degree = atand(P_Y/P_X);

            general_z_offset = 0.0035;
            y_offset = (offset2 + offset + constant3) * sind(degree);
            x_offset = (offset2 + offset + constant3) * cosd(degree);
            
            y_offset_2 = (forward_offset) * sind(degree);
            x_offset_2 = (forward_offset) * cosd(degree);
            
            disp(x_offset);
            disp(y_offset);
            [SERVO_THETA_1_PRE_ROTATION, SERVO_THETA_2_PRE_ROTATION, SERVO_THETA_3_PRE_ROTATION, SERVO_THETA_4_PRE_ROTATION] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z + general_z_offset, phi);
            phi = 0;
            [SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION] = trajectoryLib.IK_with_PHI(P_X-x_offset + x_offset_2, P_Y - y_offset + y_offset_2, P_Z - offset + z_offset, phi);
            
            % Move to coordinate directly above cube by 0.025 m
            write_angles_to_all_servos(obj, SERVO_THETA_1_PRE_ROTATION, 180, 180, 180);
            
            open_gripper(obj);
            pause(1);
            % - Pick up cube at position with phi = 0 
            write_angles_to_all_servos(obj, SERVO_THETA_1_PRE_ROTATION, SERVO_THETA_2_PRE_ROTATION, SERVO_THETA_3_PRE_ROTATION, SERVO_THETA_4_PRE_ROTATION);
            pause(1);
            close_gripper(obj);
            % - Move upward             pause(3);
            % - Move cube to the exact same position with phi = -90
            write_angles_to_all_servos(obj, SERVO_THETA_1_POST_ROTATION, 180, 180, SERVO_THETA_4_POST_ROTATION);
            % Drop cube at intended position
            write_angles_to_all_servos(obj, SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION);
            pause(1);
            open_gripper(obj);
            pause(1);
            write_angles_to_all_servos(obj, SERVO_THETA_1_PRE_ROTATION, 180, 180, 180);
        end
        
        function rotate_cube_forward_at(obj, trajectoryLib, ROW, COLUMN)
            [P_X, P_Y, P_Z] = get_board_location(trajectory_obj, ROW, COLUMN);
            
            P_Z_WITH_CUBE = P_Z + 0.0125;
            
            % Modify P_Z height to grip the cube at 3/4 the gripper's
            % length to avoid gripper the platform as well
            % Shift the designated height upward by 0.00965
            % P_Z_CALIBRATED = P_Z_WITH_CUBE + 0.00965; 
            P_Z = P_Z_WITH_CUBE;
            
            
            phi = 0;
            [SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z + 0.025, phi);
            [SERVO_THETA_1_PRE_ROTATION, SERVO_THETA_2_PRE_ROTATION, SERVO_THETA_3_PRE_ROTATION, SERVO_THETA_4_PRE_ROTATION] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z, phi);
            phi = -90;
            [SERVO_THETA_1_ABOVE_POST_ROTATION, SERVO_THETA_2_ABOVE_POST_ROTATION, SERVO_THETA_3_ABOVE_POST_ROTATION, SERVO_THETA_4_ABOVE_POST_ROTATION] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z + 0.025, phi);
            [SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z, phi);
            
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
        
        function rotate_cube_backward_at(obj, trajectoryLib, ROW, COLUMN)
            [P_X, P_Y, P_Z] = get_board_location(trajectory_obj, ROW, COLUMN);
            
            P_Z_WITH_CUBE = P_Z + 0.0125;
            
            % Modify P_Z height to grip the cube at 3/4 the gripper's
            % length to avoid gripper the platform as well
            % Shift the designated height upward by 0.00965
            % P_Z_CALIBRATED = P_Z_WITH_CUBE + 0.00965; 
            P_Z = P_Z_WITH_CUBE;
            
            
            phi = -90;
            [SERVO_THETA_1_ABOVE_PRE_ROTATION, SERVO_THETA_2_ABOVE_PRE_ROTATION, SERVO_THETA_3_ABOVE_PRE_ROTATION, SERVO_THETA_4_ABOVE_PRE_ROTATION] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z + 0.025, phi);
            [SERVO_THETA_1_PRE_ROTATION, SERVO_THETA_2_PRE_ROTATION, SERVO_THETA_3_PRE_ROTATION, SERVO_THETA_4_PRE_ROTATION] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z, phi);
            phi = 0;
            [SERVO_THETA_1_ABOVE_POST_ROTATION, SERVO_THETA_2_ABOVE_POST_ROTATION, SERVO_THETA_3_ABOVE_POST_ROTATION, SERVO_THETA_4_ABOVE_POST_ROTATION] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z + 0.025, phi);
            [SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z, phi);
           
            
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