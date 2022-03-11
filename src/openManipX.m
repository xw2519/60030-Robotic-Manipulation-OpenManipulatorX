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
        DEVICENAME                  = 'COM7';       % Check which port is being used on your controller
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
        LEN_PRO_PRESENT_POSITION = 4
        LIB_NAME
        PORT_NUM
        
        % Gripper calibrate values 
        GRIPPER_CLOSE = 3467.05
        GRIPPER_OPEN = 4162.86
        
        % Pick up and drop cubes calibration values 
        ABOVE_CUBE_OFFSET = 0.05
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
            %obj.write_angles
            %obj.write_angles
            %obj.write_angles
            %obj.write_angles
            
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
        
        %% --- Servo modes --- %%
        function position_control_mode(obj)    
            logger(mfilename, "Setting Position Control mode") 
            
            % Put actuators into Position Control Mode
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID1_BaseRotation, obj.ADDR_PRO_OPERATING_MODE, 3);            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID2_Shoulder, obj.ADDR_PRO_OPERATING_MODE, 3);            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID3_Elbow, obj.ADDR_PRO_OPERATING_MODE, 3);            
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID4_Wrist, obj.ADDR_PRO_OPERATING_MODE, 3);           
            write1ByteTxRx(obj.PORT_NUM, obj.PROTOCOL_VERSION, obj.DXL_ID5_Gripper, obj.ADDR_PRO_OPERATING_MODE, 4);
            
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
            ID1_VALUE = ID1_ANGLE / 0.088;
            ID2_VALUE = ID2_ANGLE / 0.088;
            ID3_VALUE = ID3_ANGLE / 0.088;
            ID4_VALUE = ID4_ANGLE / 0.088;

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
            
            logger(mfilename, "Gripper closed") 
        end
        
        %% --- Task functions --- %%
        function pick_up_cube_at_coord(obj, trajectoryLib, P_X, P_Y, P_Z, PHI)            
            % Calculate IK
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = trajectoryLib.IK_with_PHI(P_X, P_Y, (P_Z + obj.ABOVE_CUBE_OFFSET), PHI);
            [SERVO_THETA_1_GRIP, SERVO_THETA_2_GRIP, SERVO_THETA_3_GRIP, SERVO_THETA_4_GRIP] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z, PHI);
            
            % Move to coordinate directly above cube by 0.05 m
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            open_gripper(obj);
            
            pause(1);
            
            % Move arm down and grip the cube 
            write_angles_to_all_servos(obj, SERVO_THETA_1_GRIP, SERVO_THETA_2_GRIP, SERVO_THETA_3_GRIP, SERVO_THETA_4_GRIP);
            close_gripper(obj); 
            
            pause(2);
            
            % Move arm back up
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
        end
        
        function pick_up_cube_at(obj, trajectoryLib, trajectory_obj, ROW, COLUMN, PHI)
            % Get board location specified 
            [P_X, P_Y, P_Z] = get_board_location(trajectory_obj, ROW, COLUMN);
            
            P_Z_WITH_CUBE = P_Z + 0.0125;
            
            % Modify P_Z height to grip the cube at 3/4 the gripper's
            % length to avoid gripper the platform as well
            % Shift the designated height upward by 0.00965
            % P_Z_CALIBRATED = P_Z_WITH_CUBE + 0.00965; 
            P_Z_CALIBRATED = P_Z_WITH_CUBE;
            
            % Calculate IK
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = trajectoryLib.IK_with_PHI(P_X, P_Y, (P_Z_CALIBRATED + 0.05), PHI);
            [SERVO_THETA_1_GRIP, SERVO_THETA_2_GRIP, SERVO_THETA_3_GRIP, SERVO_THETA_4_GRIP] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z_CALIBRATED, PHI);
            
            % Move to coordinate directly above cube by 0.025 m
            write_angles_to_all_servos(arm_obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            pause(1);
            open_gripper(obj);
            
            pause(3);
            
            % Move arm down 
            write_angles_to_all_servos(obj, SERVO_THETA_1_GRIP, SERVO_THETA_2_GRIP, SERVO_THETA_3_GRIP, SERVO_THETA_4_GRIP);
            
            % Grip the cube 
            close_gripper(obj);
            
            % Move arm back up
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            pause(3);
            write_angles_to_all_servos(arm_obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            pause(2);
        end
        
        function drop_cube_at_coord(obj, trajectoryLib, P_X, P_Y, P_Z, PHI) 
            % Calculate IK
            [SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE] = trajectoryLib.IK_with_PHI(P_X, P_Y, (P_Z + obj.ABOVE_CUBE_OFFSET), PHI);
            [SERVO_THETA_1_DROP, SERVO_THETA_2_DROP, SERVO_THETA_3_DROP, SERVO_THETA_4_DROP] = trajectoryLib.IK_with_PHI(P_X, P_Y, P_Z, PHI);
            
            % Move to coordinate directly above cube by 0.05 m
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
            pause(2);
            
            % Move arm down 
            write_angles_to_all_servos(obj, SERVO_THETA_1_DROP, SERVO_THETA_2_DROP, SERVO_THETA_3_DROP, SERVO_THETA_4_DROP);
            pause(1);
            
            % Drop the cube 
            open_gripper(obj);
            
            pause(2);
            
            % Move arm back up
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_CUBE, SERVO_THETA_2_ABOVE_CUBE, SERVO_THETA_3_ABOVE_CUBE, SERVO_THETA_4_ABOVE_CUBE);
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
        
        function move_cube_coord(obj, trajectoryLib, P_X1, P_Y1, P_Z1, P_X2, P_Y2, P_Z2)
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
            
            pick_up_cube_at_coord(obj, trajectoryLib, P_X1, P_Y1, P_Z1, phi);
            drop_cube_at_coord(obj, trajectoryLib, P_X2, P_Y2, P_Z2, phi);
        end
        
        function rotate_cube_forward_at_coord(obj, trajectoryLib, P_X, P_Y, P_Z)
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
        
        function rotate_cube_backward_at_coord(obj, trajectoryLib, P_X, P_Y, P_Z)
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
            pause(3);
            % - Move cube to the exact same position with phi = -90
            write_angles_to_all_servos(obj, SERVO_THETA_1_ABOVE_POST_ROTATION, SERVO_THETA_2_ABOVE_POST_ROTATION, SERVO_THETA_3_ABOVE_POST_ROTATION, SERVO_THETA_4_ABOVE_POST_ROTATION);
            pause(3);
            % Drop cube at intended position
            write_angles_to_all_servos(obj, SERVO_THETA_1_POST_ROTATION, SERVO_THETA_2_POST_ROTATION, SERVO_THETA_3_POST_ROTATION, SERVO_THETA_4_POST_ROTATION);
            pause(3);
            open_gripper(obj);
            pause(3);
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