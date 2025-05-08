%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for robot pose and laser scan data
global pose scan imu

%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');

%% Display available ROS2 topics (for debug)
ros2 topic list

%% Create a ROS2 node for communication
controlNode = ros2node('/base_station');

%% Define subscribers
odomSub = ros2subscriber(controlNode, '/odom', @odomCallback); % odometry topic
scanSub = ros2subscriber(controlNode, '/scan', @scanCallback, 'Reliability', 'besteffort'); % laser scan topic
imuSub = ros2subscriber(controlNode, '/imu', @imuCallback);
% Pause to allow ROS subscriptions to initialize
pause(0.5);
    
try
    %% Define publishers
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');
    
    %% Create figure for TurtleBot's data
    visualise = TurtleBotVisualise();
    %vis_accel = imuVisualise();
    %% Initialize array for desired positions
    position_desired = [1, 1];
    
    %% Initial position
    init_pos = [pose.position.x pose.position.y];
    quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
    orientation = quat2eul(quat);  % Convert quaternion to Euler angles
    init_heading = orientation(3); % Extract heading (yaw)
    
    %% PID variables
    Kp_pos = 0.2; Ki_pos = 0.00001; Kd_pos = 0.04;

    Kp_ang = 0.3; Ki_ang = 0.0001; Kd_ang = 0.2;
    %% PID error variables
    prevErrorPos = 0;
    integralErrorPos = 0;
    prevErrorAng = 0;
    integralErrorAng = 0;

    %% Collision detection threshold
    T = 5;

    time = tic;
    visualUpdateRate = 0.05;
    lastVisualUpdate = toc(time);

    %% Infinite loop for real-time visualization, until the figure is closed
    while true
        %% Visialise desired position
        if(toc(time) - lastVisualUpdate > visualUpdateRate)
            visualise = updatePositionDesired(visualise, position_desired);
            
        end
        
        %% Get the robot's current position and heading
        position = [pose.position.x pose.position.y] - init_pos;
        quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
        orientation = quat2eul(quat);  % Convert quaternion to Euler angles
        heading = orientation(3); % Extract heading (yaw)
        if(toc(time) - lastVisualUpdate > visualUpdateRate)
            visualise = updatePose(visualise, position, heading);
        end
        %% Process and plot laser scan data
        cart = rosReadCartesian(scan);  % Convert scan to Cartesian coordinates
        cart = cart * [cos(heading), -sin(heading); sin(heading), cos(heading)]' + position; % Transform based on robot position and heading
        
        %% Find pos Error
        e_x = position_desired(1) - position(1);
        e_y = position_desired(2) - position(2);
        errorPos = norm(position_desired - position);
        %errorPos = (position_desired - position);
        deltaErrorPos = (errorPos - prevErrorPos);
        integralErrorPos = integralErrorPos + errorPos;

        %% Find heading error
        desiredHeading = atan2(e_y,e_x);
        errorAng = desiredHeading - heading;
        deltaErrorAng = (errorAng - prevErrorAng);
        integralErrorAng = integralErrorAng + errorAng;
        
        if(toc(time) - lastVisualUpdate > visualUpdateRate)
            visualise = updateScan(visualise, cart);
            lastVisualUpdate = toc(time);
        end

        %% PID controller for heading
        angularVelocity = Kp_ang * errorAng + Ki_ang * integralErrorAng + Kd_ang * deltaErrorAng;

        %% PID controller for position
        linearVelocity = Kp_pos * errorPos + Ki_pos * integralErrorPos + Kd_pos * deltaErrorPos;
        linearVelocity = linearVelocity * (1 - min(abs(angularVelocity) / 2.0, 1)); 
        prevErrorAng = errorAng;
        prevErrorPos = errorPos;

        %% Detect collision
        %vis_accel = updateAccel(vis_accel, imu);
        collision = imu > T || imu < -T;
        %collision = false;
        if collision
            linearVelocity = 0;
            angularVelocity = 0;
            ME = MException('NonExeption:CollisionDetected', 'Collision detected.');
            throw(ME)
            
        end
        
        F_total = 0;
        attractive = 0;

        scan_front = [scan.ranges(346:360); scan.ranges(1:15)];
        scan_back = scan.ranges(165:195);
        
        d_0 = 1;   % Influence distance
        beta = 0.1;  % Strength of repulsion
        
        % Initialize repulsive force vectors
        fr_front = zeros(length(scan_front), 2);
        fr_back = zeros(length(scan_back), 2);
        
        for i = 1:length(scan_front)
            if scan_front(i) <= d_0 && scan_front(i) > 0  % Valid range
                angle = deg2rad(351 + i - 1);  % Convert to radians (assumes scan angle is known)
                magnitude = -(2 * beta / scan_front(i)^2) * (1/scan_front(i) - 1/d_0);
                fr_front(i,1) =  magnitude * cos(angle);  % X component
                fr_front(i,2) =  magnitude * sin(angle);  % Y component
            end
        end
        
        for i = 1:length(scan_back)
            if scan_back(i) <= d_0 && scan_back(i) > 0
                angle = deg2rad(171 + i - 1);
                magnitude = (2 * beta / scan_back(i)^2) * (1/scan_back(i) - 1/d_0);
               
                fr_back(i,1) = magnitude * cos(angle);
                fr_back(i,2) = magnitude * sin(angle);
                
            end
        end
        attractive = abs(position_desired - position);  % Vector to target
        attractive = attractive / norm(attractive);  % Normalize

        % Compute total repulsive force
        F_total = sum(fr_front,1) + sum(fr_back,1) + attractive;

        
        repulsiveX = F_total(1);
        repulsiveY = F_total(2);
        


        

        %% Obstacle Avoidance
        %linearVelocity = 0;
        %angularVelocity = 0;
        if abs(repulsiveX) > 1 || abs(repulsiveY) > 1
            linearVelocity = linearVelocity + repulsiveX;
            angularVelocity = angularVelocity + repulsiveY * 2;
        end
      
        %% Publish velocity commands
        cmdMsg = ros2message('geometry_msgs/Twist');
        cmdMsg.linear.x = clip(linearVelocity, -0.2, 0.2);
        cmdMsg.angular.z = clip(angularVelocity, -2.0, 2.0);
        send(cmdPub, cmdMsg);
    
        %% Pause to visualize and delete old plots
        pause(0.01)
    
        %% Exit the loop if the figure is closed
        if size(findobj(visualise.fig)) == 0
            ME = MException('NonExeption:EndProgram', 'The program was closed.');
            throw(ME)
        end
    end
catch ME
    % Stop the robot
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.Linear.X = 0;
    cmdMsg.Angular.Z = 0;
    send(cmdPub, cmdMsg);
    
    % Clean up ROS subscriptions
    clear odomSub scanSub

    % Show the error
    if ~strcmp(ME.identifier, 'NonExeption:EndProgram')
        rethrow(ME)
    end
end 

%% Callback functions
function odomCallback(message)
    % Use global variable to store the robot's position and orientation
    global pose

    % Extract position and orientation data from the ROS message
    pose = message.pose.pose;
end

function scanCallback(message)
    % Use global variable to store laser scan data
    global scan

    % Save the laser scan message
    scan = message;
end

function imuCallback(message)
    global imu

    imu = message.linear_acceleration.x;
end