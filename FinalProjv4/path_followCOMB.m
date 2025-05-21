%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for robot pose and laser scan data
global pose scan image

%% Set the ROS domain ID for communication
setenv('ROS_DOMAIN_ID', '30');

%% Display available ROS2 topics (for debug)
ros2 topic list

%% Create a ROS2 node for communication
controlNode = ros2node('/base_station');

%% Define subscribers
odomSub = ros2subscriber(controlNode, '/odom', @odomCallback); % odometry topic
scanSub = ros2subscriber(controlNode, '/scan', @scanCallback, 'Reliability', 'besteffort'); % laser scan topic
imageSub = ros2subscriber(controlNode, '/camera/image_raw/compressed', @imageCallback); % image topic
% Pause to allow ROS subscriptions to initialize
pause(1);
    
try
    %% Define publishers
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');
    
    %% Create figure for TurtleBot's data
    visualise = TurtleBotVisualise();
    
    %% Create figures for position and heading
    %plotter = TurtleBotPlot();
    %% Find path
    heading_res = 0;

    prog = 0;
    map = projmap(prog);
    scan_right = scan.ranges(265:275);
    scan_back = scan.ranges(175:185);
    avg_right = double(mean(scan_right));
    avg_back = double(mean(scan_back));
    y_wall = 24.6;
    x_wall = 0.7;
    start = [6.187 25.6];
    goal = [14.2  25.65];
    path = new_createPRMpath(start, goal, map);
    x_waypoints = path(:,1);
    y_waypoints = path(:,2);
    t_waypoints = 1:length(path);
    init_pos = [pose.position.x pose.position.y];
    Offset = start - init_pos;  % Compute the true offset only once
    % Define the finer resolution for interpolation
    t_fine = linspace(1, length(path), 50); % 25 intermediate points
        
    % Interpolate x and y positions
    x_interp = interp1(t_waypoints, x_waypoints, t_fine, 'pchip');
    y_interp = interp1(t_waypoints, y_waypoints, t_fine, 'pchip');

    %% Initial position
    init_pos = [pose.position.x pose.position.y];
    quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
    orientation = quat2eul(quat);  % Convert quaternion to Euler angles
    init_heading = wrapToPi(orientation(3)); % Extract heading (yaw)
    
    %% PID variables
    Kp_pos = 0.3; Ki_pos = 0.00001; Kd_pos = 0.04;

    Kp_ang = 0.3; Ki_ang = 0.0001; Kd_ang = 0.2;
    %% PID error variables
    prevErrorPos = 0;
    integralErrorPos = 0;
    prevErrorAng = 0;
    integralErrorAng = 0;
    
    time = tic;
    visualUpdateRate = 0.2;
    lastVisualUpdate = toc(time);
    i = 0;
   
    %% Infinite loop for real-time visualization, until the figure is closed
    while prog < 2 %i <= length(x_interp)
        if (prevErrorPos < 0.5 && i<=length(x_interp) -1)
            i = i + 1;
        end
        position_desired = [x_interp(i) y_interp(i)]
        %% Visialise desired position
        if(toc(time) - lastVisualUpdate > visualUpdateRate)
            visualise = updatePositionDesired(visualise, position_desired);
        end
        %% Get the robot's current position and heading
        %position = [pose.position.x pose.position.y] + Offset
        quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
        orientation = quat2eul(quat);  % Convert quaternion to Euler angles
        heading =  wrapToPi(orientation(3) - init_heading);% - init_heading; % Extract heading (yaw)
        raw_pos = [pose.position.x pose.position.y];
        relative_pos = raw_pos - init_pos;
        rotation_matrix = [cos(-init_heading), -sin(-init_heading); sin(-init_heading), cos(-init_heading)];
        position = (rotation_matrix * relative_pos')' + start;

        rad2deg(heading)
        %heading = orientation(3);
        if(toc(time) - lastVisualUpdate > visualUpdateRate)
            visualise = updatePose(visualise, position, heading);
        end
        %% Reset heading in hallway
        if heading_res == 4 && position(1) > 9.5
            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = 0;
            send(cmdPub, cmdMsg);
            
            heading_res = 1;
            pause(0.1);

            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = 0.5;
            send(cmdPub, cmdMsg);

            
            while abs(heading) - 0 > 0.04
                pause(0.1);
                quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
                orientation = quat2eul(quat);  % Convert quaternion to Euler angles
                heading =  wrapToPi(orientation(3) - init_heading);
            end
            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = 0;
            send(cmdPub, cmdMsg);

            pause(0.1);
            ranges = scan.ranges;
            angles = scan.angle_min : scan.angle_increment : scan.angle_min + (length(ranges) - 1)*scan.angle_increment;
            
            % Remove invalid ranges
            valid = isfinite(ranges);
            ranges = ranges(valid);
            angles = angles(valid);
            
            % Convert to Cartesian coordinates
            x = ranges .* cos(angles);
            y = ranges .* sin(angles);

            % Filter left side points (60° to 120°)
            left_mask = angles > deg2rad(75) & angles < deg2rad(105);
            x_left = x(left_mask);
            y_left = y(left_mask);
            
            % Filter right side points (-120° to -60°)
            right_mask = angles > deg2rad(-75) & angles < deg2rad(-105);
            x_right = x(right_mask);
            y_right = y(right_mask);

            % Fit left wall
            if length(x_left) > 5
                p_left = polyfit(x_left, y_left, 1); % [slope, intercept]
                angle_left = atan(p_left(1));        % orientation of left wall
            else
                angle_left = NaN;
            end
            
            % Fit right wall
            if length(x_right) > 5
                p_right = polyfit(x_right, y_right, 1);
                angle_right = atan(p_right(1));
            else
                angle_right = NaN;
            end
            if isfinite(angle_left) && isfinite(angle_right)
                % Average the wall angles (modulo pi, since walls face opposite)
                wall_angle = wrapToPi((angle_left + angle_right)/2);
                
                % Rotate 90° to get hallway direction
                hallway_heading = wrapToPi(wall_angle + 0);
            else 
                hallway_heading = orientation(3)
            end
            quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
            orientation = quat2eul(quat);
            pause(0.1);
            init_heading = wrapToPi(hallway_heading - orientation(3));
            pause(0.1);
            quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
            orientation = quat2eul(quat);  % Convert quaternion to Euler angles
            heading =  wrapToPi(orientation(3) - init_heading);% - init_heading; % Extract heading (yaw)
            raw_pos = [pose.position.x pose.position.y];
            relative_pos = raw_pos - init_pos;
            rotation_matrix = [cos(-init_heading), -sin(-init_heading); sin(-init_heading), cos(-init_heading)];
            position = (rotation_matrix * relative_pos')' + start;
            pause(0.1);
        end

        %% Reached goal, circle search and new path
        if (abs(position(1) - goal(1)) < 0.05 && abs(position(2) - goal(2)) < 0.05 && i == length(x_interp))
            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = 0;
            send(cmdPub, cmdMsg);
            if prog == 0
                des_ang = 0;
            else
                des_ang = -pi/2;
            end
            dir = 1;
            if heading > 0
                dir = -1;
            end
            pause(0.1);

            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = 0.5 * dir;
            send(cmdPub, cmdMsg);

            
            while abs(heading) - des_ang > 0.04
                pause(0.1);
                quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
                orientation = quat2eul(quat);  % Convert quaternion to Euler angles
                heading =  wrapToPi(orientation(3) - init_heading);
            end
            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = 0;
            send(cmdPub, cmdMsg);

            pause(0.5);
            known_pose_before = [position(1) position(2) heading]
            odom_pose_before = [pose.position.x pose.position.y orientation(3)]

            circleSearchFunc(prog, cmdPub, init_heading);
            pause(0.5);
            prog = prog + 1;
            if prog == 2
                continue
            end
            dir = 1;
            if heading > 0
                dir = -1;
            end
            pause(0.1);
            quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
            orientation = quat2eul(quat);  % Convert quaternion to Euler angles
            heading =  wrapToPi(orientation(3) - init_heading);
            pause(0.5);
            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = 0.5 * dir;
            send(cmdPub, cmdMsg);

            
            while abs(heading) - des_ang > 0.04
                pause(0.1);
                quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
                orientation = quat2eul(quat);  % Convert quaternion to Euler angles
                heading =  wrapToPi(orientation(3) - init_heading);
            end

            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = 0;
            send(cmdPub, cmdMsg);
            pause(2);

            
            %map = projmap(prog);
            i = 0;
            

            scan_right = scan.ranges(265:275);
            
            avg_right = double(mean(scan_right));
            y_avg = avg_right;
            pause(0.1);
            quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
            orientation = quat2eul(quat);  % Convert quaternion to Euler angles
            heading =  wrapToPi(orientation(3) - init_heading);% - init_heading; % Extract heading (yaw)
            raw_pos = [pose.position.x pose.position.y];
            relative_pos = raw_pos - init_pos;
            rotation_matrix = [cos(-init_heading), -sin(-init_heading); sin(-init_heading), cos(-init_heading)];
            position = (rotation_matrix * relative_pos')' + start;
            pause(0.5);
            odom_pose_now = [raw_pos(1) raw_pos(2) orientation(3)];
            new_pos = correctPoseAfterSpin(known_pose_before, odom_pose_before, odom_pose_now, y_avg + y_wall);
            start = new_pos(1:2);
            init_pos = raw_pos;
            %init_heading = new_pos(3);
            pause(0.5);
            quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
            orientation = quat2eul(quat);  % Convert quaternion to Euler angles
            heading =  wrapToPi(orientation(3) - init_heading);% - init_heading; % Extract heading (yaw)
            raw_pos = [pose.position.x pose.position.y];
            relative_pos = raw_pos - init_pos;
            rotation_matrix = [cos(-init_heading), -sin(-init_heading); sin(-init_heading), cos(-init_heading)];
            position = (rotation_matrix * relative_pos')' + start;
            pause(0.5);
            start2 = [position(1) position(2)];
            goal = [26.5 5.54];
            path = new_createPRMpath(start2, goal, map);
            
            pause(0.5);
            x_waypoints = path(:,1);
            y_waypoints = path(:,2);
            t_waypoints = 1:length(path);
            
            % Define the finer resolution for interpolation
            t_fine = linspace(1, length(path), 50); % 25 intermediate points
                
            % Interpolate x and y positions
            x_interp = interp1(t_waypoints, x_waypoints, t_fine, 'pchip');
            y_interp = interp1(t_waypoints, y_waypoints, t_fine, 'pchip');
            continue;

        end
        %% Process and plot laser scan data
        cart = rosReadCartesian(scan);  % Convert scan to Cartesian coordinates
        cart = cart * [cos(heading), -sin(heading); sin(heading), cos(heading)]' + position; % Transform based on robot position and heading
        if(toc(time) - lastVisualUpdate > visualUpdateRate)
            visualise = updateScan(visualise, cart);
           
            lastVisualUpdate = toc(time);
        end
        %% Find pos Error
        e_x = position_desired(1) - position(1);
        e_y = position_desired(2) - position(2);
        errorPos = norm(position_desired - position);
        %errorPos = (position_desired - position);
        deltaErrorPos = (errorPos - prevErrorPos);
        integralErrorPos = integralErrorPos + errorPos;

        %% Find heading error
        desiredHeading = atan2(e_y,e_x);
        errorAng = atan2(sin(desiredHeading - heading), cos(desiredHeading - heading));
        %errorAng = desiredHeading - heading;
        deltaErrorAng = (errorAng - prevErrorAng);
        integralErrorAng = integralErrorAng + errorAng;
        
       
        %% PID controller for heading
        angularVelocity = Kp_ang * errorAng + Ki_ang * integralErrorAng + Kd_ang * deltaErrorAng;

        %% PID controller for position
        linearVelocity = Kp_pos * errorPos + Ki_pos * integralErrorPos + Kd_pos * deltaErrorPos;
        linearVelocity = linearVelocity * (1 - min(abs(angularVelocity) / 0.5, 1));
        
        prevErrorAng = errorAng;
        prevErrorPos = errorPos;
        
        %% Obstacle avoidance
        F_total = 0;
        attractive = 0;
        pause(0.1);
        scan_front = [scan.ranges(346:360); scan.ranges(1:15)];
        scan_back = scan.ranges(166:195);
        
        d_0 = 0.4;   % Influence distance
        beta = 0.1;  % Strength of repulsion
        
        % Initialize repulsive force vectors
        fr_front = zeros(length(scan_front), 2);
        fr_back = zeros(length(scan_back), 2);
        
        for j = 1:length(scan_front)
            if scan_front(j) <= d_0 && scan_front(j) > 0  % Valid range
                angle = deg2rad(351 + j - 1);  % Convert to radians (assumes scan angle is known)
                magnitude = -(2 * beta / scan_front(j)^2) * (1/scan_front(j) - 1/d_0);
                
                fr_front(j,1) =  magnitude * cos(angle) * 0.7;  % X component
                fr_front(j,2) =  magnitude * sin(angle) * 2;  % Y component
            end
        end
        
        for j = 1:length(scan_back)
            if scan_back(j) <= d_0 && scan_back(j) > 0
                angle = deg2rad(171 + j - 1);
                magnitude = -(2 * beta / scan_back(j)^2) * (1/scan_back(j) - 1/d_0);
                
                fr_back(j,1) = magnitude * cos(angle);
                fr_back(j,2) = magnitude * sin(angle) * 1.5;
                
            end
        end
        attractive = (position_desired - position);  % Vector to target
        attractive = attractive / norm(attractive);  % Normalize

        % Compute total repulsive force
        F_total = sum(fr_front,1) + sum(fr_back,1); % + attractive;

        
        repulsiveX = F_total(1) * 0.7;
        repulsiveY = F_total(2) * 1.2;
        


        

        %% Obstacle Avoidance
        %linearVelocity = 0;
        %angularVelocity = 0;
       
        if norm(F_total(1:2)) > 1e-3
            linearVelocity = linearVelocity + repulsiveX;
            angularVelocity = angularVelocity - repulsiveY * 2.5;
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
    disp("Reached goal")
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.linear.x = 0;
    cmdMsg.angular.z = 0;
    send(cmdPub, cmdMsg);
catch ME
    % Stop the robot
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.linear.x = 0;
    cmdMsg.angular.z = 0;
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

function imageCallback(message)
    % Use global variable to store laser scan data
    global image

    % Save the laser scan message
    image = rosReadImage(message);
end

function circleSearchFunc(prog, cmdPub, init_heading)
    global image pose
    
    
    done = 0;
    distance = 0;
    
    if prog == 0
        fileName = "B.png";
        des_ang = 0;
    else
        fileName = "C.png";
        des_ang = -pi/2;
    end
    turn_speed = 0.5;
    while done == 0
        
        turn_speed = turn_speed * -1;
        i = 0;
        while (all(distance == 0) && i < 3)
    
    
            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = turn_speed;
            send(cmdPub, cmdMsg);
            pause(pi);
    
    
            if i == 0 || i == 2
                cmdMsg = ros2message('geometry_msgs/Twist');
                cmdMsg.linear.x = 0;
                cmdMsg.angular.z = 0;
                send(cmdPub, cmdMsg);
                pause(0.5)
                distance = circleIdentify(image);
            end
            if any(distance ~= 0)
                %imshow(image);
                continue
            end
            
            i = i + 1;
        end
        if (all(distance == 0))
            quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
            orientation = quat2eul(quat);  % Convert quaternion to Euler angles
            heading =  wrapToPi(orientation(3) - init_heading);
            pause(0.1);
            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = turn_speed;
            send(cmdPub, cmdMsg);
    
            while abs(heading) - des_ang > 0.04 && all(distance == 0)
                pause(0.1);
                quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
                orientation = quat2eul(quat);  % Convert quaternion to Euler angles
                heading =  wrapToPi(orientation(3) - init_heading);
            end
            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = 0;
            send(cmdPub, cmdMsg);
        end
    
    
        if (all(distance == 0))
            disp("Drive forward");
            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0.2;
            cmdMsg.angular.z = 0;
            send(cmdPub, cmdMsg);
            pause(1);
            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = 0;
            send(cmdPub, cmdMsg);
        end
    
        if (any(distance ~= 0))
            if (abs(distance - 1) < 0.05 || abs(1 - distance) < 0.05)
                im = image;
                imwrite(im, fileName);
            elseif (any(distance < 1))
                
    
                cmdMsg = ros2message('geometry_msgs/Twist');
                cmdMsg.linear.x = -0.1;
                cmdMsg.angular.z = 0;
                send(cmdPub, cmdMsg);
                while distance < 0.95
                    pause(0.1);
                    im = image;
                    distance = circleIdentify(im);
    
                end
                if any(distance ~= 0)
                    cmdMsg = ros2message('geometry_msgs/Twist');
                    cmdMsg.linear.x = 0;
                    cmdMsg.angular.z = 0;
                    send(cmdPub, cmdMsg);
                    im = image;
                    imwrite(im, fileName);
                    done = 1;
                end
    
            else
                
    
                cmdMsg = ros2message('geometry_msgs/Twist');
                cmdMsg.linear.x = 0.1;
                cmdMsg.angular.z = 0;
                send(cmdPub, cmdMsg);
                while distance > 1.1
                    pause(0.1)
                    im = image;
                    distance = circleIdentify(im);
    
                end
                if any(distance ~= 0)
                    cmdMsg = ros2message('geometry_msgs/Twist');
                    cmdMsg.linear.x = 0;
                    cmdMsg.angular.z = 0;
                    send(cmdPub, cmdMsg);
                    im = image;
                    imwrite(im,fileName);
                    done = 1;
                end
                if (done == 0)
                    quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
                    orientation = quat2eul(quat);  % Convert quaternion to Euler angles
                    heading =  wrapToPi(orientation(3) - init_heading);
                    pause(0.1);
                    cmdMsg = ros2message('geometry_msgs/Twist');
                    cmdMsg.linear.x = 0;
                    cmdMsg.angular.z = turn_speed;
                    send(cmdPub, cmdMsg);
            
                    while abs(heading) - des_ang > 0.04 && all(distance == 0)
                        pause(0.1);
                        quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
                        orientation = quat2eul(quat);  % Convert quaternion to Euler angles
                        heading =  wrapToPi(orientation(3) - init_heading);
                    end
                    cmdMsg = ros2message('geometry_msgs/Twist');
                    cmdMsg.linear.x = 0;
                    cmdMsg.angular.z = 0;
                    send(cmdPub, cmdMsg);
                end
            end
            
        end
    end
end
function corrected_pose = correctPoseAfterSpin(known_pose_before, odom_pose_before, odom_pose_now, corrected_y)
% correctPoseAfterSpin - Estimate current global pose using odometry and y-correction
%
% Inputs:
%   known_pose_before: 1x3 vector [x y theta] global pose before spin
%   odom_pose_before:  1x3 vector [x y theta] odometry before spin
%   odom_pose_now:     1x3 vector [x y theta] odometry after spin
%   corrected_y:       scalar - y position estimated from wall scan
%
% Output:
%   corrected_pose:    1x3 vector [x y theta] corrected global pose

    % Step 1: Compute odometry delta (robot frame movement)
    delta_x = odom_pose_now(1) - odom_pose_before(1);
    delta_y = odom_pose_now(2) - odom_pose_before(2);
    delta_theta = odom_pose_now(3) - odom_pose_before(3);

    % Step 2: Rotate delta into global frame using known_pose_before orientation
    theta = known_pose_before(3);
    global_dx = cos(theta) * delta_x - sin(theta) * delta_y;
    global_dy = sin(theta) * delta_x + cos(theta) * delta_y;

    % Step 3: Add global delta to known pose
    corrected_x = known_pose_before(1) + global_dx;
    corrected_y_final = corrected_y;  % Use wall-corrected y
    corrected_theta = wrapToPi(known_pose_before(3) + delta_theta);

    % Output
    corrected_pose = [corrected_x, corrected_y_final, corrected_theta];
end


%%
cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.linear.x = 0;
    cmdMsg.angular.z = 0;
    send(cmdPub, cmdMsg);
    
    % Clean up ROS subscriptions
    clear odomSub scanSub