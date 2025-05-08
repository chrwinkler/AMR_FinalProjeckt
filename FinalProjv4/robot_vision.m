%% Clear workspace, command window, and close all figures
clear all
clc
close all

%% Declare global variables for robot pose and laser scan data
global pose poseOffset scan image

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
pause(0.5);
    
try
    %% Define publishers
    cmdPub = ros2publisher(controlNode, '/cmd_vel', 'geometry_msgs/Twist');
    
    %% Create figure for TurtleBot's data
    visualise = TurtleBotVisualise();
    
    %% Initialize array for desired positions
    positionDesired = [1; 1];

    %% Calculate offset
    quatOffset = [poseOffset.orientation.x poseOffset.orientation.y poseOffset.orientation.z poseOffset.orientation.w];
    orientationOffset = quat2eul(quatOffset);  % Convert offset quaternion to Euler angles
    headingOffset = orientationOffset(3); % Extract offset heading (yaw)

    %% Calculate transformations for offset
    positionOffset = [poseOffset.position.x; poseOffset.position.y];
    R_W2R = [cos(-headingOffset), -sin(-headingOffset); sin(-headingOffset), cos(-headingOffset)];
    t_R2V = -R_W2R * positionOffset;
    R_R2V = [cos(headingOffset), -sin(headingOffset); sin(headingOffset), cos(headingOffset)]';
    
    %% Infinite loop for real-time visualization, until the figure is closed
    while true
        %% Visialise desired position
        %visualise = updatePositionDesired(visualise, positionDesired);

        %% Get the robot's current position and heading
        position = [pose.position.x; pose.position.y];
        quat = [pose.orientation.x pose.orientation.y pose.orientation.z pose.orientation.w];
        orientation = quat2eul(quat);  % Convert quaternion to Euler angles
        heading = orientation(3); % Extract heading (yaw)

        %% Apply offset
        position = R_R2V * position + t_R2V;
        heading = heading - headingOffset; % Offset heading

        %% Visualise the robot
        %visualise = updatePose(visualise, position, heading);
    
        %% Process and plot laser scan data
        cart = rosReadCartesian(scan);  % Convert scan to Cartesian coordinates
        cart = cart * [cos(heading), -sin(heading); sin(heading), cos(heading)]' + position'; % Transform based on robot position and heading
        %visualise = updateScan(visualise, cart);

        %% Visualise image
        im = image;
        HSV = rgb2hsv(im);  % Correct usage

        % Extract channels
        hue = HSV(:,:,1);
        saturation = HSV(:,:,2);
        value = HSV(:,:,3);
        
        % Create masks for red color
        blue_mask = (hue > 0.55) & (hue < 0.7);  % Blue
        %green_mask = (hue > 0.20) & (hue < 0.50); % Green
        %red_mask = (hue > 0.95) | (hue < 0.05);
        %color_mask = blue_mask | green_mask;
        saturation_mask = saturation > 0.2;  % Ensure it's not a dull color
        value_mask = value > 0.1;  % Ensure brightness
        
        % Combine masks
        mask =  blue_mask & saturation_mask & value_mask;
        
        % Apply mask to each color channel
        red = im(:,:,1) .* uint8(mask);
        green = im(:,:,2) .* uint8(mask);
        blue = im(:,:,3) .* uint8(mask);
        
        % Combine back into an image
        im_masked = cat(3, red, green, blue);

        imG = im2gray(im_masked);
        imBW = imG > 0.5;
        se = strel('disk',5);
        imBW = imopen(imBW, se);
        imBW = imclose(imBW, se);

        [centers, radii, metric] = imfindcircles(imBW, [10 500], 'ObjectPolarity','bright', 'Sensitivity',0.87);
        circle_diameter = radii * 2
        f = 1247;
        if (isempty(circle_diameter))
            circle_diameter = 0;
        end
        distance = (f * 0.1) / circle_diameter
        % Visualize result
        visualise = updateImage(visualise, im, im_masked, imBW);

        %% PID controller for heading
        angularVelocity = 0.0;

        %% PID controller for position
        linearVelocity = 0.0;
    
        %% Publish velocity commands
        cmdMsg = ros2message('geometry_msgs/Twist');
        cmdMsg.linear.x = clip(linearVelocity, -0.1, 0.1);
        cmdMsg.angular.z = clip(angularVelocity, -1.0, 1.0);
        % send(cmdPub, cmdMsg);
    
        %% Pause to visualize and delete old plots
        pause(0.1)
    
        %% Exit the loop if the figure is closed
        if size(findobj(visualise.figAvatar)) == 0 | size(findobj(visualise.figImage)) == 0
            ME = MException('NonExeption:EndProgram', 'The program was closed.');
            throw(ME)
        end
    end
catch ME
    % Stop the robot
    cmdMsg = ros2message('geometry_msgs/Twist');
    cmdMsg.Linear.x = 0;
    cmdMsg.Angular.z = 0;
    send(cmdPub, cmdMsg);

    % Close all figures
    close all
    
    % Clean up ROS subscriptions
    clear odomSub scanSub imageSub

    % Show the error
    if ~strcmp(ME.identifier, 'NonExeption:EndProgram')
        rethrow(ME)
    end
end 

% %% Callback functions
function odomCallback(message)
    % Use global variable to store the robot's position and orientation
    global pose poseOffset

    % Extract position and orientation data from the ROS message
    pose = message.pose.pose;

    if isempty(poseOffset)
        poseOffset = message.pose.pose;
    end
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