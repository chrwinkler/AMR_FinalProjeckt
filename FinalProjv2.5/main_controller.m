function control_center()
    % --- Init ROS2 & Topics ---
    %ros2init;
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
    scanSub = ros2subscriber(controlNode,'/scan', 'sensor_msgs/LaserScan');
    odomSub = ros2subscriber(controlNode,'/odom', 'nav_msgs/Odometry');
    velPub = ros2publisher(controlNode,'/cmd_vel', 'geometry_msgs/Twist');
    
    % --- Load Map A→B ---
    prog = "AB";
    map = projectMap(prog);
    start = [0.3, 0.3];
    goal = [3.8, 2.2];

    prm = createPRMpath(start, goal, map);
    visualiser = TurtleBotVisualise(map);

    % --- Initialize Control State ---
    robot_state = "FOLLOW";
    current_index = 1;
    goal_reached_flag = false;

    % --- Main Control Loop ---
    while ~goal_reached_flag
        % Read sensors
        odomMsg = receive(odomSub);
        scanMsg = receive(scanSub);
        [pose, ranges, angles] = readSensorData(odomMsg, scanMsg);

        % Update plot
        visualiser.update(pose, ranges);

        % --- Behavior Switching ---
        switch robot_state
            case "FOLLOW"
                if check_obstacle(ranges, angles)
                    robot_state = "AVOID";
                else
                    [velMsg, current_index] = path_follow(pose, prm, current_index, velPub);
                end

            case "AVOID"
                if ~check_obstacle(ranges, angles)
                    robot_state = "FOLLOW";
                else
                    velMsg = obstacle_avoid(ranges, angles);
                end

            case "STOP"
                velMsg = ros2message("geometry_msgs/Twist");
        end

        % --- Check if goal reached ---
        if goal_reached(pose, goal)
            if isequal(goal, [3.8, 2.2])  % Reached B → switch to C
                disp("🗺 Switching to map BC...");
                prog = "BC";
                map = projmap(prog);
                start = pose(1:2);
                goal = [0.5, 3.5];  % Final goal in C
                prm = new_createPRMpath(map, start, goal);
                visualiser = TurtleBotVisualise(map);
                current_index = 1;
                robot_state = "FOLLOW";
                continue;
            else
                disp("🎯 Final goal reached!");
                robot_state = "STOP";
                goal_reached_flag = true;
                velMsg = ros2message("geometry_msgs/Twist"); % Stop
            end
        end

        % Send movement command
        send(velPub, velMsg);
        pause(0.1);
    end
end

