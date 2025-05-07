function control_center()
    % --- Init ROS2 & Topics ---
    ros2init;
    scanSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');
    odomSub = rossubscriber('/odom', 'nav_msgs/Odometry');
    velPub = ros2publisher('/cmd_vel', 'geometry_msgs/Twist');
    imageSub = rossubscriber('/camera/image_raw', 'sensor_msgs/Image');

    % --- Load Initial Map A→B ---
    prog = "AB";
    map = projmap(prog);
    start = [0.3, 0.3];
    goal = [3.8, 2.2];

    prm = new_createPRMpath(map, start, goal);
    visualiser = TurtleBotVisualise(map);

    % --- Control State ---
    robot_state = "FOLLOW";
    current_index = 1;
    goal_reached_flag = false;

    % --- Main Loop ---
    while ~goal_reached_flag
        % --- Get Sensor Data ---
        odomMsg = receive(odomSub);
        scanMsg = receive(scanSub);
        [pose, ranges, angles] = readSensorData(odomMsg, scanMsg);

        % --- Visualise ---
        visualiser.update(pose, ranges);

        % --- Control Behavior ---
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

        % --- Check if Goal Reached ---
        if goal_reached(pose, goal)
            if isequal(goal, [3.8, 2.2])  % Reached B
                disp("📍 Reached B — starting circle search...");
                circle_search(velPub, imageSub, scanSub, odomSub);  % NEW STEP

                % Then switch to map BC and go to C
                disp("🗺 Switching to map BC...");
                prog = "BC";
                map = projmap(prog);
                start = pose(1:2);
                goal = [0.5, 3.5];
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

        % --- Send Movement Command ---
        send(velPub, velMsg);
        pause(0.1);
    end
end
