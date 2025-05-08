function [velMsg, current_index] = path_follow(pose, prm, current_index, velPub)
    waypoints = prm.Path;
    kp = 0.5; ka = 1.0;

    if current_index > size(waypoints, 1)
        velMsg = ros2message("geometry_msgs/Twist");
        return;
    end

    goal = waypoints(current_index, :);
    dx = goal(1) - pose(1);
    dy = goal(2) - pose(2);
    distance = norm([dx dy]);

    if distance < 0.2
        current_index = current_index + 1;
    end

    angle_to_goal = atan2(dy, dx);
    angle_diff = wrapToPi(angle_to_goal - pose(3));

    % PID control
    velMsg = ros2message("geometry_msgs/Twist");
    velMsg.linear.x = kp * distance;
    velMsg.angular.z = ka * angle_diff;
end
