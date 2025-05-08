function velMsg = obstacle_avoid(ranges, angles)
    % Filter: only front/back (±45°)
    front_mask = angles > -pi/4 & angles < pi/4;
    back_mask  = angles > 3*pi/4 | angles < -3*pi/4;

    repulse = [0, 0];

    % Front repulsion
    f_ranges = ranges(front_mask);
    f_angles = angles(front_mask);
    for i = 1:length(f_ranges)
        if f_ranges(i) < 0.8
            repulse = repulse - [cos(f_angles(i)), sin(f_angles(i))] / f_ranges(i)^2;
        end
    end

    % Back repulsion
    b_ranges = ranges(back_mask);
    b_angles = angles(back_mask);
    for i = 1:length(b_ranges)
        if b_ranges(i) < 0.8
            repulse = repulse - [cos(b_angles(i)), sin(b_angles(i))] / b_ranges(i)^2;
        end
    end

    angle = atan2(repulse(2), repulse(1));
    strength = norm(repulse);

    % Avoid force
    velMsg = ros2message("geometry_msgs/Twist");
    velMsg.linear.x = min(0.1, 0.5 * strength);
    velMsg.angular.z = angle;
end
