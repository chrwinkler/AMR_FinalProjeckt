function danger = check_obstacle(ranges, angles)
    front_mask = angles > -pi/6 & angles < pi/6;
    front_ranges = ranges(front_mask);
    danger = any(front_ranges < 0.6);
end
