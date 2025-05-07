function [pose, ranges, angles] = readSensorData(odomMsg, scanMsg)
    % Pose from odometry
    pos = odomMsg.pose.pose.position;
    orient = odomMsg.pose.pose.orientation;
    angles_q = eulerd(quaternion([orient.w orient.x orient.y orient.z]), 'ZYX', 'frame');
    pose = [pos.x, pos.y, angles_q(1)]; % [x, y, theta]

    % Ranges and angles from LIDAR
    ranges = scanMsg.ranges;
    angle_min = double(scanMsg.angle_min);
    angle_increment = double(scanMsg.angle_increment);
    num_readings = length(ranges);
    angles = angle_min + (0:num_readings-1) * angle_increment;
end
