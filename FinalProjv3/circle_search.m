function circle_search(velPub, imageSub, scanSub, odomSub)
    disp("🔍 Starting circle search...");

    found = false;
    max_attempts = 50; % avoid infinite loops
    attempts = 0;

    while ~found && attempts < max_attempts
        % Small forward + rotate
        velMsg = ros2message("geometry_msgs/Twist");
        velMsg.linear.x = 0.05;
        velMsg.angular.z = 0.2 * (-1)^mod(attempts, 2); % left/right alternate
        send(velPub, velMsg);
        pause(0.5);
        velMsg.linear.x = 0; velMsg.angular.z = 0;
        send(velPub, velMsg);

        % Get camera image
        imgMsg = receive(imageSub, 3);
        img = rosReadImage(imgMsg);

        % Try to detect a circle
        [found, center] = detect_circle(img);
        attempts = attempts + 1;
    end

    if found
        disp("✅ Circle found — approaching...");
        % Approach until 1m using LIDAR
        while true
            scanMsg = receive(scanSub);
            ranges = scanMsg.ranges;
            angles = linspace(scanMsg.angle_min, scanMsg.angle_max, length(ranges));
            front = ranges(angles > -0.1 & angles < 0.1);
            dist = min(front);

            if dist < 1.0
                break;
            end

            velMsg = ros2message("geometry_msgs/Twist");
            velMsg.linear.x = 0.1;
            send(velPub, velMsg);
            pause(0.2);
        end

        % Stop and save image
        send(velPub, ros2message("geometry_msgs/Twist"));
        save_image(img);
        disp("📸 Image saved. Circle search complete.");
    else
        disp("❌ Circle not found.");
    end
end
