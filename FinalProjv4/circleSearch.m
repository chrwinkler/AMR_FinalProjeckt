function circleSearch(prog)
    done = 0;
    distance = 0;
    prog = 0;
    if prog == 0
        fileName = "B.png"
    else
        fileName = "C.png"
    end
    while done == 0
        t_start = tic()
        while (distance == 0 || t_now >= 4)
            %% Publish velocity commands
            cmdMsg = ros2message('geometry_msgs/Twist');
            cmdMsg.linear.x = 0;
            cmdMsg.angular.z = 1;
            send(cmdPub, cmdMsg);
            pause(1.5708)
            distance = circleIdentify();
            t_now = toc(t_start);
        end
        cmdMsg = ros2message('geometry_msgs/Twist');
        cmdMsg.angular.z = 0;
        send(cmdPub, cmdMsg);
        if (distance ~= 0)
            if (distance - 1 < 0.05)
                im = image;
                imwrite(im, fileName);
            elseif (distance < 1)
                distDif = 1 - distance;
    
                cmdMsg = ros2message('geometry_msgs/Twist');
                cmdMsg.linear.x = -0.1;
    
                send(cmdPub, cmdMsg);
                pause(distDif*10);
                cmdMsg = ros2message('geometry_msgs/Twist');
                cmdMsg.linear.x = 0;
                send(cmdPub, cmdMsg);
                im = image;
                imwrite(im, fileName);
    
            else
                distDif = distance - 1;
    
                cmdMsg = ros2message('geometry_msgs/Twist');
                cmdMsg.linear.x = 0.1;
    
                send(cmdPub, cmdMsg);
                pause(distDif*10);
                cmdMsg = ros2message('geometry_msgs/Twist');
                cmdMsg.linear.x = 0;
                send(cmdPub, cmdMsg);
                im = image;
                imwrite(im,filename = fileName);
            end
            done = 1;
        end
        cmdMsg = ros2message('geometry_msgs/Twist');
        cmdMsg.linear.x = 1;
    
        send(cmdPub, cmdMsg);
        pause(0.5);
        cmdMsg = ros2message('geometry_msgs/Twist');
        cmdMsg.linear.x = 0;
        send(cmdPub, cmdMsg);
    end

end