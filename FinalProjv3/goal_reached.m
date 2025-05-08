function flag = goal_reached(pose, goal)
    distance = norm(goal - pose(1:2));
    flag = distance < 0.3;
end
