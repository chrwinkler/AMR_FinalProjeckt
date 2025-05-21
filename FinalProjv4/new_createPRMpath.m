function path = new_createPRMpath(start, goal, map)
%% Initialization - create map, set start and goal points

% robot size -> 15cm x 15cm (xy base, only approx)
robot_size_side = 0.07; % in meters


map.inflate(robot_size_side);


%% Probabilistic Roadmap method
figure('Name','PRM map', 'NumberTitle','off')
prm = mobileRobotPRM(map); % create prm planner
prm.NumNodes = 400;

prm.show() % plot the roadmap
temp_path = [];
while isempty(temp_path)
    temp_path = prm.findpath(start, goal); % query planner for path
end
path = temp_path;

prm.show()

hold on, plot(start(1), start(2), 'r*', 'MarkerSize', 20), text(start(1), start(2), 'START')
hold on, plot(goal(1), goal(2), 'ro', 'MarkerSize', 20), text(goal(1), goal(2), 'GOAL')



end

