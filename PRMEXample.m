% makeprm.m

% Load map
projmap; % This should create variable 'map'

% Create PRM
prm = mobileRobotPRM(map, 150); % adjust node count as needed
prm.ConnectionDistance = 3.0;

% Define start and goal (A → B)
startLocation = [xA, yA]; % pick based on your map
endLocation = [xB, yB];

% Find path
path = findpath(prm, startLocation, endLocation);

% Rebuild PRM if needed
while isempty(path)
    prm.NumNodes = prm.NumNodes + 50;
    update(prm);
    path = findpath(prm, startLocation, endLocation);
end

% Show map and path
show(prm)
hold on
plot(path(:,1), path(:,2), 'r-', 'LineWidth', 2)

% Optionally save path or prm object
save('pathAB.mat', 'path');
