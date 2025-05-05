function map = projmap()

    robot_size = 0.07;
    image = imread("shannon1.png");
    grayimage = rgb2gray(image);
    bwimage = grayimage < 0.5;

    grid = binaryOccupancyMap(bwimage, 21);
    %grid.inflate(robot_size);
    show(grid)
    map = grid;
end