function map = projmap(prog)

    robot_size = 0.07;
    
    if prog == 0
        image = imread("shannonAB.png");
    end
    if prog == 1
        image = imread("shannonBC.png")
    end
    grayimage = rgb2gray(image);
    bwimage = grayimage < 0.5;

    grid = binaryOccupancyMap(bwimage, 16.5);
    %grid.inflate(robot_size);
    show(grid)
    map = grid;
end