function [found, center] = detect_circle(img)
    gray = rgb2gray(img);
    blurred = imgaussfilt(gray, 2);
    [centers, radii] = imfindcircles(blurred, [10 60], 'ObjectPolarity','dark');

    if ~isempty(centers)
        found = true;
        center = centers(1,:);
    else
        found = false;
        center = [];
    end
end
