function distance = circleIdentify()
    im = image;
    HSV = rgb2hsv(im);  % Correct usage

    % Extract channels
    hue = HSV(:,:,1);
    saturation = HSV(:,:,2);
    value = HSV(:,:,3);

    % Create masks for red color
    %blue_mask = (hue > 0.55) & (hue < 0.7);  % Blue
    %green_mask = (hue > 0.20) & (hue < 0.50); % Green
    %red_mask = (hue > 0.95) | (hue < 0.05);
    orange_mask = (hue > 0.0) & (hue < 0.0); % Orange
    purple_mask = (hue > 0.0) & (hue < 0.0); % Purple

    color_mask = orange_mask | purple_mask;
    saturation_mask = saturation > 0.2;  % Ensure it's not a dull color
    value_mask = value > 0.1;  % Ensure brightness

    % Combine masks
    mask =  color_mask & saturation_mask & value_mask;

    % Apply mask to each color channel
    red = im(:,:,1) .* uint8(mask);
    green = im(:,:,2) .* uint8(mask);
    blue = im(:,:,3) .* uint8(mask);

    % Combine back into an image
    im_masked = cat(3, red, green, blue);

    imG = im2gray(im_masked);
    imBW = imG > 0.5;
    se = strel('disk',5);
    imBW = imopen(imBW, se);
    imBW = imclose(imBW, se);

    [centers, radii, metric] = imfindcircles(imBW, [10 500], 'ObjectPolarity','bright', 'Sensitivity',0.87);
    circle_diameter = radii * 2
    f = 1247;
    if (isempty(circle_diameter))
        circle_diameter = 0;
    end
    distance = (f * 0.1) / circle_diameter
end