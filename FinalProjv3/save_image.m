function save_image(img)
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = fullfile(pwd, ['circle_', timestamp, '.png']);
    imwrite(img, filename);
end
