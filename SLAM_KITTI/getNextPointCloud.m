function [points, timestamps] = getNextPointCloud(filename)
% Reads a binary point cloud file and returns Nx3 points and normalized timestamps

    fid = fopen(filename, 'rb');
    if fid == -1
        error('Unable to open file: %s', filename);
    end

    % Skip header until "end_header"
    while true
        line = fgetl(fid);
        if strcmp(strtrim(line), 'end_header')
            break;
        end
    end

    % Read binary data: 4 floats per point (x, y, z, timestamp)
    data = fread(fid, [4, inf], 'float32')';
    fclose(fid);

    % Extract points and timestamps
    points = data(:, 1:3);
    rawTimestamps = data(:, 4);

    % Normalize timestamps between 0 and 1
    minTime = min(rawTimestamps);
    maxTime = max(rawTimestamps);
    if maxTime > minTime
        timestamps = (rawTimestamps - minTime) / (maxTime - minTime);
    else
        timestamps = zeros(size(rawTimestamps));
    end
end
