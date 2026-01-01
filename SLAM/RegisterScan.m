function [newPose, map] = RegisterScan(map,scan, initialGuess, maxRange, minRange)
    
    % Compute norms of all points (Euclidean distance from origin)
    norms = sqrt(sum(scan.^2, 2));  % Nx1 vector
    % Select inlier points
    scan = scan((norms < maxRange) & (norms > minRange), :);
    scan_downsample=pcdownsample(pointCloud(scan), 'gridAverage', 0.3);
    scan_downsample=scan_downsample.Location;
 
    % Estimate new pose using scan-to-map alignment
    
    newPose = AlignPointsToMap(map, scan_downsample, initialGuess, 3);
    
    scan_transformed = TransformPoints(scan_downsample, newPose);
    % Update local map with new frame
    map.update(newPose.t,scan_transformed);

      
end
