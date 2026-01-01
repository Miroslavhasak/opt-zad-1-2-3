% --- CONFIGURATION ---
addpath("..\geometry");
close all
clear
folder = 'C:/00/frames/';

voxelSize = 0.3;
prob_hit=0.7;
prob_miss=0.4;
map = Map(voxelSize,prob_hit,prob_miss);

% --- SETUP ---
files = dir(fullfile(folder, '*'));
filenames = sortFilenamesByNumericPart(files);
numFrames = numel(filenames);

pose.R=eye(3);  % First pose
pose.t=zeros(3,1);
poses=pose;
trajectory = zeros(numFrames, 3);     % Pre-allocate trajectory

% --- VISUALIZATION SETUP ---
figure; hold on;
pcshow(pointCloud(zeros(1, 3))); axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');

hPc = findobj(gca, 'Type', 'Scatter');
hTraj = plot3(nan, nan, nan, 'w-', 'LineWidth', 2);

% --- MAIN LOOP ---
for i = 50:numFrames
    
    [scan, timestamps] = getNextPointCloud(filenames{i});
    [pose,map] = RegisterScan(map,scan, pose,50,2);
    poses(end + 1) = pose;

   
    [mapPoints, prob] = map.getPoints();
    mapPoints = mapPoints';

    % Update point cloud data
    hPc.XData = mapPoints(:,1);
    hPc.YData = mapPoints(:,2);
    hPc.ZData = mapPoints(:,3);

    % Set per-point colors based on probability
    hPc.CData = prob;    % use probabilities directly
    colormap(jet);       % e.g. jet colormap (blue→red)
    caxis([0 1]);        % map [0,1] probability range to colormap
    colorbar;            % optional, to show scale
    
    T = poses(end);
    trajectory(i, :) =T.t;

        
    hTraj.XData = trajectory(1:i, 1);
    hTraj.YData = trajectory(1:i, 2);
    hTraj.ZData = trajectory(1:i, 3);

    drawnow limitrate;
end


