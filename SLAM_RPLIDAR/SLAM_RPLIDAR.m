
close all
clear

slobj = slamteclidar("COM1", 115200); % sample port and baudrate values for Slamtec RPLIDAR A1 model
cleanupObj = onCleanup(@()slobj.delete);
slobj.setRPM(360);
%slobj.setScanMode("Boost");
%Standard"    "Express"    "Boost"    "Stability
slobj.start();


voxelSize = 0.03;
prob_hit = 0.52;
prob_miss = 0.45;
map = MapICP(voxelSize,prob_hit,prob_miss);

% --- VISUALIZATION SETUP ---
figure; hold on;
pcshow(pointCloud(zeros(1, 3)),'MarkerSize', 70); axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(0, 90)

hPc = findobj(gca, 'Type', 'Scatter');
hTraj = plot3(nan, nan, nan, 'w-', 'LineWidth', 1.5);
hPos = plot3(nan, nan, nan, 'or', 'LineWidth', 1.5);

arrowLength = 0.5; % length of the orientation arrow
hArrow = quiver3(0,0,0, 0,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 2);

trajectory =[];

t_hat=[0;0;0];
R_hat=eye(3);

maxRange=4.0;
minRange=0.5;

while 1
    
    [scan] = slobj.read();
 
    scan=scan.Cartesian;
    scan=[scan, zeros(size(scan,1),1)];

    norms = sqrt(sum(scan.^2, 2)); 
    scan = scan((norms < maxRange) & (norms > minRange),:);
    
    [t_hat,R_hat] = map.alignScan(scan,0.2,t_hat,R_hat,1e-4,1e-4,1e-3);
   
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
    
    trajectory = [trajectory; t_hat'];  

    hTraj.XData = trajectory(:, 1);
    hTraj.YData = trajectory(:, 2);
    hTraj.ZData = trajectory(:, 3);

    hPos.XData = t_hat(1);
    hPos.YData = t_hat(2);
    hPos.ZData = t_hat(3);

    hArrow.XData = t_hat(1);
    hArrow.YData = t_hat(2);
    hArrow.ZData = t_hat(3);
    hArrow.UData = arrowLength * R_hat(1,1);
    hArrow.VData = arrowLength * R_hat(1,2);
    hArrow.WData = arrowLength * R_hat(1,3);

    drawnow limitrate;
end

%%

% Stop scanning
slobj.stop();

% delete and clear object
slobj.delete();
clear slobj;



