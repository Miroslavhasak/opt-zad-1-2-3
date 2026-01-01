
function [Rz] = rotZ(theta)

%TODO
% Táto matica hovorí ako sa má otočiť bod v priestore o určitý uhol theta
% tak aby sa krútil okolo zvislej osi z
Rz=[cos(theta)  -sin(theta)   0;
          sin(theta)   cos(theta)   0;
               0            0       1]; % definuje rotačnú maticu okolo osi z v 3d priestore

end