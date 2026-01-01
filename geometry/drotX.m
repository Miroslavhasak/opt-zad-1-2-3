function [dRx] = drotX(theta)

%TODO
dRx=[0            0                 0;
           0     -sin(theta)      -cos(theta);
           0      cos(theta)      -sin(theta)];

end