function [dRy] = drotY(theta)

%TODO

dRy = [-sin(theta)   0   cos(theta);
                0        0        0;
           -cos(theta)   0   -sin(theta)];
end
