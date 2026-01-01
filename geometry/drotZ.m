function [dRz] = drotZ(theta)

%TODO

dRz = [-sin(theta)  -cos(theta)   0;
            cos(theta)  -sin(theta)   0;
                 0            0       1*0]; % dRz definuje deriváciu rotačnej matice podľa uhla theta, 
% hovorí algoritmu ako rýchlo a akým smerom sa pohnú body v priestore ak zmeníme uhol otočenia

end