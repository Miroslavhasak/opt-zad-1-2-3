function [A, B, C] = create_thermal_model(theta)
% CREATE_THERMAL_MODEL Constructs state-space matrices for a thermal system.
%
%   [A, B, C] = CREATE_THERMAL_MODEL(theta)
%   returns the state-space matrices (A, B, C) of a thermal process model
%   
%
%   INPUT:
%     theta - Parameter vector containing the thermal model parameters:
%             theta(1) = K   (input gain)
%             theta(2) = a1  (thermal conduction/diffusion coefficient 1)
%             theta(3) = a2  (thermal conduction/diffusion coefficient 2)
%             theta(4) = a3  (thermal conduction/diffusion coefficient 3)
%
%   OUTPUT:
%     A         - State matrix (3x3)
%     B         - Input matrix (3x1)
%     C         - Output matrix (1x3)
%
%
%   MODEL DESCRIPTION:
%     The thermal system is modeled as a linear state-space system:
%        x_dot = A*x + B*u
%        y     = C*x
%
%     where the state vector x represents temperature states at different nodes
%     or layers, u is the input (e.g., heat input), and y is the measured output
%     (here, the temperature of the third node).
%
%   DETAILS OF MATRICES:
%     A = [-a1,    0,      0;
%           a2,   -a2,     0;
%           0,     a3,    -a3];
%
%     B = [K*a1; 0; 0];
%
%     C = [0, 0, 1];
%

% Unpack parameters
K = theta(1);
a1 = theta(2);
a2 = theta(3);
a3 = theta(4);

% State matrix A
A = [-a1, 0, 0;
      a2, -a2, 0;
      0, a3, -a3];

% Input matrix B
B = [K*a1; 0; 0];

% Output matrix C (measuring the third state variable)
C = [0, 0, 1];



end