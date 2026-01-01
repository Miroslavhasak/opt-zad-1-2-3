function [A, B, C] = create_pendulum_model(theta)
% CREATE_PENDULUM_MODEL Constructs state-space matrices for a damped pendulum model.
%
%   [A, B, C] = CREATE_PENDULUM_MODEL(theta)
%   returns the state-space matrices (A, B, C) of a linearized damped pendulum
%   system.
%
%   INPUT:
%     theta - Parameter vector containing the pendulum parameters:
%             theta(1) = K       (gain factor)
%             theta(2) = a       (damping coefficient)
%             theta(3) = omega_0 (natural frequency)
%             theta(4) = b       (damping ratio or friction coefficient)
%
%   OUTPUT:
%     A         - State matrix (3x3)
%     B         - Input matrix (3x1)
%     C         - Output matrix (1x3)
%
%   MODEL DESCRIPTION:
%     The pendulum system is modeled in state-space form as:
%        x_dot = A*x + B*u
%        y     = C*x
%
%     where the state vector x might represent angle, angular velocity, and an auxiliary state.
%
%   DETAILS OF MATRICES:
%     A = [-a,    0,          0;
%           0,    0,          1;
%         omega_0^2, -omega_0^2, -2*b*omega_0];
%
%     B = [K*a; 0; 0];
%
%     C = [0, 1, 0];
%

% Unpack parameters
K = theta(1);
a = theta(2);
omega_0 = theta(3);
b = theta(4);

% State matrix A
A = [-a, 0, 0;
      0, 0, 1;
      omega_0^2, -omega_0^2, -2*b*omega_0];

% Input matrix B
B = [K*a; 0; 0];

% Output matrix C
C = [0, 1, 0];

end