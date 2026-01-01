function [Ad, Bd, Cd] = ss_discretize(A, B, C, Ts)
%SS_DISCRETIZE Discretizes a continuous-time state-space system
%
%   [Ad, Bd, Cd] = ss_discretize(A, B, C, Ts) returns the discrete-time
%   state-space matrices (Ad, Bd, Cd) given the continuous-time system
%   matrices (A, B, C) and the sampling time Ts.
%
%   Inputs:
%       A  - Continuous-time state matrix (n x n)
%       B  - Continuous-time input matrix (n x m)
%       C  - Continuous-time output matrix (p x n)
%       Ts - Sampling time (scalar)
%
%   Outputs:
%       Ad - Discrete-time state matrix (n x n)
%       Bd - Discrete-time input matrix (n x m)
%       Cd - Discrete-time output matrix (p x n)
%
%   Method:
%       Ad = expm(A*Ts) computes the matrix exponential for the state transition.
%       Bd = A \ (Ad - I) * B computes the discrete-time input matrix using
%            the relationship Bd = integral_0^Ts (e^(A*t) dt) * B
%       Cd = C (output matrix remains the same for discretization)

% Compute discrete-time state matrix using matrix exponential
Ad = expm(A*Ts);

% Compute discrete-time input matrix
Bd = A \ ((Ad - eye(size(A))) * B);

% Output matrix remains unchanged
Cd = C;

end