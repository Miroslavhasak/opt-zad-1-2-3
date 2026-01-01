function [Gamma] = MPC_Gamma(r, p)
% MPC_GAMMA Constructs the Gamma matrix used in Model Predictive Control (MPC).
%
%   Gamma = MPC_GAMMA(r, p) returns a block-lower-triangular matrix Gamma
%   of size (r*p)-by-(r*p), which is commonly used in the construction of 
%   prediction models in MPC.
%
%   Inputs:
%       r - The number of outputs or system dimensions.
%       p - The prediction horizon (number of future steps).
%
%   Output:
%       Gamma - A block-lower-triangular matrix with identity matrices on
%               and below the diagonal. Each block is of size r-by-r.
%
%   Structure:
%       Gamma = [ I     0     0   ...    0
%                 I     I     0   ...    0
%                 I     I     I   ...    0
%                 .     .     .    .     .
%                 I     I     I   ...    I ]
%
%   Example:
%       r = 2;
%       p = 3;
%       Gamma = MPC_Gamma(r, p);
%       % Gamma is a 6x6 block-lower-triangular matrix with 2x2 identity blocks.

Gamma = zeros(r*p, r*p);

for i = 1:p
    for j = 1:i
        Gamma(1+(i-1)*r:i*r, 1+(j-1)*r:j*r) = eye(r, r);
    end    
end

end