function D_ = block_diag(D,m)
% BLOCK_DIAG Constructs a block diagonal matrix by repeating a matrix D along the diagonal.
%
%   D_ = BLOCK_DIAG(D, m) returns a block diagonal matrix D_ consisting of
%   m copies of the matrix D along its diagonal. All off-diagonal blocks are zeros.
%
%   Inputs:
%       D - A square matrix of size n-by-n to be repeated.
%       m - The number of times the matrix D is to be repeated along the diagonal.
%
%   Output:
%       D_ - A block diagonal matrix of size (m*n)-by-(m*n) with m copies of D.
%
%   Example:
%       D = [1 2; 3 4];
%       m = 3;
%       D_ = block_diag(D, m);
%       % D_ is a 6x6 matrix with three 2x2 D matrices on the diagonal.

n = size(D,1);
D_ = zeros(m*n,m*n);

for i = 1:m
    D_(1+(i-1)*n:(i*n),1+(i-1)*n:(i*n)) = D;
end    

end