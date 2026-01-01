function [M, N] = MPC_MN_state(A, B, p)

n = size(A,1);
r = size(B,2);

M = zeros(n*p, n); %TODO
N = zeros(n*p, r*p); %TODO

for i = 1:p
    %TODO 
    M((i-1)*n+1:i*n, :) = A^i;
end

for i = 1:p  % -1
    %TODO 
    for j = 1:i
        N((i-1)*n+1:i*n, (j-1)*r+1:j*r) = A^(i-j) * B;
    end
end

end