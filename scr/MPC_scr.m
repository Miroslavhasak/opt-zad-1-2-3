% State-space MPC without constraints

addpath("..\MPC")  % Add path to MPC helper functions

% System dynamics
A = [0.2, 0.8;
    -0.4, 0.5];
B = [2, 0.5;
     0, 1];

n = size(A,1);  % Number of states
r = size(B,2);  % Number of inputs

p = 10;  % Prediction horizon

% Get prediction matrices M and N
%[M,N]= %TODO

M = zeros(n*p, n);
N = zeros(n*p, r*p);

for i = 1:p
    M((i-1)*n+1:i*n, :) = A^i;
    for j = 1:i
        N((i-1)*n+1:i*n, (j-1)*r+1:j*r) = A^(i-j)*B;
    end
end

% Cost function weighting
Q = eye(n);                % State weighting
Q_ = block_diag(Q, p);     % Block diagonal over horizon

R = diag([5,5]);           % Input weighting
R_ = block_diag(R, p);     % Block diagonal over horizon

% Compute MPC gain for unconstrained optimization
K = (N' * Q_ * N + R_) \ (N' * Q_ * M); %TODO

x = [0;10];  % Initial state

n_steps = 15;  % Number of simulation steps

x_ = [];  % State history
u_ = [];  % Input history

for i = 1:n_steps
    
    u = -K * x; %TODO
    u = u(1:r);

    x_ = [x_, x];
    u_ = [u_, u];

    x = A*x + B*u;  % State update
end

%%
style = '-k';

subplot(2,2,1)
stairs(0:(n_steps-1), x_(1,:), style, 'LineWidth', 1.5)
xlabel('k'); ylabel('x_1(k)'); xlim([0, n_steps-1]); grid on

subplot(2,2,2)
stairs(0:(n_steps-1), x_(2,:), style, 'LineWidth', 1.5)
xlabel('k'); ylabel('x_2(k)'); xlim([0, n_steps-1]); grid on

subplot(2,2,3)
stairs(0:(n_steps-1), u_(1,:), style, 'LineWidth', 1.5)
xlabel('k'); ylabel('u_1(k)'); xlim([0, n_steps-1]); grid on

subplot(2,2,4)
stairs(0:(n_steps-1), u_(2,:), style, 'LineWidth', 1.5)
xlabel('k'); ylabel('u_2(k)'); xlim([0, n_steps-1]); grid on

set(gcf, 'position', [0, 200, 700, 350]);