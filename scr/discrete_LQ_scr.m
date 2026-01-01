%% Design of LQ controller (discrete-time)
clear all

% System matrices
A = [0, 1;
     0.45, 0.2];

B = [2, 0.5;
     0, 1];

% Weighting matrices
Q = [1, 0;
     0, 5];       % State cost

R = [5, 0;
     0, 10];      % Input cost

% Solve the Discrete Algebraic Riccati Equation
[P,~,K]= dare(A, B, Q, R);  %TODO

%% Regulation to zero

x_ = [];   % Store state trajectory
u_ = [];   % Store control inputs

x = [5; 15];   % Initial state

J = x' * P * x  % Initial cost (from value function)
J = 0;          % Reset cost accumulator

N = 15;         % Number of simulation steps

for i = 1:N
    u = -K * x;  %TODO;    % Compute control input
    J = J + x' * Q * x + u' * R * u;  %TODO;    % Accumulate cost

    x_ = [x_, x];    % Store state
    u_ = [u_, u];    % Store input

    x = A * x + B * u;  % Update system state
end


%% Extended model (Integrator Augmentation)

C = eye(2);   % Output matrix (identity — full-state measurement)

A = [0, 1;
     0.45, 0.2];

% Augmented A matrix 
A_tilde = [A, zeros(2,2);
           -C, eye(2)];  %TODO;

B = [2, 0.5;
     0, 1];

% Augmented B matrix 
B_tilde = [B;
           zeros(2,2)];  %TODO;

% Weighting matrices for augmented system
Q  = [1, 0;
      0, 0.5];   % State cost

Qz = [1, 0;
      0, 0.5];   % Integral state cost

R = [5, 0;
     0, 2];      % Input cost

% Augmented cost matrix
Q_tilde = blkdiag(Q, Qz);  %TODO;

% Solve DARE for augmented system
% [P, ~, K] = dare(A_tilde, B_tilde, Q_tilde, R); %TODO
[P,~,K_tilde] = dare(A_tilde, B_tilde, Q_tilde, R);

% Separate gains for state and integral part
Kx = K_tilde(:, 1:2);  %TODO
Kz = K_tilde(:, 3:4);  %TODO

% Desired output reference
y_r = [10; 15];

% Constant disturbance (model mismatch or external input)
d = [5; 3];

x_ = [];
u_ = [];
z_ = [];

x = [0; 0];                  % Initial state
z = zeros(size(y_r));        % Initial integrator state

N = 15;

for i = 1:N
    u = -Kx * x - Kz * z;  %TODO ;     % Control law with integral action

    x_ = [x_, x];            % Store state
    u_ = [u_, u];            % Store input
    z_ = [z_, z];            % Store integral state

    z = z + (y_r - C * x);  %TODO;   % Integrate tracking error
    x = A * x + B * u + d;   % Update system state (with disturbance)
end

%% Plotting

style = '-k';  % Black line style

% Plot state x1(k)
subplot(2,2,1)
stairs(0:(N-1), x_(1,:), style, 'LineWidth', 1.5)
xlabel('k');
ylabel('x_1(k)');
xlim([0, N-1]);
grid on
hold on

% Plot state x2(k)
subplot(2,2,2)
stairs(0:(N-1), x_(2,:), style, 'LineWidth', 1.5)
xlabel('k');
ylabel('x_2(k)');
xlim([0, N-1]);
grid on
hold on

% Plot input u1(k)
subplot(2,2,3)
stairs(0:(N-1), u_(1,:), style, 'LineWidth', 1.5)
xlabel('k');
ylabel('u_1(k)');
xlim([0, N-1]);
grid on
hold on

% Plot input u2(k)
subplot(2,2,4)
stairs(0:(N-1), u_(2,:), style, 'LineWidth', 1.5)
xlabel('k');
ylabel('u_2(k)');
xlim([0, N-1]);
grid on
hold on

set(gcf, 'position', [0, 200, 700, 350]);

%%
% Plot integral states
figure

subplot(1,2,1)
stairs(0:(N-1), z_(1,:), style, 'LineWidth', 1.5)
xlabel('k');
ylabel('z_1(k)');
xlim([0, N-1]);
grid on
hold on

subplot(1,2,2)
stairs(0:(N-1), z_(2,:), style, 'LineWidth', 1.5)
xlabel('k');
ylabel('z_2(k)');
xlim([0, N-1]);
grid on
hold on

set(gcf, 'position', [0, 200, 700, 170]);