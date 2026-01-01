clear; clc; close all;   % Clear workspace and variables

delta_t = 0.9;   % Discrete time step (sampling interval)

% State transition matrix A (motion model: constant velocity in 2D)
A = [1 0 delta_t 0;
     0 1 0 delta_t;
     0 0 1 0;
     0 0 0 1];  %TODO

% Input matrix B (no control input here, all zeros)
B = [0;
     0;
     0;
     0];

% Measurement matrix C
% Defines which states are observed: velocity_x, velocity_y, position_x, position_y
C = [0 0 1 0;
     0 0 0 1;
     1 0 0 0;
     0 1 0 0];  %TODO

% Measurement noise covariance matrix R
R = diag([0.05,0.05,5,5]);

% Process noise covariance matrix Q
Q = diag([0,0,0.3,0.3]);

% Noise generation using matrix square roots
chol_Q = sqrt(Q);   % For process noise
chol_R = chol(R);   % For measurement noise

n_steps = 25;       % Number of simulation steps

% Dimensions
m = size(C,1);   % Number of measurements
r = size(B,2);   % Number of inputs
n = size(A,1);   % Number of states

% Data storage for plotting and analysis
x_ = [];      % True states
x_hat_ = [];  % Estimated states
y_ = [];      % Noisy measurements
y_hat_ = [];  % Estimated measurements

u = zeros(n_steps,1);   % Control input (all zeros in this case)

%x=[0;3]   % (alternative initial condition, commented out)

x = [0;0;0;0];   % Initial true state (position and velocity)

x_hat = x;       % Initial estimated state
P = zeros(n,n);  % Initial covariance (no prior uncertainty)

% Propagate the true system for the first step
x = A*x + B*u(1);

% ----------- Main loop ------------
for i=1:n_steps
    
    % Generate measurement noise
    v = chol_R*randn(m,1);
    % Measurement equation (y = Cx + noise)
    y = C*x + v;

    % ----- Kalman filter equations 
    x_hat = A*x_hat + B*u(i);  %TODO;   % Prediction of the state
    P     = A*P*A' + Q; %TODO;   % Prediction of the covariance

    K     = P*C' / (C*P*C' + R); %TODO;   % Kalman gain
        
    x_hat = x_hat + K*(y - C*x_hat); %TODO;   % Update of the state estimate
    y_hat = C*x_hat; %TODO;   % Estimated measurement
    P     = (eye(n) - K*C)*P; %TODO;   % Update of the covariance
    % ----------------------------------------------------------

    % Store results
    x_ = [x_,x];
    x_hat_ = [x_hat_,x_hat];
    y_ = [y_,y];
    y_hat_ = [y_hat_,y_hat];

    % Propagate the true system with process noise
    if i < n_steps
        w = chol_Q*randn(n,1);     % Process noise
        x = A*x + B*u(i+1) + w;    % State update
    end
end

% Estimation error over time
e_ = x_ - x_hat_;

% Covariance of estimation error
cov(e_');

close all

% ----------- Visualization ------------
figure

% Plot true and estimated positions
plot(x_(1,:),x_(2,:),'ok');       % True positions (black circles)
hold on
plot(x_hat_(1,:),x_hat_(2,:),'or'); % Estimated positions (red circles)

% Plot velocity vectors (arrows)
quiver(x_(1,:),x_(2,:),x_(3,:),x_(4,:),0.3)           % True velocities
quiver(x_hat_(1,:),x_hat_(2,:),x_hat_(3,:),x_hat_(4,:),0.3) % Estimated velocities
grid on

axis equal

legend('skutočná poloha','odhadovaná poloha','skutočná rýchlosť','odhadovaná rýchlosť')

set(gcf,'position',[400,0,650,550]);   % Figure size/position

xlabel('X')
ylabel('Y')