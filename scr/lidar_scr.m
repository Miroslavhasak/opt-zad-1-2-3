addpath("..\ICP");               % Add path to ICP (Iterative Closest Point) functions
addpath("..\geometry");          % Add path to geometry-related utilities
addpath("..\optim_alg\");        % Add path to optimization algorithms

%% Lidar localization
clear                             % Clear workspace
global v_r omega                  % Global variables for linear and angular velocity
close all                         % Close all figures

% Initial state [x; y; theta]
t=[90;60];
theta=pi/3;
x=[t;theta];
delta_t=0.1;                      % Time step

[map] = gen_map();                % Generate occupancy grid map

% Extract obstacle coordinates from map
[obs_y,obs_x]=find(map);
obs=[obs_x,obs_y];
pB=obs;

KDtree = KDTreeSearcher(pB);

% Lidar parameters
sigma=0.5;                        % Noise standard deviation
n_points=50;                      % Number of lidar beams
range=40;                         % Max lidar range

% Robot motion
v_r=0; omega=0;                   % Initial velocities

% Covariances
R=diag([pi/15,1/5,1/5]);           % Measurement noise covariance
Q=diag([0.015,0.015,0.001])*2;      % Process noise covariance
chol_Q=sqrt(Q);                   % Square root of Q for noise sampling

P=zeros(3,3);                     % Initial covariance matrix
x_hat=x;                          % Initial state estimate

% Setup figure and keyboard controls
fig = figure;
set(fig, 'WindowKeyPressFcn', @keyCallback);
set(fig, 'WindowKeyReleaseFcn', @onKeyRelease);

% === Main loop ===
while 1
    clf
    plot_map(map);               % Display map

    % Predict next position
    r=round(x(2)+v_r*delta_t*sin(x(3)));
    c=round(x(1)+v_r*delta_t*cos(x(3)));

    % Check for obstacle collision (robot footprint)
    no_hit=sum(sum(map((r-3):(r+3),(c-3):(c+3))))==0;

    % Plot true position before motion
    if (~no_hit)
         plot(x(1), x(2), 'or', 'MarkerSize',15,'LineWidth',1.5);
    end

    % Simulate motion with noise if no obstacle
    x=[x(1)+v_r*delta_t*cos(x(3))*no_hit;
       x(2)+v_r*delta_t*sin(x(3))*no_hit;
       x(3)+delta_t*omega]+chol_Q*randn(3,1)*(omega>0||v_r>0)*no_hit;

    % Plot new position
    plot(x(1), x(2), 'om', 'MarkerSize',10,'LineWidth',1.5);
    quiver(x(1), x(2), cos(x(3)),sin(x(3)),10,'m','LineWidth',1.5);

    % Simulate Lidar scan
    [pA,pB_] = simulateLidar(map, x(1:2), x(3), range,n_points,1/2,sigma);
    scatter(pB_(:,1), pB_(:,2), 10, 'g', 'filled');

      % Prediction step
    A=[1,0,-v_r*delta_t*sin(x_hat(3));
       0,1,+v_r*delta_t*cos(x_hat(3));
       0,0,1];
    P=A*P*A'+Q;

    x_hat=[x_hat(1)+v_r*delta_t*cos(x_hat(3));
           x_hat(2)+v_r*delta_t*sin(x_hat(3));
           x_hat(3)+delta_t*omega];


    % Estimate relative transformation using ICP
    [theta_hat,t_hat] = ICP_estimate_2D(pA,pB,x_hat(3),x_hat(1:2),5e-1,0.5,KDtree);
    y=[theta_hat;t_hat];

    % Update step
    C=[0,0,1;
       1,0,0;
       0,1,0];
    y_hat=C*x_hat;
    K=P*C'*inv(C*P*C'+R);
    x_hat=x_hat+K*(y-y_hat);
    P=P-K*C*P;

    % Plot estimates
    plot(t_hat(1), t_hat(2), 'ob', 'MarkerSize',10,'LineWidth',1.5);
    quiver(t_hat(1), t_hat(2), cos(theta_hat),sin(theta_hat),10,'b','LineWidth',1.5);
    plot(x_hat(1), x_hat(2), 'og', 'MarkerSize',10,'LineWidth',1.5);
    quiver(x_hat(1), x_hat(2), cos(x_hat(3)),sin(x_hat(3)),10,'g','LineWidth',1.5);

    pause(delta_t)
end


%% Handle keyboard key press
function keyCallback(src, event)
    global v_r omega
    switch event.Key
        case 'w'
            v_r=25;              % Forward
        case 's'
            v_r=-25;             % Backward
        case 'a'
            omega=2;             % Turn left
        case 'd'
            omega=-2;            % Turn right
    end    
end

%% Handle keyboard key release
function onKeyRelease(src, event)
    global v_r omega
    v_r=0; omega=0;
    switch event.Key
        case 'w'
            v_r=0;
        case 's'
            v_r=0;
        case 'a'
            omega=0;
        case 'd'
            omega=0;
    end    
end