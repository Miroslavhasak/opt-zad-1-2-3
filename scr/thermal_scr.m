addpath("..\lab_models\");  % add paths to system models
addpath("..\MPC\");         % add paths to MPC-related functions
addpath("..\misc\");

%% ===========================
%   MPC (Output-based, Quadratic Programming)
%   Thermal system control example
% ===========================
close all

n_steps=100;  % number of simulation/control steps

% Options for MATLAB quadprog solver
options = optimoptions('quadprog', ...
                       'ConstraintTolerance', 1e-5, ...
                       'OptimalityTolerance',1e-5, ...
                       'Display', 'off');

Ts=10;        % sampling time (seconds)
y_ref=50;     % reference output value
p=20;         % prediction horizon

sim=1;        % simulation flag: 1 = MATLAB simulation, 0 = real hardware
theta_hat=[1.1896 0.0248 0.0008 0.0875];  % identified thermal system parameters

% --- Continuous and discrete state-space model ---
[Ac,Bc,Cc] = create_thermal_model(theta_hat);  % continuous-time model
[A,B,C] = ss_discretize(Ac,Bc,Cc,Ts);          % discretize with sampling time Ts

% Dimensions
m=size(C,1);   % number of outputs
r=size(B,2);   % number of inputs
n=size(A,1);   % number of states

% --- Augmented system for integral action ---
A_tilde=%TODO
B_tilde=%TODO
C_tilde=%TODO  

% --- MPC prediction matrices ---
[M,N] = MPC_MN_output(A_tilde,B_tilde,C_tilde,p);  % build prediction matrices
[Gamma] = MPC_Gamma(r,p);                           % map control increments to predictions

% --- Weighting matrices for MPC cost ---
Q_=%TODO            % output tracking weights
R_=%TODO      % input change penalty

% --- Quadratic cost Hessian ---
H=%TODO
H=(H+H')/2;   % ensure symmetry for quadprog

% --- Reference trajectory over horizon ---
Y_ref=repmat(y_ref,p,1);

% --- Input constraints ---
u_underbar=[0];          % lower bound on control
u_overbar=[100];         % upper bound on control
U_underbar=repmat(u_underbar,p,1);
U_overbar=repmat(u_overbar,p,1);

% --- Constraint matrices for quadprog ---
A_con=%TODO;          % inequality matrix for delta_u constraints

% --- State estimation (Kalman filter) initialization ---
x_hat=[0;0;0;0];          % augmented state estimate (n+1 states)
P=zeros(n+1);             % covariance matrix
 

R=0.1250^2;  % measurement noise covariance
Q=diag([0.1;0.1;0.1;1]); % process noise covariance

% --- Logging arrays ---
x_hat_=[]; y_hat_=[]; y_=[]; u_=[];

% Initial control
u=zeros(r,1);
delta_U=zeros(p*r,1);

% --- Hardware interface / simulation ---
if ~sim
    COM_port=9;
    baudrate=250000;
    data_stream_ptr=data_stream_start_mex(COM_port,baudrate);
    [~,~]=data_stream_read_mex(data_stream_ptr,1,1);
else
    x=[25;25;25];  % initial temperature for simulation
end

%% ===========================
%   Simulation / Control loop
% ===========================
for i=1:n_steps
    
    % --- Measurement acquisition ---
    if ~sim
        [y,t]=data_stream_read_mex(data_stream_ptr,1,1);
        y=double(y);  % convert to double precision
    end
   
    % --- State prediction ---
    x_hat=%TODO

    % --- Simulated output (for sim mode) ---
    if sim
        y=C*x+sqrt(R)*randn();
    end
    y   % display output for debugging

    % --- Kalman filter update ---
    P=%TODO                        % covariance prediction
    K = %TODO           % Kalman gain
    e = %TODO                           % innovation
    x_hat = %TODO                            % state update
    y_hat = %TODO                            % estimated output
    P = %TODO;  % covariance update

    % --- MPC optimization ---
    b = %TODO   % linear term in QP
    b_con = %TODO                     % inequality bounds
    
    delta_U = quadprog(H,b,A_con,b_con,[],[],[],[],[],options);  % solve QP

    delta_u = delta_U(1:r,:);  % only first input increment applied
    u = u + delta_u;            % update control input
    u  % display control for debugging

    % --- Plant update ---
    if ~sim
        data_stream_write_mex(data_stream_ptr,1,single(u));  % send control to hardware
    else
        x = A*x + B*(u + 25/theta_hat(1)) + chol(Q(1:3,1:3))*randn(3,1);                   % simulate thermal system
    end

    % --- Log data for plotting ---
    x_hat_ = [x_hat_, x_hat];
    y_hat_ = [y_hat_, y_hat];
    y_     = [y_, y];
    u_     = [u_, u];
end

%% --- Shutdown hardware stream ---
if ~sim
    data_stream_write_mex(data_stream_ptr,1,single(0));
    data_stream_end_mex(data_stream_ptr);
end

%% ===========================
%   Plotting results
% ===========================
figure
style='-k';

subplot(4,1,1)
stairs(0:(n_steps-1), x_hat_(1,:), style, 'LineWidth', 1.5)
xlabel('k'); ylabel('x_1(k)'); grid on; xlim([0,n_steps-1]);

subplot(4,1,2)
stairs(0:(n_steps-1), x_hat_(2,:), style, 'LineWidth', 1.5)
xlabel('k'); ylabel('x_2(k)'); grid on; xlim([0,n_steps-1]);

subplot(4,1,3)
stairs(0:(n_steps-1), y_hat_, '-g', 'LineWidth', 1.5)
hold on
stairs(0:(n_steps-1), y_, '-b', 'LineWidth', 1.5)
stairs(0:(n_steps-1), x_hat_(3,:), style, 'LineWidth', 1.5)
stairs(0:(n_steps-1), ones(1,n_steps)*y_ref, '--r', 'LineWidth', 1.5)
xlabel('k'); ylabel('x_3(k)'); grid on; xlim([0,n_steps-1]);

subplot(4,1,4)
stairs(0:(n_steps-1), u_(:), style, 'LineWidth', 1.5)
hold on
stairs(0:(n_steps-1), ones(1,n_steps)*u_overbar(1), '--g', 'LineWidth', 1.5)
stairs(0:(n_steps-1), ones(1,n_steps)*u_underbar(1), '--g', 'LineWidth', 1.5)
xlabel('k'); ylabel('u(k)'); grid on; xlim([0,n_steps-1]);

set(gcf,'position',[0,200,650,400]);