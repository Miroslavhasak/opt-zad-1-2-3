clc; clear; close all;
addpath("..\drone");
addpath("..\MPC");
%% --- Parameters ---
m = 0.5;                            %weight of drone kg
l = 0.1750;                         %length of arm
J = diag([0.0023 0.0023 0.004]);    %inertial matrix
g = 9.81;                           %magnitude of gravity vector
kt = 1.0;                           %thrust coeff
km = 0.0245;                        %torque coeff
h = 0.05;                           %20 Hz - Freq of controller

n = 12;                             %Number of states
r = 4;                              %Number of inputs

Tfinal = 5; %s
n_steps = round(Tfinal/h)+1;  % Number of simulation steps
thist = 0:h:Tfinal;
utils = quad_utils(m, J, g, kt, km, l, h);

%% --- Hover equilibrium ---
uhover = (m*g/4)*ones(4,1); % hover thrust per prop [N]
r0=[0;0;1.5]; q0=[1;0;0;0]; v0=zeros(3,1); w0=zeros(3,1);
x0=[r0;q0;v0;w0]; % Initial state

%% --- Linearization (numerical Jacobians) ---
A = utils.fjacobian(@(x) utils.quad_dynamics_rk4(x,uhover),x0);
B = utils.fjacobian(@(u) utils.quad_dynamics_rk4(x0,u),uhover);

%% --- Reduced linear model ---
A_tilde = utils.E_fn(q0)'*A*utils.E_fn(q0);
B_tilde = utils.E_fn(q0)'*B;

%% --- MPC setup ---
p = 25; % Prediction horizon
[M,N] = MPC_MN_state(A_tilde, B_tilde, p); %TODO
Q = diag([ones(3,1)*5; ones(3,1)*5; ones(3,1)*1; ones(3,1)*1]);                % State weighting
Q_ = block_diag(Q, p);     % Block diagonal over horizon
R = 0.1*eye(r);            % Input weighting
R_ = block_diag(R, p);     % Block diagonal over horizon

H = N' * Q_ * N + R_; %TODO

H = (H + H')/2;

% Input bounds (thrust [N])
u_min = [0;0;0;0];   % lower bound
u_max = [2;2;2;2];   % upper bound

% Delta-U bounds
du_min = u_min - uhover;
du_max = u_max - uhover;

% Expand over horizon p
delta_U_min = repmat(du_min, p, 1);
delta_U_max = repmat(du_max, p, 1);

% Inequality matrices for input bounds
A_ineq_u = [ eye(p*r); %TODO;   % ΔU ≤ ΔU_max
             -eye(p*r) ]; %TODO];  % -ΔU ≤ -ΔU_min

b_ineq_u = [ delta_U_max; %TODO;
            -delta_U_min ]; %TODO ];

opts = optimoptions('quadprog','Display','off');

%% --- Simulation ---
xhist=zeros(n+1,n_steps); uhist=zeros(r,n_steps);
xhist(:,1)=[r0+randn(size(r0)); utils.L_fn(q0)*utils.rptoq([0;1;0]); v0; w0];
xhist(3,1)=max(xhist(3,1),0);

for k = 1:n_steps-1
    % --- Inline MPC controller ---
    x = xhist(:,k);
    
    % Reference states
    r0 = x0(1:3);
    v0 = x0(8:10);
    w0 = x0(11:13);
    q0 = x0(4:7);
    
    % Compute error in quaternion using qtorp and L_fn
    q  = x(4:7);
    phi = utils.qtorp(utils.L_fn(q0)' * q);
    
    % State deviation
    dx_tilde = [x(1:3)-r0; phi; x(8:10)-v0; x(11:13)-w0];
    
    % QP linear term
    f = N' * Q_ * (M * dx_tilde); %TODO
   
    % Solve QP
    % [dU_opt] = quadprog(); %TODO
    [dU_opt] = quadprog(H, f, A_ineq_u, b_ineq_u, [], [], [], [], [], opts);

    % Compute control input
    du0 = dU_opt(1:4); %TODO
    u = uhover + du0; %TODO
    
    % Store control
    uhist(:,k) = u;
    
    % Propagate dynamics
    xhist(:,k+1) = utils.quad_dynamics_rk4(xhist(:,k), u);
end

uhist(:,n_steps)=uhist(:,n_steps-1);

quad_viz(xhist,n_steps,h);


roll  = zeros(1,n_steps); pitch = zeros(1,n_steps); yaw = zeros(1,n_steps);
for k=1:n_steps
    [roll(k), pitch(k), yaw(k)] = utils.quat2eul(xhist(4:7,k));
end
figure;
subplot(4,1,1);
plot(thist, xhist(1:3,:)'); ylabel('Position [m]'); legend('x','y','z'); grid on;
subplot(4,1,2);
plot(thist, xhist(8:10,:)'); ylabel('Velocity [m/s]'); legend('vx','vy','vz'); grid on;
subplot(4,1,3);
plot(thist, [roll; pitch; yaw]'*180/pi); ylabel('Euler angles [deg]'); legend('roll','pitch','yaw'); grid on;
subplot(4,1,4);
plot(thist, uhist'); ylabel('Propeller thrust [N]'); xlabel('Time [s]'); legend('u1','u2','u3','u4'); grid on;



