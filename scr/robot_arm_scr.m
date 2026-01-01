clc; clear; close all;
addpath("..\geometry");
addpath("..\robot_arm");
addpath("..\misc");
addpath("..\lab_models\");

%% 3 DOF Robot Initialization

% Link lengths
L1 = 0.2; L2 = 0.5; L3 = 0.5;

% Translation vectors for each joint
T1 = [0; 0; L1];
T2 = [0; 0; L2];
T3 = [0; 0; L3];

% Stack translation vectors into a matrix
T = [T1, T2, T3];

% Rotation functions for each joint
R = {@rotZ, @rotX, @rotX};
% Derivatives of rotation functions (for Jacobians)
dR = {@drotZ, @drotX, @drotX};


%% 6 DOF Robot Initialization

% Define link lengths
T1 = [0; 0; 0.25];
T2 = [0; 0; 0.75]; 
T3 = [0; 0; 1.0];
T4 = [0; 0; 0.5];
T5 = [0; 0; 0.4];
T6 = [0; 0; 0.3];

% Combine all translation vectors
T = [T1, T2, T3, T4, T5, T6];

R = {@rotZ, @rotX, @rotX, @rotZ, @rotY, @rotZ};
dR = {@drotZ, @drotX, @drotX, @drotZ, @drotY, @drotZ};

%% Inverse Kinematics without Orientation

% Desired position of end effector
pr = [0.5; 0.25; 0.4];

% Initial guess for joint angles
theta = ones(3,1) * pi/4;

% Solve inverse kinematics including orientation constraint
theta = solve_IK(pr, theta, R, dR, T, 1e-4);

% Transform last frame position to base frame
p0 = transform_pi_to_p0(theta, [0;0;0], 3, R, T);

% Plot the resulting robot configuration
plot_robot(theta, R, T)
plot3(pr(1),pr(2),pr(3),'or','LineWidth',2,'MarkerSize',11)

%% Inverse Kinematics with Orientation

% Desired position of end effector
pr = [1.0; 0.5; 2.0];

% Desired orientation using rotation matrices
Rr = rotZ(pi/3) * rotY(pi/3) * rotX(pi/3);

% Initial guess for joint angles

theta =zeros(6,1);

% Solve inverse kinematics including orienňtation constraint
theta = solve_IK_orientation(pr, Rr, theta, R, dR, T, 1e-6);

% Transform last frame position to base frame
p0 = transform_pi_to_p0(theta, [0;0;0], 6, R, T);

% Plot the resulting robot configuration
plot_robot(theta, R, T)

plot3(pr(1),pr(2),pr(3),'or','LineWidth',2,'MarkerSize',11)

e = Rr * [1;0;0];
quiver3(pr(1), pr(2), pr(3), e(1), e(2), e(3), 0.2, 'r','LineWidth',2);
e = Rr * [0;1;0];
quiver3(pr(1), pr(2), pr(3), e(1), e(2), e(3), 0.2, 'g','LineWidth',2);
e = Rr * [0;0;1];
quiver3(pr(1), pr(2), pr(3), e(1), e(2), e(3), 0.2, 'b','LineWidth',2);



