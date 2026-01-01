addpath("..\ICP");          % Add path to ICP (Iterative Closest Point) related functions
addpath("..\geometry");     % Add path to geometry-related functions
addpath("..\optim_alg\");   % Add path to optimization algorithms
clear                       % Clear all variables from workspace
close all                   % Close all figures

load('teapot.mat');

% Define rotation angles around x, y, z axes
theta = [pi/5; pi/3; pi/7];

% Construct rotation matrix using ZYX Euler angles
R = rotZ(theta(3)) * rotY(theta(2)) * rotX(theta(1));

% Define translation vector
t = [4.5; -2; 3];

% Initialize figure
figure
xlabel('x')
ylabel('y')
zlabel('z')
grid on
hold on
axis equal

% Draw global coordinate frame (red-x, green-y, blue-z)
quiver3(0,0,0,1,0,0,1.0,'r');
quiver3(0,0,0,0,1,0,1.0,'g');
quiver3(0,0,0,0,0,1,1.0,'b');

% Draw transformed coordinate frame using R and t
e = R*[1;0;0] + t;
quiver3(t(1), t(2), t(3), e(1)-t(1), e(2)-t(2), e(3)-t(3), 1.0, 'r');  % X-axis

e = R*[0;1;0] + t;
quiver3(t(1), t(2), t(3), e(1)-t(1), e(2)-t(2), e(3)-t(3), 1.0, 'g');  % Y-axis

e = R*[0;0;1] + t;
quiver3(t(1), t(2), t(3), e(1)-t(1), e(2)-t(2), e(3)-t(3), 1.0, 'b');  % Z-axis

% Plot original point set pA in red
%plot3(pA(:,1), pA(:,2), pA(:,3), '+r')
pcshow(pA,[0 1 1]);
hold on

N = size(pA, 1);                % Number of points
pB = zeros(size(pA));           % Initialize transformed point set

% Transform point set pA using R and t, add small noise
for i = 1:N
    pB(i,:) = R * pA(i,:)' + t + eye(3)/50 * randn(3,1);  
end

% Plot transformed point set pB in blue
%plot3(pB(:,1), pB(:,2), pB(:,3), '+b')
pcshow(pB,[1 1 0]);

% Set 3D view angle
view(-140, 30)

% Initial guess for transformation (rotation angles and translation)
theta_hat = [0; 0; 0];
t_hat = [0; 0; 0];

% Tolerance for optimization
tol = 1e-1;

% Estimate transformation parameters (rotation, translation) from pA to pB
[theta_hat, t_hat] = transform_estimate(pA, pB, theta_hat, t_hat, tol);
% Reconstruct estimated rotation matrix from estimated Euler angles
R_hat = rotZ(theta_hat(3)) * rotY(theta_hat(2)) * rotX(theta_hat(1));

pB_hat = zeros(size(pB));  % Initialize reprojected point set

% Apply estimated transformation to pA and generate pB_hat
for i = 1:N
    pB_hat(i,:) = R_hat * pA(i,:)' + t_hat;
end

% Plot estimated transformed point set in green
%plot3(pB_hat(:,1), pB_hat(:,2), pB_hat(:,3), 'og')
pcshow(pB_hat,[1 0 1]);

