clear
addpath("..\mobile_robot");       
%------------------------------
% PARAMETERS FOR OPTIMAL CONTROL
%------------------------------
q = 15;       % Weight on position error in cost function
%q = 10;
rv = 1;       % Weight on linear velocity magnitude in cost function
%romega = 2;   % Weight on angular velocity magnitude in cost function
romega = 5;

%------------------------------
% INITIAL ROBOT STATE
%------------------------------
x0 = 0;       % Initial x position
y0 = 0;       % Initial y position
theta0 = pi;  % Initial orientation (radians), facing backwards

tf = 10;      % Final time horizon for each trajectory segment (seconds)

%------------------------------
% WAYPOINTS TO FOLLOW (2D points)
%------------------------------
X_ref = [2, 1;
         1, -3;
        -2, -2;
        -4,  2;
         1,  3;
         2,  1];

close all   % Close all figures before starting

%------------------------------
% LOOP OVER EACH WAYPOINT
%------------------------------
for i = 1:size(X_ref,1)
    
    % Extract current target waypoint coordinates
    x_ref = X_ref(i,1);
    y_ref = X_ref(i,2);
    
    % Shift initial coordinates so that target is at origin,
    % simplifying boundary conditions for solver
    x0_shifted = x0 - x_ref;
    y0_shifted = y0 - y_ref;
    
    %------------------------------
    % BVP SOLVER OPTIONS
    %------------------------------
    % Set relative and absolute tolerances for the solver
    %opts = bvpset('RelTol', 5e-4, 'AbsTol', 5e-4, 'NMax', 250000);
    opts = bvpset('RelTol', 1e-3, 'AbsTol', 1e-3, 'NMax', 5000);
    
    % Initialize mesh points for solver and initial guess for solution
    solinit = bvpinit(linspace(0, tf, 15), @(t) mobile_robot_initial_guess(t, x0_shifted, y0_shifted, theta0, tf));
    
    % Define the ODE system as a function handle with current parameters
    ode = @(t, y) mobile_robot_odefun(t, y, q, rv, romega);
    % Solve the boundary value problem (BVP)
    sol = bvp4c(ode, @(ya,yb) mobile_robot_bcfun(ya, yb, x0_shifted, y0_shifted, theta0), solinit, opts);
    
    % Extract solution time points and corresponding solution variables
    t = sol.x;        % Time vector
    Y = sol.y;        % Solution matrix: rows correspond to variables
    
    % Extract state variables from solution
    x = Y(1, :);      % x position over time
    y = Y(2, :);      % y position over time
    theta = Y(3, :);  % orientation over time
    
    % Extract costate variables (Lagrange multipliers from PMP)
    lambda_x = Y(4, :);
    lambda_y = Y(5, :);
    lambda_theta = Y(6, :);
    
    % Compute optimal control inputs (linear and angular velocities)
    [v, omega] = mobile_robot_get_optimal_u(theta, rv, romega, lambda_x, lambda_y, lambda_theta);
    
    %------------------------------
    % PLOT RESULTS
    %------------------------------
    figure(1);
    
    % Plot the trajectory adjusted back to global coordinates
    plot(x + x_ref, y + y_ref, 'b', 'LineWidth', 2);
    hold on;
    
    
    % Mark final position reached after this segment
    plot(x(end) + x_ref, y(end) + y_ref, 'or', 'LineWidth', 2);
    
    % Mark the reference waypoint as a magenta plus
    plot(X_ref(i,1), X_ref(i,2), '+m', 'LineWidth', 1, 'MarkerSize', 15);
    
    % Update initial conditions for next waypoint segment to current endpoint
    x0 = x(end) + x_ref;
    y0 = y(end) + y_ref;
    theta0 = theta(end);
end

% Add grid, axis equal, labels, and legend to trajectory plot
grid on
axis equal
xlabel('x');
ylabel('y');
legend('Trajectory', 'End Point', 'Waypoint');

%------------------------------
% PLOT CONTROL INPUTS OVER TIME
%------------------------------
figure;

% Linear velocity profile
subplot(2,1,1);
plot(t, v, '-k', 'LineWidth', 2);
xlabel('t [s]');
ylabel('v(t) [m/s]');
grid on;

% Angular velocity profile
subplot(2,1,2);
plot(t, omega, '-k', 'LineWidth', 2);
xlabel('t [s]');
ylabel('\omega(t) [rad/s]');
grid on;

%------------------------------
% PLOT COSTATE VARIABLES OVER TIME
%------------------------------
figure;

% lambda_x (costate for x position)
subplot(3,1,1);
plot(t, lambda_x, '-k', 'LineWidth', 2);
xlabel('t [s]');
ylabel('\lambda_x(t)');
grid on;

% lambda_y (costate for y position)
subplot(3,1,2);
plot(t, lambda_y, '-k', 'LineWidth', 2);
xlabel('t [s]');
ylabel('\lambda_y(t)');
grid on;

% lambda_theta (costate for orientation)
subplot(3,1,3);
plot(t, lambda_theta, '-k', 'LineWidth', 2);
xlabel('t [s]');
ylabel('\lambda_\theta(t)');
grid on;



