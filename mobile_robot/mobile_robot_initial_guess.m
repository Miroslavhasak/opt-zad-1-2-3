function X_lambda = mobile_robot_initial_guess(t, x0, y0, theta0, tf)
    % Provides initial guess for state and costate trajectories
    % Linear interpolation of states from initial values to zero
    % Costates guess proportionally to states with zero terminal
    
    x = x0 * (1 - t/tf);
    y = y0 * (1 - t/tf);
    theta = theta0 * (1 - t/tf);
    
    % guess for costates scaled similarly (arbitrary scaling factor 1/5)
    lambda_x = (1/5) * x0 * (1 - t/tf);
    lambda_y = (1/5) * y0 * (1 - t/tf);
    
    lambda_theta = 0; % start guess for lambda_theta
    
    X_lambda = [x; y; theta; lambda_x; lambda_y; lambda_theta];
end