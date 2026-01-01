function dX_lambda_dt = mobile_robot_odefun(t, x_vec, q, rv, romega)
    % ODEFUN Defines the system and costate dynamics based on PMP
    
    % Unpack state variables
    x_pos = x_vec(1);
    y_pos = x_vec(2);
    theta = x_vec(3);
    
    % Unpack costate variables
    lambda_x = x_vec(4);
    lambda_y = x_vec(5);
    lambda_theta = x_vec(6);
    
    % Compute optimal controls based on current state and costates
    [v, omega] = mobile_robot_get_optimal_u(theta, rv, romega, lambda_x, lambda_y, lambda_theta);
    
    % System dynamics (robot kinematics)
    dx = %TODO
    dy = %TODO
    dtheta = %TODO
    
    % Costate dynamics (derived from Hamiltonian system)
    dlambda_x = %TODO
    dlambda_y = %TODO
    dlambda_theta = %TODO
    
    % Pack derivatives into output vector
    dX_lambda_dt = [dx; dy; dtheta; dlambda_x; dlambda_y; dlambda_theta];
end

