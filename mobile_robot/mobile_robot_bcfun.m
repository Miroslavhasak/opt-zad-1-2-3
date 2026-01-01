function res = mobile_robot_bcfun(ya, yb, x0, y0, theta0)
    % Boundary condition function for BVP solver
    %
    % Ensures initial state matches starting pose and final costates vanish
    

    res = [ya(1) - x0;       % initial x position equals x0
           ya(2) - y0;       % initial y position equals y0
           ya(3) - theta0;   % initial orientation equals theta0
           yb(4);            % terminal lambda_x = 0
           yb(5);            % terminal lambda_y = 0
           yb(6)];           % terminal lambda_theta = 0
end