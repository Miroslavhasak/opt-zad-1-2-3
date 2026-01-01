function [x, x_log, s, lambda] = optim_quadprog(x_0, A, b, A_c, b_c, eps, log)

x_log = [];

n_params = size(A, 2);
n_con = size(A_c, 1);

theta = zeros(n_params + n_con + n_con, 1);
theta(1:n_params) = x_0;

theta(n_params+1:n_params+n_con) = ones(n_con,1) * 1e-2;
theta(n_params+n_con+1:end) = -(A_c * x_0 - b_c);

while 1
    x = theta(1:n_params);
    lambda = theta(n_params+1:n_params+n_con);
    s = theta(n_params+n_con+1:end);

    if log
        x_log = [x_log; x'];
    end   
    
    %TODO
    r_1 = A*x + b + A_c' * lambda;      % gradient Lagrangeovej funkcie
    r_2 = A_c * x + s - b_c;            % primalne obmedzenia
    r_3 = lambda .* s;                  % komplementarita

    r = [r_1; r_2; r_3];
    
    
    % Jacobian systemu (Newtonova matica)
    %TODO
    J = [A,              A_c',           zeros(n_params, n_con);
         A_c,            zeros(n_con),   eye(n_con);
         zeros(n_con, n_params), diag(s), diag(lambda)];

    % Newtonov krok
    %TODO
    delta = -J \ r;
    
    gamma = 1;

    if sum(theta(n_params+1:end) + delta(n_params+1:end) < 0)
        i = (1:length(theta)) > n_params & theta' + delta' < 0 & delta' < 0;
        gamma = 0.9 * min(-theta(i) ./ delta(i));
    end    
    
    % podmienka zastavenia
    %TODO
    if norm(r) < eps
        break;
    end

    % aktualizacia parametrov
    %TODO
    theta = theta + gamma * delta;
    
end

end