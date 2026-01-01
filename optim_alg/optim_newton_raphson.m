function [x_k, x_log, J_log, s_log] = optim_newton_raphson(J_fun, x_0, eps, log_on)

x_log = [];
J_log = [];
s_log = [];
x_k   = x_0;

while true
    % vyhodnot funkciu, gradient a Hessian v bode x_k
    [J_k, g_k, H_k] = J_fun(x_k);

    % Newtonov krok
    dx = -H_k \ g_k;
    x_k = x_k + dx;

    % logovanie
    if log_on
        x_log = [x_log; x_k'];
        J_log = [J_log; J_k];
        s_log = [s_log; norm(dx)];
    end

    % podmienka zastavenia
    if norm(g_k) < eps
        break;
    end
end

end
