function [x_k, x_log, J_log, s_log] = optim_gradient_optimal_step(J_fun, x_0, eps, log_on)

x_log = [];
J_log = [];
s_log = [];
x_k   = x_0;

while true
    % vyhodnotenie v aktualnom bode
    [J_k, g_xk, ~] = J_fun(x_k);

    % smer zostupu
    p = -g_xk;

    % hladanie optimalneho kroku (line search)
    f_alpha = @(alpha) J_fun(x_k + alpha*p);   % funkcia v smere p
    alpha_opt = fminbnd(@(alpha) f_alpha(alpha), 0, 1);  

    % krok
    x_k = x_k + alpha_opt * p;

    % logovanie
    if log_on
        x_log = [x_log; x_k'];
        J_log = [J_log; J_k];
        s_log = [s_log; norm(g_xk)];
    end

    % podmienka zastavenia
    if norm(g_xk) < eps
        break;
    end
end

end
