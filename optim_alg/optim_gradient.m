function [x_k, x_log, J_log, s_log] = optim_gradient(J_fun, x_0, eps, gamma, log_on)

x_log = [];
J_log = [];
s_log = [];
x_k   = x_0;

while true
    % vyhodnotenie funkcie, gradientu a hessiana
    [J_k, g_xk, ~] = J_fun(x_k);

    % krok gradient descentu
    x_k = x_k - gamma * g_xk;

    % ak je zapnute logovanie, uloz priebeh
    if log_on
        x_log = [x_log; x_k'];
        J_log = [J_log; J_k];
        s_log = [s_log; norm(g_xk)];
    end

    % konvergencia – norma gradientu je mensia ako eps
    if norm(g_xk) < eps
        break;
    end
end

end
