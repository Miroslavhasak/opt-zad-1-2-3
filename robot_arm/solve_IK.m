% function [theta] = solve_IK(pr, theta, R, dR, T, tol)
% 
% while true
%     p0 = transform_pi_to_p0(theta, [0;0;0], length(theta), R, T); %TODO
%     % p0 = transform_pi_to_p0(theta, [0;0;0], n, current_R, current_T); %TODO
%     r = pr - p0; %TODO
%     J = get_Jacobian(theta, [0;0;0], R, dR, T);
%     % J = get_Jacobian(theta, [0;0;0], current_R, current_dR, current_T);
% 
%     H= J' * J; %TODO;
%     delta_theta = H \ (J' * r); %TODO;
%     % delta_theta = (H + 1e-9*eye(n)) \ (J' * r); %TODO
%     theta = theta + delta_theta; %TODO;
% 
%     if (r' * r) < tol
%         break
%     end
% end
% 
% theta = mod(theta, 2*pi);
% 
% end
%%
function [theta] = solve_IK(pr, theta, R, dR, T, tol)
    n = length(theta);
    current_R = R(1:n);
    current_dR = dR(1:n);
    current_T = T(:, 1:n);
    
    max_iter = 500; % Poistka: maximálne 500 opakovaní
    iter = 0;
    alpha = 0.5;    % Tlmenie: urobíme len polovičný krok, aby sme nepreskakovali cieľ

    while iter < max_iter
        p0 = transform_pi_to_p0(theta, [0;0;0], n, current_R, current_T);
        r = pr - p0; 
        
        % Ak je už chyba dosť malá, skonči
        if (r' * r) < tol
            break
        end
        
        J = get_Jacobian(theta, [0;0;0], current_R, current_dR, current_T);
        
        % Gaussov-Newtonov krok s reguláciou (aby matica nebola singulárna)
        H = J' * J + (1e-6 * eye(n)); 
        delta_theta = H \ (J' * r);
        
        % Aktualizácia uhlov s tlmením
        theta = theta + alpha * delta_theta;
        
        iter = iter + 1;
    end

    if iter == max_iter
        disp('Varovanie: IK nedosiahla konvergenciu (max_iter dosiahnutý).');
    end

    theta = mod(theta, 2*pi);
end