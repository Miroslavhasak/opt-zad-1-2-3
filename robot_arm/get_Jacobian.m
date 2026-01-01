function [J] = get_Jacobian(theta, pn, R, dR, T)
% rata geometricky jakobian
J = zeros(3, length(R));

for i = 1:length(R)  % tento cyklus prechadza klby jeden po druhom
    % pre kazdy klb i vyratame jeden stlpec jakobianu to je dobre aby sme vedeli kam sa posunie koniec robota % TODO
    p = pn + T(:, length(R));
    for j = length(R)-1:-1:i
        p = R{j+1}(theta(j+1)) * p;
        p = p + T(:, j);
    end

    p = dR{i}(theta(i)) * p;

    for j = i-1:-1:1
        p = R{j}(theta(j)) * p;
    end

    J(:, i) = p;
end

end