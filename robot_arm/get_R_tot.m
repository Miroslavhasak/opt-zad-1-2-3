function [R_tot] = get_R_tot(theta, R)

R_tot = eye(3,3);

for i = 1:length(R)
    R_tot = R_tot * R{i}(theta(i)); % tento riadok rata ako je robot ako celok natoceny v priestore 
    % tym ze postupne nasobi matice rotacie kazdeho jedneho klbu %TODO
end

end