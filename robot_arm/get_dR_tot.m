function [dR_tot] = get_dR_tot(theta, R, dR, j)
% tato funkcia rata parcialnu derivaciu celkovej rotacie robota podla
% jedneho klbu j
% da sa to predstavit tak ze ked chceme vidiet ako sa zmeni natocenie
% celeho robota ak pohneme iba klbom cislo j a ostatne nechame nehybne
dR_tot = eye(3);

for i = 1:length(R)
    if i == j % ak narazime na klb podla ktoreho derivujeme tak sa vykona podmienka
        % namiesto obycajnej rotacie R pouzivame derivaciu rotacie dR
        % to znamena ze tu sa odohrava zmena pohybu ktoru skumame
        dR_tot = dR_tot * dR{i}(theta(i)); %TODO
    else % toto sa vykona pre vsetky ostatne klby a to pouzijeme klasicku maticu rotacie R 
        % tento klb sa nehybe iba prenasa natocenie z predchadzajucich
        % klbov dalej
        dR_tot = dR_tot * R{i}(theta(i)); %TODO
    end
end

end