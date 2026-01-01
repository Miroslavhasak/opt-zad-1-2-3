function [theta] = solve_IK_orientation(pr, Rr, theta, R, dR, T, tol)
% 
lambda=1e0;

while true
  
    p0 = transform_pi_to_p0(theta, [0;0;0], length(R), R, T);
    rp = pr - p0; % tu s rata rozdiel medzi tym kde robot ma byt pr a kde sa aktualne nachadza p0 je to ako keby vektor kt ukazuje smerom k cielu %TODO

    R_tot = get_R_tot(theta, R);
 
    Re = R_tot' * Rr; % zistuje rozdiel v natoceni vynasobenim cielovej rotacie Rr a transponovanej aktualnej rotacie R_tot ziskame maticu chyby ak by bol
    % robot otoceny presne Re by bola jednotkova matica %TODO
    ro = 0.5 * [Re(3,2)-Re(2,3); Re(1,3)-Re(3,1); Re(2,1)-Re(1,2)]; % tu sa meni matica chyby Re na jednoduchy vektor chyby orientacie, 
    % vybera konkretne prvky matice ktore reprezentuju rozdiely v uhloch %TODO
    Jp = get_Jacobian(theta, [0;0;0], R, dR, T);

   
    Jo = zeros(3, length(R));

    for i = 1:length(R)
        dR_tot = get_dR_tot(theta, R, dR, i);
       
        Jo(:, i) = 0.5 * [dR_tot(3,2)-dR_tot(2,3);
                           dR_tot(1,3)-dR_tot(3,1);
                           dR_tot(2,1)-dR_tot(1,2)]; % tu berieme derivaciu celkovej rotacie dR_tot a extrahujeme z nej zmeny v uhloch pre kazdy klb i %TODO
    end

    J = [Jp; Jo]; % Jp polohovy jakobian a Jo orientacny jakobian dame pod seba a vznikne nam jedna velka matica kt popisuje kompletnu citlivost robota na pohyb %TODO    
    H = J' * J + lambda * eye(length(theta)); % vytvarame maticu H (hessian) pridanie lambda je levenberg-marquardtova regulacia sluzi na to aby kod nezamrzol ked je robot v neprirodzenej polohe %TODO
    delta_theta = H \ (J' * [rp; ro]); % hladame take zmeny uhlov delta_theta kt najlepsie zmensia chybu polohy rp aj orientacie ro naraz %TODO
    theta = theta + delta_theta(1:length(theta)); % tu sa aktualizuju uhly klbov a to tak ze k aktualnym uhlom pripocitame vypocitanu zmenu a v dalsom kole cyklu je robot uz o kusok blizsie k cielu %TODO
    
    if (rp' * rp + ro' * ro) < tol
        break
    end
end

end
