function [theta] = solve_IK_orientation(pr, Rr, theta, R, dR, T, tol)

lambda=1e0; 

while true 
  
    p0 = transform_pi_to_p0(theta, [0;0;0], length(R), R, T);
    rp = p0 - pr; % tu s rata rozdiel medzi tym kde robot ma byt pr a kde sa aktualne nachadza p0 je to ako keby vektor kt ukazuje smerom k cielu %TODO
	
    R_tot = get_R_tot(theta, R); % zistuje rozdiel v natoceni vynasobenim cielovej rotacie Rr a transponovanej aktualnej rotacie R_tot ziskame maticu chyby ak by bol
    % robot otoceny presne Re by bola jednotkova matica %TODO
	
    Re= R_tot * Rr'; % vyrata maticu relativnej rotacie medzi akt orientaciou robota rtot a cielovou orientaciou rr co umoznuje porovnat vzajomne natocenie ich bazovych vektorov %TODO
	
    ro = [ [0 1 0] * Re * [1;0;0];
           [0 0 1] * Re * [1;0;0];
           [0 0 1] * Re * [0;1;0];]; % extrahuje 3 zlozky chyby orientacie tym ze premieta osi aktualneho ramu do rovin cieloveho ramu a nulove hodnoty znamenaju ze osi su dokonalo zarovnane %TODO
		   
	Jp = get_Jacobian(theta, [0;0;0], R, dR, T);
	
    Jo = zeros(3, length(R)); 
    
    for i = 1:length(R)
        dR_tot = get_dR_tot(theta, R, dR, i); 

        Jo(1,i) = [0 1 0] * dR_tot * Rr' * [1;0;0]; % urcuje ako pohyb klbu i meni odchylku medzi osou x robota a osou y ciela %TODO
        Jo(2,i) = [0 0 1] * dR_tot * Rr' * [1;0;0]; % urcuje ako pohyb klbu i meni odchylku medzi osou x robota a osou z ciela %TODO
        Jo(3,i) = [0 0 1] * dR_tot * Rr' * [0;1;0]; % urcuje ako pohyb klbu i meni odchylku medzi osou y robota a osou z ciela %TODO
    end

    J = [Jp; Jo]; % Jp polohovy jakobian a Jo orientacny jakobian dame pod seba a vznikne nam jedna velka matica kt popisuje kompletnu citlivost robota na pohyb %TODO 
	
    H = J' * J + lambda * eye(length(theta)); % vytvarame maticu H (hessian) pridanie lambda je levenberg-marquardtova regulacia sluzi na to aby kod nezamrzol ked je robot v neprirodzenej polohe %TODO
	
    delta_theta = H \ (J' * [rp; ro]); % hladame take zmeny uhlov delta_theta kt najlepsie zmensia chybu polohy rp aj orientacie ro naraz %TODO
	
    theta = theta - delta_theta; % aktualizuje vektory uhlov odcitanim vypocitanej zmeny kde v kazdom kroku iteracie posuva robota blizsie k cielovej pozicii a orientacii %TODO
    if (rp' * rp + ro' * ro) < tol         
		break
    end
end

end
