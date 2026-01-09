function [theta_hat, t_hat] = ICP_estimate_2D(pA, pB, theta_hat, t_hat, tol, tol_d, KDtree)

i=0;
    while true
        % 2D rotation matrix from theta_hat (single angle)
        R_hat = [cos(theta_hat), -sin(theta_hat); 
                 sin(theta_hat),  cos(theta_hat)]; % tu vytvárame 2d rotačnú maticu z aktuálneho odhadu uhla theta %TODO
        % je to matematický predpis kt otočí určitý bod o konkrétny uhol
        % bez tejto matice by sme nevedeli pohnúť mračnom bodov v kruhovom
        % smere
        

        % Transform points pA using current estimate
        pA_transformed = (R_hat * pA')' + t_hat'; % zoberie všetky body z pA a v pamäti ich otočí a prida k nim posun t_hat %TODO
        
        % Find nearest neighbors and distances using knnsearch
        [idx, d] = knnsearch(KDtree, pA_transformed,'k',1);
        
        % Získanie korešpondujúcich bodov z pB na základe indexov
        pB_corr = pB(idx, :);

        % Estimate updated transform parameters with correspondences
        % tu voláme pomocnú funkciu s gauss newtonom kde jej posiela
        % pôvodné body pA a ku nim najbližšie body pB_corr
        % je to ako keby taký vylepšovač že my mu dáme dvojicu bodov a on
        % skúša nájsť ešte lepšiu rotáciu a posun aby boli ku sebe bližšie
        % než doteraz a funkcia vracia aktualizované hodnoty theta_hat a
        % t_hat
        [theta_hat, t_hat, ~] = transform_estimate2D(pA, pB_corr, theta_hat, t_hat, tol); %TODO, theta_hat, t_hat, tol);

        % Check convergence by mean distance
        i=i+1;
        if mean(d) < tol_d || i>200
            break;
        end
    end

end