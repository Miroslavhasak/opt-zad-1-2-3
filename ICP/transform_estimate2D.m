function [theta_hat,t_hat,H] = transform_estimate2D(pA,pB,theta_hat,t_hat,tol)

[x_hat,~,H] = optim_GaussNewton(@(x_hat)get_rJ(x_hat,pA,pB),[theta_hat;t_hat],tol);

theta_hat= x_hat(1); % vytiahne prvé číslo z výsledku optimalizácie čo je vyrátaný uhol %TODO
t_hat= x_hat(2:3); % vytiahne zvyšné 2 čísla kt predstavujú posun v smere osi x a y %TODO

end


function [r,J] = get_rJ(x_hat,pA,pB)

% rozdelí vstupný vektor na uhol a posun aby sme to mohli neskôr rátať
theta_hat= x_hat(1); %TODO
t_hat= x_hat(2:3); %TODO

N=size(pA,1);

R_hat= [cos(theta_hat), -sin(theta_hat); 
             sin(theta_hat),  cos(theta_hat)]; % vytvorí maticu ktorá hovorí ako sa body otočia o daný uhol %TODO
d_R_hat= [-sin(theta_hat), -cos(theta_hat); 
                cos(theta_hat), -sin(theta_hat)]; % ide o deriváciu rotácie matica nám hovori ako veľmi sa body posunú keď zmeníme uhol %TODO

pA_transformed = (R_hat * pA')' + t_hat';
r_matrix = pB - pA_transformed;

% vyráta rozdiel medzi bodmi a ako keby ho rozbalí do jedného vektora aby Gauss-Newton vedel aká je celková chyba %TODO
r = r_matrix'; 
r = r(:); 
df_d_theta= (d_R_hat * pA')'; % vyráta zmenu polohy každého bodu v závislosti od zmeny uhla %TODO
J= zeros(2*N, 3); % vytvorí prázdnu nulovú maticu kt má 2 riadky pre každý bod a 3 stĺpce %TODO
for i = 1:N
        % Derivácia podľa theta (uhol)
        J(2*i-1 : 2*i, 1) = -df_d_theta(i, :)'; % zapíše vplyv zmeny uhla do jacobiho matice pre každý bod
        % Derivácia podľa translácie (t_x, t_y) - je to identita
        J(2*i-1 : 2*i, 2:3) = -eye(2); % zapíše vplyv zmeny posunu
end

end