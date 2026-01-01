function [x_hat, g, H] = optim_GaussNewton(f, x_hat, tol)

while 1

   [r, J] = f(x_hat); 
   
   H=J' * J; % tu rátame hessovu maticu %TODO
   % hessova matica ako keby hovorí o zakrivení chyby 
   % algoritmu pomáha pochopiť, ako sa mení sklon terénu po ktorom 
   % sa kĺžeme smerom k najlepšiemu riešeniu.
   g=J' * r; % tu rátame gradient vektoru smeru %TODO
   % gradient je ako šípka kt ukazuje smerom k najväčšej chybe a keďže my
   % chceme chybu zmenšiť algoritmus tento smer použije na to aby sa vydal
   % presne opačným smerom tam kde chyba klesá

    if g' * g < tol
        break;
    end

   delta=-H \ g; % tu rátame krok úpravy konkrétne smer a velkosť zmeny %TODO
   % 
   x_hat = x_hat + delta; % tu aktualizujeme odhad a to tak že prirátamevypočítanú zmenu %TODO
   % delta a tak sa v každom cykle stávame o niečo presnejšími až dokým sa
   % mračná bodov dokonale neprekrujú
end

end