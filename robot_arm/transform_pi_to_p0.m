function [p0] = transform_pi_to_p0(theta, pi, i, R, T)
% tato funkcia hovori o doprednej kinematike
% jej ulohou je vziat bod ktory je v suradnicovej sustave nejakeho klbu a
% prepocitat jeho polohu tak aby sme vedeli kde sa nachadza z pohladu
% zakladne robota
p0 = pi;

for j = i:-1:1
    p0 = R{j}(theta(j)) * p0 + T(:,j); %  %TODO
end

end