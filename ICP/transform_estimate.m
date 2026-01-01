function [theta_hat,t_hat] = transform_estimate(pA,pB,theta_hat,t_hat,tol)

x_hat = optim_GaussNewton(@(x_hat)get_rJ(x_hat,pA,pB),[theta_hat;t_hat],tol);

theta_hat=x_hat(1:3);  %TODO
t_hat=x_hat(4:6);  %TODO

end


function [r,J] = get_rJ(x_hat,pA,pB)


theta_hat=x_hat(1:3);%TODO
t_hat=x_hat(4:6);%TODO

N=size(pA,1);
I = repmap(eye(3, N, 1));
tx = t_hat(1);
ty = t_hat(2);
tz = t_hat(3);

thx = theta_hat(1);
thy = theta_hat(2);
thz = theta_hat(3);

% % rotation matrices
Rx = rotX(theta_hat(1));
Ry = rotY(theta_hat(2));
Rz = rotZ(theta_hat(3));

dRx = drotX(thx);
dRy = drotY(thy);
dRz = drotZ(thz);

R_hat=Rz * Ry * Rx;  %TODO

d_R_hat_1=Rz*Ry*drotX(theta_hat(1));
d_R_hat_2=Rz*drotY(theta_hat(2))*Rx;
d_R_hat_3=drotZ(theta_hat(3))*Ry*Rx;

r = reshape(R_hat*pA' + t_hat - pB', [], 1);

df_d_theta_1 = reshape(d_R_hat_1 * pA', [], 1);  
df_d_theta_2 = reshape(d_R_hat_2 * pA', [], 1);
df_d_theta_3 = reshape(d_R_hat_3 * pA', [], 1);

J = [df_d_theta_1, df_d_theta_2, df_d_theta_3, I];

end