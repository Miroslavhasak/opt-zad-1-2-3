function [f,J] = transform_estimate_get_f_J(pA,theta_hat,t_hat)

N=size(pA,1);


[R_hat,d_R_hat_1,d_R_hat_2,d_R_hat_3] = RPY_diff(theta_hat);

f = %TODO;
df_d_theta_1 = reshape(%TODO);
df_d_theta_2 = reshape(%TODO);
df_d_theta_3 = reshape(%TODO);


J=[%TODO];


end