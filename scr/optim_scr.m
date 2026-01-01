addpath("..\fcn");
addpath("..\optim_alg");
addpath("..\misc");

%% Himmelblau function
clear; clc; close all;
J = @(x)(fcn_himmelblau(x));
xl=[-5;-5];
xh=[5;5];
x_anal=[3,2;-2.805118,3.131312;-3.779310,-3.283186;3.584458,-1.848126]';

 % x_0=[4;-4];
 x_0=[4;4];
 % x_0=[-4;4];
 % x_0=[-4;-4];
 % x_0=[0;0];
 % x_0=[0;4];
 % x_0=[4;0];
 % x_0=[-4;0];
 % x_0=[0;-4];

plot_2D_function(J,xl,xh);
%% Quadratic form
clear;

A=[1,0.5;0.5,3];
b=[-6;10];
c=10;

J = @(x)(fcn_QF(x,A,b,c));
xl=[-10;-15];
xh=[20;10];
plot_2D_function(J,xl,xh);
x_anal=-A\b;

x_0=[-10;10];
%x_0=[15;5];
%x_0=[-10;-5];
%x_0=[0;0];



%% Gradient method
gamma=2e-1;
%gamma=0.7;
%gamma=1e-2;
%gamma=1e-3;

eps=1e-3;
[x_grad,x_log,J_log,s_log]=optim_gradient(J,x_0,eps,gamma,true);
[~,~,H]=J(x_grad);
eig(H)
plot_2D_optim(J,xl,xh,x_grad,x_anal,x_log,s_log);
plot_convergence(J_log)
%% Gradient method with optimal step

eps=1e-3;
[x_grad,x_log,J_log,s_log]=optim_gradient_optimal_step(J,x_0,eps,true);
[~,~,H]=J(x_grad);
eig(H)
plot_2D_optim(J,xl,xh,x_grad,x_anal,x_log,s_log);
plot_convergence(J_log)
%% Newton Raphson method

[x_newton,x_log,J_log,s_log]=optim_newton_raphson(J,x_0,1e-3,true);
[~,~,H]=J(x_newton);
eig(H)
plot_2D_optim(J,xl,xh,x_newton,x_anal,x_log,s_log);
plot_convergence(J_log)



