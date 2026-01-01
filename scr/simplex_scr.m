addpath("..\fcn");
addpath("..\optim_alg");
addpath("..\misc");

%% Himmelblau function
clear;
J = @(x)(fcn_himmelblau(x));
xl=[-5;-5];
xh=[5;5];
x_anal=[3,2;-2.805118,3.131312;-3.779310,-3.283186;3.584458,-1.848126]';
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
%% Nelder-Mead method
s0=[xl(1),xl(2);xl(1),xh(2);xh(1),(xl(2)+xh(2))/2];

alpha=0.75;
beta=1.5;
gamma=0.5;
delta=0.25;

 % alpha=0.5;
 % beta=1.2;
 % gamma=0.7;
 % delta=0.7;

eps=1e-4;
[x_nelder_mead,x_log,J_log] = optim_nelder_mead(J,s0,alpha,beta,gamma,delta,eps,true);
plot_2D_optim(J,xl,xh,x_nelder_mead,x_anal,x_log,[]);




