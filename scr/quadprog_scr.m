addpath("..\fcn");
addpath("..\optim_alg");
addpath("..\misc");

%% Quadratic Programming

A=[1,0.5;0.5,3];
b=[-6;10];
c=10;

x_anal=-A\b;

A_c=[1,-2;
     -5,2;
     2,1;
     0.5,1;
     -1,-3];
b_c=[7;15;20;10;15];

J = @(x)(fcn_QF(x,A,b,c));
xl=[-6;-6];
xh=[12;10];

x_0=[2;8];
%x_0=[-2;-2];
%x_0=[6;6];
%x_0=[4;6];

[x_qp,x_log,s,lambda]=optim_quadprog(x_0,A,b,A_c,b_c,1e-6,true)

A_c*x_qp-b_c

plot_2D_lin_ineq(xl,xh,A_c,b_c)
hold on
plot_2D_optim(J,xl,xh,x_qp,x_anal,x_log,[]);

quiver(x_qp(1),x_qp(2),[1,0]*(A*x_qp+b)/3,[0,1]*(A*x_qp+b)/3,1,'-r','LineWidth',1.5);
quiver(x_qp(1),x_qp(2),[1,0]*(A_c'*lambda)/3,[0,1]*(A_c'*lambda)/3,1,'-c','LineWidth',1.5);

