function plot_2D_lin_ineq(xl,xh,A,b)

plotregion(-A,-b,xl,xh,'g');
xlim([xl(1),xh(1)]);
ylim([xl(2),xh(2)]);

  
xlabel('x_1');
ylabel('x_2');

grid on

set(gcf,'position',[0,200,450,350]);

end