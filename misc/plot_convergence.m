function plot_convergence(J_log)
figure(2);
plot(1:length(J_log),J_log,'-k','LineWidth',1.5);
xlabel('k')
ylabel('J(x(k))');
end