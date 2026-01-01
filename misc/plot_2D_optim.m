function plot_2D_optim(J,xl,xh,x_num,x_anal,x_log,s_log)

%close all;
figure(1);

[x1,x2]=meshgrid(xl(1):0.1:xh(1),xl(2):0.1:xh(2));
y=zeros(size(x1));

hold on;
grid on;

for i=1:size(x1,1)
    for j=1:size(x1,2)
        y(i,j)=J([x1(i,j);x2(i,j)]);
    end
end

contour(x1,x2,y,50);

%axis equal;

%plot(x_anal(1,:),x_anal(2,:),'+r','LineWidth',1.5);
if ~isempty(x_num)
   plot(x_num(1),x_num(2),'or','LineWidth',1.5);
end

if ~isempty(x_log)
    plot(x_log(:,1),x_log(:,2),'ob','LineWidth',1.5);
    plot(x_log(:,1),x_log(:,2),'-b','LineWidth',1.5);
end

% for i=1:size(s_log,1)
%     quiver(x_log(i,1),x_log(i,2),s_log(i,1)/5,s_log(i,2)/5,1.5,'-g','LineWidth',1.0);
% end

%plot(x_anal(1,:),x_anal(2,:),'+r','LineWidth',1.5);
if ~isempty(x_num)
    plot(x_num(1),x_num(2),'or','LineWidth',1.5);
end

xlim([xl(1),xh(1)]);
ylim([xl(2),xh(2)]);

%legend('J(x)','x_{anal}','x_{num}');

xlabel('x_1');
ylabel('x_2');

set(gcf,'position',[0,200,450,350]);

end


