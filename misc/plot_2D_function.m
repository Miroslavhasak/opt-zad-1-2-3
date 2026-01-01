function plot_2D_function(J,xl,xh)

%close all;
figure(3);

[x1,x2]=meshgrid(xl(1):0.1:xh(1),xl(2):0.1:xh(2));
y=zeros(size(x1));
dJ_dx1=zeros(size(x1));
dJ_dx2=zeros(size(x1));
y=zeros(size(x1));

hold on;
%grid on;

for i=1:size(x1,1)
    for j=1:size(x1,2)
        [y_,g_]=J([x1(i,j);x2(i,j)]);
        y(i,j)=y_;
        dJ_dx1(i,j)=g_(1);
        dJ_dx2(i,j)=g_(2);
    end
end

%set(streamslice(x1,x2,dJ_dx1,dJ_dx2,1),'Color','r');

contour(x1,x2,y,50);

%axis equal;

xlim([xl(1),xh(1)]);
ylim([xl(2),xh(2)]);

xlabel('x_1');
ylabel('x_2');

end