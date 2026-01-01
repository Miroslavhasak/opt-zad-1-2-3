function [fig,h_true_pose,h_est_pose,h_true_dir,h_est_dir,h_text] = setup_figure_diff_drive(x_true,x_hat,landmarks,world_limits)
    % Create figure
    fig = figure('Name','EKF Localization Student','NumberTitle','off',...
        'KeyPressFcn',@(s,e) mykeyboard('down',s,e),...
        'KeyReleaseFcn',@(s,e) mykeyboard('up',s,e),...
        'CloseRequestFcn',@(s,e) mykeyboard('close',s));
    
    axis equal; hold on; grid on;
    xlim(world_limits(1,:)); ylim(world_limits(2,:));
    
    n_clusters = 3;  % adjust if needed
    points_per_cluster = size(landmarks,2)/n_clusters;
    colors = lines(n_clusters);  % distinct colors
    for c = 1:n_clusters
        idx = (c-1)*points_per_cluster + 1 : c*points_per_cluster;
        scatter(landmarks(1,idx), landmarks(2,idx), 80, colors(c,:), '^', 'filled');
    end

    h_true_pose = plot(x_true(1),x_true(2),'ok','MarkerFaceColor','k');
    h_est_pose  = plot(x_hat(1),x_hat(2),'or','MarkerFaceColor','r');
    h_true_dir  = quiver(x_true(1),x_true(2),cos(x_true(3)),sin(x_true(3)),0.6,'-k','LineWidth',1);
    h_est_dir   = quiver(x_hat(1),x_hat(2),cos(x_hat(3)),sin(x_hat(3)),0.6,'-r','LineWidth',1);
    h_text = text(0.02,0.98,'','Units','normalized','FontSize',10,'VerticalAlignment','top');
end
