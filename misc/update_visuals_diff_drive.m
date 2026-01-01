function update_visuals_diff_drive(h_true_pose,h_est_pose,h_true_dir,h_est_dir,h_text,history,landmarks,step,v_cmd,w_cmd)
x_true = history.x_true(:,end);
x_hat  = history.x_hat(:,end);
P      = history.P(:,:,end);
visible = history.visible{end};

set(h_true_pose,'XData',x_true(1),'YData',x_true(2));
set(h_est_pose,'XData',x_hat(1),'YData',x_hat(2));
set(h_true_dir,'XData',x_true(1),'YData',x_true(2),'UData',cos(x_true(3)),'VData',sin(x_true(3)));
set(h_est_dir,'XData',x_hat(1),'YData',x_hat(2),'UData',cos(x_hat(3)),'VData',sin(x_hat(3)));

delete(findobj(gca,'Tag','cov_ellipse'));
draw_covariance_ellipse(P(1:2,1:2), x_hat(1:2), 3, [1 0 0], 'cov_ellipse');

delete(findobj(gca,'Tag','visible_line'));
for i = visible
    plot([x_true(1), landmarks(1,i)], [x_true(2), landmarks(2,i)], 'g--','Tag','visible_line');
end

set(h_text,'String',sprintf('step=%d | v=%.2f w=%.2f | visible=%d',step,v_cmd,w_cmd,numel(visible)));
end
