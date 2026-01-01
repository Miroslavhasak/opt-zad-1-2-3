function draw_covariance_ellipse(P2, mu, k, color, tag)
P2 = (P2+P2')/2;
try
    L = chol(P2,'lower');
catch
    [V,D] = eig(P2);
    D = max(D,1e-12);
    L = V*sqrt(D);
end
t = linspace(0,2*pi,100);
circle = [cos(t); sin(t)];
ellipse = mu(:) + k*(L*circle);
h = plot(ellipse(1,:), ellipse(2,:), '--', 'Color', color, 'LineWidth', 1.2);
if nargin>=5 && ~isempty(tag), set(h,'Tag',tag); end
end
