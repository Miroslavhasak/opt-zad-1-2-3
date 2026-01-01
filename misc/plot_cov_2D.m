function plot_cov_2D(Sigma,mu,style)

% Eigen decomposition
[V, D] = eig(Sigma);

% Extract the eigenvalues and eigenvectors
eigval = diag(D);
[~, idx] = sort(eigval, 'descend');
eigval = eigval(idx);
V = V(:, idx);

% Angle of ellipse rotation (in degrees)
theta = atan2(V(2,1), V(1,1)) * (180/pi);

% Ellipse axis lengths (1-sigma)
a = sqrt(eigval(1));  % semi-major axis
b = sqrt(eigval(2));  % semi-minor axis

% Generate ellipse points
t = linspace(0, 2*pi, 100);
ellipse = [a*cos(t); b*sin(t)];

% Rotate the ellipse
R = V;
ellipse_rotated = R * ellipse;

% Translate to mean
ellipse_translated = ellipse_rotated + mu;

% Plot
plot(ellipse_translated(1, :), ellipse_translated(2, :), style, 'LineWidth', 0.5);
hold on;

axis equal
end