function plot_theta_p0(theta_, p0_, t)
% PLOT_THETA_P0 Plots joint variables and position trajectories over time.
%
%   plot_theta_p0(theta_, p0_, t) creates two figures:
%     - The first figure contains subplots of joint variables \theta_i versus time.
%     - The second figure contains three subplots for the components of position p0_
%       (x, y, and z) versus time.
%
%   Inputs:
%       theta_ - Matrix of joint variables, size [n_joints x n_time_steps]
%       p0_    - Matrix of position vectors, size [3 x n_time_steps]
%       t      - Vector of time instants corresponding to columns of theta_ and p0_
%
%   Description:
%       The function plots:
%         1. Each row of theta_ in a separate subplot with label θ_i.
%         2. The x, y, z components of the position p0_ in separate subplots.
%
%   Example:
%       plot_theta_p0(theta_matrix, position_matrix, time_vector);

figure

n = size(theta_, 1);

for i = 1:n
    subplot(n,1,i);
    plot(t, theta_(i,:), '-k', 'LineWidth', 1.5);
    grid on
    xlabel('t')
    ylabel("\theta_" + num2str(i));
end

figure

subplot(3,1,1);
plot(t, p0_(1,:), '-k', 'LineWidth', 1.5);
grid on
xlabel('t')
ylabel('p_0^x(t)')

subplot(3,1,2);
plot(t, p0_(2,:), '-k', 'LineWidth', 1.5);
grid on
xlabel('t')
ylabel('p_0^y(t)')

subplot(3,1,3);
plot(t, p0_(3,:), '-k', 'LineWidth', 1.5);
grid on
xlabel('t')
ylabel('p_0^z(t)')

end