function plot_robot(theta,R,T)
% PLOT_ROBOT Visualizes a robot manipulator in 3D space.
%
%   plot_robot(theta, R, T) plots the robot's links and coordinate frames 
%   using the joint variables and transformation parameters.
%
%   Inputs:
%       theta - Vector of joint variables (e.g., joint angles)
%       R     - Cell array or array of rotation matrices for each link/frame
%       T     - Cell array or array of translation vectors for each link/frame
%
%   Description:
%       The function plots the robot manipulator in 3D by:
%         - Setting up a 3D plot with axes labeled and grid enabled.
%         - Drawing the global coordinate axes (X in red, Y in green, Z in blue).
%         - Iteratively calculating and plotting each joint position.
%         - Drawing links between consecutive joints as black lines.
%         - Plotting joint positions as black filled circles.
%         - Drawing local coordinate axes at each joint frame (small colored arrows).
%
%   Note:
%       The function relies on another function `transform_pi_to_p0` to compute
%       the position of points in the base coordinate frame, given joint variables
%       and transformation parameters.
%
%   Example:
%       plot_robot(theta, R, T);

figure(1);
hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3);

% Plot global coordinate axes
quiver3(0,0,0,1,0,0,0.3,'r');
quiver3(0,0,0,0,1,0,0.3,'g');
quiver3(0,0,0,0,0,1,0.3,'b');

P0=[0;0;0];
for i=1:length(R)
    Pi=transform_pi_to_p0(theta,[0;0;0],i,R,T);

    % Plot link between previous and current joint
    plot3([P0(1) Pi(1)], [P0(2) Pi(2)], [P0(3) Pi(3)], 'k', 'LineWidth', 3);
    % Plot current joint as a filled circle
    scatter3(Pi(1), Pi(2), Pi(3), 100, 'k', 'filled');
    P0=Pi;

    % Plot local coordinate frame axes at joint i
    e=transform_pi_to_p0(theta,[1;0;0],i,R,T);
    quiver3(Pi(1),Pi(2),Pi(3),e(1)-Pi(1),e(2)-Pi(2),e(3)-Pi(3),0.2,'r');
    e=transform_pi_to_p0(theta,[0;1;0],i,R,T);
    quiver3(Pi(1),Pi(2),Pi(3),e(1)-Pi(1),e(2)-Pi(2),e(3)-Pi(3),0.2,'g');
    e=transform_pi_to_p0(theta,[0;0;1],i,R,T);
    quiver3(Pi(1),Pi(2),Pi(3),e(1)-Pi(1),e(2)-Pi(2),e(3)-Pi(3),0.2,'b');
end

end