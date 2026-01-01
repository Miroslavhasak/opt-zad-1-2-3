function quad_viz(xhist,n_steps,h)

%% --- 3D Simulator ---
vrsetpref('DefaultViewer','internal');
w = vrworld('quad_world.wrl');
open(w);
view(w,'-internal');
vrdrawnow;

quadNode = vrnode(w,'Quad');
camNode  = vrnode(w,'CamTransform');

camNode.translation = [0, 2, 0];
camNode.rotation    = [1 0 0 1.2];

matlab2vrml_pos = @(r) [r(1); r(2); r(3)];
matlab2vrml_rot = @(q) deal([q(2); q(3); q(4)] / norm([q(2); q(3); q(4)]), 2*acos(q(1)));

for k = 1:n_steps
    r = xhist(1:3,k);                 % position [x;y;z]
    q = xhist(4:7,k)/norm(xhist(4:7,k)); % quaternion [w;x;y;z]

    r_vr = matlab2vrml_pos(r);          
    [axis_vr, angle] = matlab2vrml_rot(q);

    quadNode.translation = r_vr';
    quadNode.rotation = [axis_vr' angle];

    vrdrawnow;
    pause(h);
end
vrclose
vrclose all
end