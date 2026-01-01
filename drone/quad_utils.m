function utils = quad_utils(m, J, g, kt, km, l, h)
%QUAD_UTILS Returns all quadrotor and quaternion utility functions
%   m, J, g, kt, km, l: quadrotor parameters
%   h: sampling time for RK4

%% --- Quaternion Utilities ---
utils.hat = @(v) [  0   -v(3)  v(2);
                    v(3)  0   -v(1);
                   -v(2)  v(1)  0 ];

utils.L_fn = @(q) [q(1) -q(2:4)';
                   q(2:4) q(1)*eye(3)+utils.hat(q(2:4))];

utils.T = diag([1 -1 -1 -1]);
utils.H = [zeros(1,3); eye(3)];

utils.qtoQ = @(q) utils.H' * utils.T * utils.L_fn(q) * utils.T * utils.L_fn(q) * utils.H;
utils.G_fn = @(q) utils.L_fn(q) * utils.H;
utils.rptoq = @(phi) (1/sqrt(1 + phi'*phi)) * [1; phi];
utils.qtorp = @(q) q(2:4) / q(1);

%% --- Helper Functions ---
utils.E_fn = @(q) blkdiag(eye(3), utils.G_fn(q), eye(6));

utils.quad_dynamics = @(x,u) quad_dyn_core(x,u,m,J,g,kt,km,l, ...
                                           utils.L_fn, utils.H, utils.qtoQ, utils.hat);

utils.quad_dynamics_rk4 = @(x,u) rk4_step(utils.quad_dynamics, x, u, h);

%% --- Core Functions ---
    function dx = quad_dyn_core(x,u,m,J,g,kt,km,l,L,H,qtoQ,hat)
        r = x(1:3);
        q = x(4:7)/norm(x(4:7));
        v = x(8:10);
        w = x(11:13);

        Q = qtoQ(q);
        rdot = Q * v;
        qdot = 0.5 * L(q) * H * w;
        vdot = Q' * [0;0;-g] + (1/m) * [zeros(2,4); kt*ones(1,4)]*u - hat(w)*v;
        wdot = J \ (-hat(w)*J*w + [0 l*kt 0 -l*kt; -l*kt 0 l*kt 0; km -km km -km]*u);

        dx = [rdot; qdot; vdot; wdot];
    end

    function xn = rk4_step(f,x,u,h)
        f1 = f(x,u);
        f2 = f(x + 0.5*h*f1, u);
        f3 = f(x + 0.5*h*f2, u);
        f4 = f(x + h*f3, u);
        xn = x + (h/6) * (f1 + 2*f2 + 2*f3 + f4);
        xn(4:7) = xn(4:7)/norm(xn(4:7));  % normalize quaternion
    end

%% --- Additional Utilities ---
utils.fjacobian = @(fun,x) fjacobian(fun,x);
utils.quat2eul = @(q) quat2eul(q);

    function J = fjacobian(fun,x)
        f0 = fun(x);
        n = numel(x);
        m = numel(f0);
        J = zeros(m,n);
        eps_fd = 1e-3;
        for i = 1:n
            xp = x;
            xp(i) = xp(i) + eps_fd;
            J(:,i) = (fun(xp) - f0)/eps_fd;
        end
    end

    function [roll, pitch, yaw] = quat2eul(q)
        % Convert quaternion [q0; q1; q2; q3] (scalar first) to ZYX Euler angles
        q0 = q(1); qx = q(2); qy = q(3); qz = q(4);
        roll  = atan2( 2*(q0*qx + qy*qz), 1 - 2*(qx^2 + qy^2) );
        pitch = asin( max(-1,min(1, 2*(q0*qy - qz*qx))) );
        yaw   = atan2( 2*(q0*qz + qx*qy), 1 - 2*(qy^2 + qz^2) );
    end

end
