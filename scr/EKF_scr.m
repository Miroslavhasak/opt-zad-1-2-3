%% EKF_localization_main.m
close all; clear; clc;

addpath("..\misc");
addpath("..\geometry");

%% --- PARAMETERS ---
dt = 0.05; max_steps = 20000;
sensor_range = 4.0; sensor_fov = deg2rad(90);

%% --- World & landmarks ---
world_limits = [0 30; 0 25];
rng(5);
clusters = [5 5; 25 5; 15 20];
landmarks = [];
for c = 1:size(clusters,1)
    landmarks = [landmarks, clusters(c,:)' + randn(2,10)*1.5];
end
num_landmarks = size(landmarks,2);

%% --- Robot states ---
x_true = [15;10;pi/4];
x_hat  = x_true + [1;-1;0.2];
P = diag([0.5,0.5,deg2rad(10)]).^2;

%% --- Noise ---
Q = diag([0.02^2,0.02^2,0.0003]);
R = diag([0.02,0.002]);
chol_Q = chol(Q,'lower'); chol_R = chol(R,'lower');

%% --- Control ---
v_cmd=0; w_cmd=0; v_speed=0; w_speed=0;
v_max=0.8; w_max=1.2; accel=0.02; friction=0.95;
keys = struct('w',0,'s',0,'a',0,'d',0);

%% --- History ---
history.x_true = zeros(3,0);
history.x_hat  = zeros(3,0);
history.P      = zeros(3,3,0);
history.visible = cell(1,0);

%% --- Setup figure & handles ---
[fig,h_true_pose,h_est_pose,h_true_dir,h_est_dir,h_text] = setup_figure_diff_drive(x_true,x_hat,landmarks,world_limits);

%% --- MAIN LOOP ---
step = 0; keep_running = true;
while step < max_steps && keep_running
    step = step + 1;

    %% --- Control updates ---
    [v_cmd,w_cmd,v_speed,w_speed] = control_update(keys,v_speed,w_speed,v_max,w_max,accel,friction);

    %% --- True motion ---
    w = chol_Q*randn(3,1);
    x_true = x_true + [v_cmd*dt*cos(x_true(3));
                       v_cmd*dt*sin(x_true(3));
                       w_cmd*dt] + w;
    x_true(3) = wrapToPi(x_true(3));

    %% --- EKF prediction ---
    
    A = [1, 0, -v_cmd*dt*sin(x_hat(3));
     0, 1,  v_cmd*dt*cos(x_hat(3));
     0, 0, 1]; %TODO
    
    P = A*P*A' + Q; %TODO

    x_hat = x_hat + [v_cmd*dt*cos(x_hat(3));
                 v_cmd*dt*sin(x_hat(3));
                 w_cmd*dt]; %TODO
    
    x_hat(3) = wrapToPi(x_hat(3));
    

    %% --- Detect visible landmarks ---
    visible = [];
    for i = 1:num_landmarks
        dx = landmarks(1,i)-x_true(1);
        dy = landmarks(2,i)-x_true(2);
        r = sqrt(dx^2+dy^2);
        if r > sensor_range, continue; end
        if abs(wrapToPi(atan2(dy,dx)-x_true(3))) > sensor_fov/2, continue; end
        visible = [visible i];
    end

    %% --- EKF update ---
    if ~isempty(visible)
        for k = 1:numel(visible)
            idx = visible(k);
            x_i = landmarks(1,idx); y_i = landmarks(2,idx);
    
            % delta and q
            dx_hat = x_i - x_hat(1);
            dy_hat = y_i - x_hat(2);
            q = max(dx_hat^2 + dy_hat^2, 1e-6);
            r_hat = sqrt(q);
            if r_hat < 1e-9, continue; end
    
            % Predicted measurement
            y_hat = [sqrt(q);
         atan2(dy_hat, dx_hat) - x_hat(3)]; %TODO
            
            y_hat(2) = wrapToPi(y_hat(2));
    
            % Correct Jacobian
            C_i = [ -dx_hat/sqrt(q), -dy_hat/sqrt(q), 0;
         dy_hat/q,       -dx_hat/q,      -1 ]; %TODO
    
            % Simulated noisy measurement
            dx_true = x_i - x_true(1);
            dy_true = y_i - x_true(2);
            r = sqrt(dx_true^2 + dy_true^2);
            b = wrapToPi(atan2(dy_true,dx_true) - x_true(3));
            y = [r;b] + chol_R*randn(2,1);
    
    
            % Kalman gain
            S = C_i*P*C_i' + R;
            K = P*C_i'/S; %TODO
    
            % Update state
            innov = [y(1) - y_hat(1);
          wrapToPi(y(2) - y_hat(2))];
            x_hat = x_hat + K*innov; %TODO
            x_hat(3) = wrapToPi(x_hat(3));
    
            % Update covariance
            P = (eye(3) - K*C_i)*P; %TODO
        end
    end

    %% --- Save history ---
    history.x_true(:,end+1) = x_true;
    history.x_hat(:,end+1)  = x_hat;
    history.P(:,:,end+1)    = P;
    history.visible{end+1}  = visible;

    %% --- Visualization ---
    if mod(step,2)==0 && ishandle(fig)
        update_visuals_diff_drive(h_true_pose,h_est_pose,h_true_dir,h_est_dir,h_text,history,landmarks,step,v_cmd,w_cmd);
        drawnow limitrate;
    end

    pause(dt*0.8);
end
