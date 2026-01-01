function [pose_hat,e] = AlignPointsToMap(map,scan, initialGuess, maxCorrespondenceDist)
% Align a scan to the existing map using ICP and return the new pose.
% If no map data exists, return the initial guess.

if map.isEmpty()
    pose_hat = initialGuess;
    e=[];
    return;
end

% Transform input scan using the initial guess
pose_hat=initialGuess;

for iter = 1:1000

    [sources,targets] = map.findCorrespondences(TransformPoints(scan, pose_hat), maxCorrespondenceDist);    
    sources=sources';
    targets=targets';
   
    e=sources-targets;
    

    t_hat=zeros(3,1);
    theta_hat=zeros(3,1);
    while 1
    
        [f,J] = transform_estimate_get_f_J(sources,theta_hat,t_hat);
        
        H=%TODO;
        g=%TODO;

        delta=%TODO;

        theta_hat=%TODO;
        t_hat=%TODO;
       
        if norm(delta) < 1e-4
            break;
        end
    end

    if (theta_hat'*theta_hat+t_hat'*t_hat)<1e-4
        break;
    end


    pose_hat.R=RPY(theta_hat)*pose_hat.R;
    pose_hat.t=pose_hat.t+t_hat;

end




end
