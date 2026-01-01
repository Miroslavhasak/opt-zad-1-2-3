function [theta_hat, t_hat] = ICP_estimate_3D(pA, pB, theta_hat, t_hat, tol, tol_d, KDtree)

i=0;
    while true
        % Build full 3D rotation matrix from RPY angles
        R_hat = %TODO
        
        % Transform all points pA using current estimate
        pA_transformed = %TODO
        
        % Find nearest neighbors from transformed pA to pB
        % Using built-in knnsearch for efficiency (requires Statistics Toolbox)
        [idx, d] = knnsearch(KDtree, pA_transformed,'k',1);
        
        % Update transform estimate based on correspondences
        [theta_hat, t_hat] = transform_estimate(pA, pB(idx, :), theta_hat, t_hat, tol);
        
        i=i+1;
        % Check convergence (mean distance below tolerance)
        if mean(d) < tol_d || i>100
            break;
        end
    end
end