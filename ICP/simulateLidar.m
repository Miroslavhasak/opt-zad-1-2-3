function [pA,pB_] = simulateLidar(map, t, theta, maxRange,n_points,stepSize,sigma)
    
    
    phi=linspace(0,2*pi,n_points)+randn(1,n_points)*0.1;

    pA = [];
    pB_ = [];

    R=rotZ(theta);
    R=R(1:2,1:2);

    for i = 1:n_points
        
        Rl=rotZ(phi(i));
        Rl=Rl(1:2,1:2);
        for r = 0:stepSize:maxRange

            pB=t+R*Rl*[r;0];         
            pB=round(pB);
          
            pBn=pB+randn(2,1)*sigma;
        if pB(2)<=size(map,1) && pB(1)<=size(map,2)
            if map(pB(2), pB(1))
                pB_ =[pB_;pBn'];
                pA=[pA; (R'*(pBn-t))'] ;  
                break;
            end
        end    
        end
       
    end

    pA=pA(randperm(size(pA, 1)), :);
    
end



