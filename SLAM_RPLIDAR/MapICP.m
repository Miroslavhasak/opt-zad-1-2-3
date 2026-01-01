classdef MapICP < handle
    properties (Access = private)
        handle_ uint64 = uint64(0);
    end
    
    methods
        function obj = MapICP(voxelSize,prob_hit,prob_miss)
            obj.handle_ = MapICPMex('new', voxelSize,prob_hit,prob_miss);
        end
        
        function delete(obj)
            if obj.handle_ ~= 0
                MapICPMex('delete', obj.handle_);
                obj.handle_ = uint64(0);
            end
        end
    

        function erase(obj, prob_thr)
            MapICPMex('erase', obj.handle_, prob_thr);
        end
        
        function tf = isEmpty(obj)
            tf = MapICPMex('isEmpty', obj.handle_);
        end
        
        function [points,prob] = getPoints(obj)
            [points,prob] = MapICPMex('getPoints', obj.handle_);
        end

        function [t_hat,R_hat] = alignScan(obj,points,maxDistance,t_hat_init,R_hat_init,GN_tol,ICP_tol_pos,ICP_tol_ori)
            [t_hat,R_hat] = MapICPMex('alignScan', obj.handle_,points, maxDistance,t_hat_init,R_hat_init,GN_tol,ICP_tol_pos,ICP_tol_ori);
        end
    end
    
    methods (Access = private)
        % Disable copying (optional, since handle class)
        function cpObj = copy(obj)
            error('Copy not supported for Map handle class');
        end
    end
end
