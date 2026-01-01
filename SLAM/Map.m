classdef Map < handle
    properties (Access = private)
        handle_ uint64 = uint64(0);
    end
    
    methods
        function obj = Map(voxelSize)
            obj.handle_ = MapMex('new', voxelSize);
        end
        
        function delete(obj)
            if obj.handle_ ~= 0
                MapMex('delete', obj.handle_);
                obj.handle_ = uint64(0);
            end
        end
        
        function update(obj, origin, points)
            validateattributes(points, {'double'}, {'ncols',3});
            MapMex('update', obj.handle_, origin, points);
        end

        function erase(obj, prob_thr)
            MapMex('erase', obj.handle_, prob_thr);
        end
        
        function tf = isEmpty(obj)
            tf = MapMex('isEmpty', obj.handle_);
        end
        
        function [sources,targets,prob] = findCorrespondencesProb(obj, points, maxDistance)
            validateattributes(points, {'double'}, {'ncols',3});
            validateattributes(maxDistance, {'double'}, {'scalar','nonnegative'});
            [sources,targets,prob] = MapMex('findCorrespondences', obj.handle_, points, maxDistance);
        end

        function [sources,targets] = findCorrespondences(obj, points, maxDistance)
            validateattributes(points, {'double'}, {'ncols',3});
            validateattributes(maxDistance, {'double'}, {'scalar','nonnegative'});
            [sources,targets] = MapMex('findCorrespondences', obj.handle_, points, maxDistance);
        end

        function [points,prob] = getPoints(obj)
            [points,prob] = MapMex('getPoints', obj.handle_);
        end
    end
    
    methods (Access = private)
        % Disable copying (optional, since handle class)
        function cpObj = copy(obj)
            error('Copy not supported for Map handle class');
        end
    end
end
