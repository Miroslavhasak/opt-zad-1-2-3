function transformed = TransformPoints(points, pose)

    transformed=pose.R*points'+repmat(pose.t,1,size(points,1));
    transformed=transformed';
end
