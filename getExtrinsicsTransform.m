function [tform] = getExtrinsicsTransform(t, q)
    %Assume given format is qx qy qz qw
    q_reformat = [q(4), q(1), q(2), q(3)];
    
    R = quat2rotm(q_reformat);
    
    T=[R, t';0 0 0 1]';
    
    tform = affine3d(T);
end

