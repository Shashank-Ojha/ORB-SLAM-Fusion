function [tform, R, t] = getExtrinsicsTransform(t, q)
    R = quat2rotm(q);
    T = [R [0; 0; 0]; [t 1]];
    
    tform = affine3d(T);
end

