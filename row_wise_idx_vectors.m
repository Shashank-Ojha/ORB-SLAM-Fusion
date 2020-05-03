function [Xs,Ys] = row_wise_idx_vectors(h, w)
    Xs = zeros(h * w, 1);
    Ys = zeros(h * w, 1);
    idx = 1;
    for i=1:h
        for j=1:w
            Xs(idx) = j;
            Ys(idx) = i;
            idx = idx + 1;
        end
    end
end

