function [ds_pointcloud] = downsampleData(pointcloud, ds_ratio)
    
    %==== Downsample both pointcloud and normals ====
    ds_points = pointcloud.Location(1:ds_ratio:end, :);
    ds_colors = pointcloud.Color(1:ds_ratio:end, :);
    ds_pointcloud = pointCloud(ds_points, 'Color', ds_colors);    
end