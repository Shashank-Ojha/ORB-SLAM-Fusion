
path = 'rgbd_dataset_freiburg1_room/';

% BASIC INFOMATION ABOUT OUR DATASET % 
h = 480;
w = 640;
total_images = 1362;

% CALIBRATION OF THE COLOR CAMERA 
%(https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect)
fx = 517.3;
fy = 516.5;
cx = 318.6;
cy = 255.3;

% CALIBRATION OF THE DEPTH IMAGES
factor = 5000;

% POINTCLOUD DENSITY %
grid_size = 0.005;


% ---------------------------------------------------------------------- %

% LOAD DATA FROM FILES %

camera_trajectory = readtable('KeyFrameTrajectory.txt');
image_table = readtable('associations.txt');    

% MAP RGB TIMESTAMP TO RGB/DEPTH FILENAMES %
image_map = containers.Map(image_table.Var1, image_table.Var2);
depth_map = containers.Map(image_table.Var1, image_table.Var4);

% ---------------------------------------------------------------------- %


% ======================== Set start time ============================== %
tic;

% SPECIFY FLAGS TO CHANGE BEHAVIOR %
use_fusion = 1;

% SPECIFY NUMBER IMAGES TO USE %
use_images = 25;
total_pts = use_images * h * w;
pc_location = zeros(total_pts, 3);
pc_color = zeros(total_pts, 3, 'uint8');

[Xs, Ys] = row_wise_idx_vectors(h, w);

for i=1:use_images
    fprintf('Frame#: %d\n', i);
    
     % Assume given format of quaternion is qx qy qz qw
    [timestamp, t, q] = extract_traj_props(camera_trajectory, i);
    
    tform = getExtrinsicsTransform(t, q);
    
    % save pose and get corresponding rgb and depth data
    rgb = imread(strcat(path, image_map(timestamp)));
    pixel_colors = reshape(permute(rgb, [2, 1, 3]), [h * w, 3]);
    
    depth = imread(strcat(path, depth_map(timestamp)));
    depth = double(reshape(depth', [h * w, 1]));
    
    % inverse project each pixel (fast version)
    z = depth / factor;
    x = ((Xs - cx) .* z) / fx;
    y = ((Ys - cy) .* z) / fy;
    
    image_pc = pointCloud([x,y,z], 'Color', pixel_colors); 
    camera_alligned_pc = pctransform(image_pc, tform);
%     camera_alligned_pc_ds = downsampleData(image_pc, ds_ratio); 
    
    if(i == 1)
        if(use_fusion)
            fusion_pointcloud = camera_alligned_pc;
        else
            pc_location(1:h*w, :) = camera_alligned_pc.Location;
            pc_color(1:h*w, :) = pixel_colors;
        end   
    else
        if(use_fusion)
            fusion_pointcloud = pcmerge(fusion_pointcloud, camera_alligned_pc, grid_size);
        else
            start_idx = (h * w * (i-1)) + 1;
            end_idx = h * w * i;
            pc_location(start_idx:end_idx, :) = camera_alligned_pc.Location;
            pc_color(start_idx:end_idx, :) = pixel_colors;
        end     
    end
end

if(use_fusion)
    final_pointcloud = fusion_pointcloud;
else
    final_pointcloud = pointCloud(pc_location, 'Color', pc_color); 
end

fprintf('______________________________________________________________________\n');
fprintf('Number of points in the final model = %d \n', final_pointcloud.Count);
fprintf('Compression ratio = %.2f %% \n', final_pointcloud.Count/(h*w*use_images)*100);
fprintf('Total time spent = %.2f sec \n', toc);

% DISPLAY FINAL POINT CLOUD %
pcshow(final_pointcloud)

title('Freiburg Room 1');
xlabel('X');
ylabel('Y');
zlabel('Z');

