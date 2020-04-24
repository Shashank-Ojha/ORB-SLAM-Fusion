
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
ds = 1.035;
factor = 5000; %ds;

% SPECIFY NUMBER IMAGES TO USE %
use_images = 10;
total_pts = use_images * h * w;

% POINTCLOUD DENSITY %
grid_size = 0.01;

% ---------------------------------------------------------------------- %

% LOAD DATA FROM FILES %

camera_trajectory = readtable('CameraTrajectory.txt');
image_table = readtable('associations.txt');    

% MAP RGB TIMESTAMP TO RGB/DEPTH FILENAMES %
image_map = containers.Map(image_table.Var1, image_table.Var2);
depth_map = containers.Map(image_table.Var1, image_table.Var4);

% ---------------------------------------------------------------------- %


%==== Set start time ====
tic;


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

for i=1:use_images

    % get ith camera trajectory
    curr_traj = table2array(camera_trajectory(i, :));
    timestamp = curr_traj(1);
    t = curr_traj(2:4);
    q = curr_traj(5:8);
    [tform, R, t] = getExtrinsicsTransform(t, q);
    
    % save pose and get corresponding rgb and depth data
    rgb = imread(strcat(path, image_map(timestamp)));
    depth = imread(strcat(path, depth_map(timestamp)));
    depth = double(reshape(depth', [h * w, 1]));
    
    
    % inverse project each pixel (fast version)
    z = depth / factor;
    x = ((Xs - cx) .* z) ./ fx;
    y = ((Ys - cy) .* z) ./ fy;
    
    % format colors
    pixel_colors = reshape(permute(rgb, [2, 1, 3]), [h * w, 3]);
    
    transformed = pctransform(pointCloud([x,y,z], 'Color', pixel_colors), tform);
    
    if(i == 1)
        fusion_pointcloud = transformed;
    else
        fusion_pointcloud = pcmerge(fusion_pointcloud, transformed, grid_size);
    end
end

fprintf('______________________________________________________________________\n');
fprintf('Number of points in the final model = %d \n', fusion_pointcloud.Count);
fprintf('Compression ratio = %.2f %% \n', fusion_pointcloud.Count/(h*w*use_images)*100);
fprintf('Total time spent = %.2f sec \n', toc);

pcshow(fusion_pointcloud)
title('Freiburg Room 1');
xlabel('X');
ylabel('Y');
zlabel('Z');




