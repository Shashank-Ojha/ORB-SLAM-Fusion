function [timestamp, t, q] = extract_traj_props(camera_trajectory, i)
    curr_traj = table2array(camera_trajectory(i, :));
    timestamp = curr_traj(1);
    t = curr_traj(2:4);
    q = curr_traj(5:8);
end