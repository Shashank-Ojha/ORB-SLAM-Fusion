function [full_pc_shared] = pc_intersection(full_pc, new_pc, grid_size)
    % basic idea: store 1 in every grid cell where new_pc is active - 0
    % otherwise
    
    % Map each point in full_pc to grind cell and keep points that match
    
    % repeat for new_pc
    
    full_pc_shared = filter_bb(full_pc, new_pc, grid_size);
    
    full_pc_grid = round(full_pc_shared.Location / grid_size) + 1000;
    new_pc_grid = round(new_pc.Location / grid_size) + 1000;
    
    hash_full_pc = get_hashes(full_pc_grid);
    hash_new_pc = get_hashes(new_pc_grid);
    
    bit_vector_full = get_overlap(hash_full_pc, hash_new_pc);
    new_locs = full_pc.Location(bit_vector_full, :);
    new_colors = full_pc.Color(bit_vector_full, :);
    full_pc_shared = pointCloud(new_locs, 'Color', new_colors);
end

function [full_pc_shared] = filter_bb(full_pc, new_pc, grid_size)

    full_pc_grid = round(full_pc.Location / grid_size) + 1000;
    new_pc_grid = round(new_pc.Location / grid_size) + 1000;

    min_x = min(new_pc_grid(:, 1));
    min_y = min(new_pc_grid(:, 2));
    min_z = min(new_pc_grid(:, 3));
    
    max_x = max(new_pc_grid(:, 1));
    max_y = max(new_pc_grid(:, 2));
    max_z = max(new_pc_grid(:, 3));
    
    in_range = (min_x <= full_pc_grid(:, 1) & full_pc_grid(:, 1) <= max_x) ...
             & (min_y <= full_pc_grid(:, 2) & full_pc_grid(:, 2) <= max_y) ...
             & (min_z <= full_pc_grid(:, 3) & full_pc_grid(:, 3) <= max_z);
         
    bit_vector_full = in_range == 1;
    
    new_locs = full_pc.Location(bit_vector_full, :);
    new_colors = full_pc.Color(bit_vector_full, :);
    full_pc_shared = pointCloud(new_locs, 'Color', new_colors);
end

function [bit_vector_full] = get_overlap(hash_full_pc, hash_new_pc)
    n_full = size(hash_full_pc, 1);
    hash_full_map = containers.Map(hash_full_pc, ones(n_full, 1));
    
    n_new = size(hash_new_pc, 1);
    hash_new_map = containers.Map(hash_new_pc, ones(n_new, 1));
    
    bit_vector_full = zeros(n_full, 1);
    for i=1:n_full
        bit_vector_full(i) = isKey(hash_new_map, hash_full_pc(i));
    end
    
    bit_vector_full = bit_vector_full == 1;
%     bit_vector_full = isKey(hash_new_map, hash_full_pc(:, 1));
end

% assume grid is at worst case (0, 2000) per dim %
function [hash_pc] = get_hashes(grid)
    x = grid(:, 1);
    y = grid(:, 2);
    z = grid(:, 3);
    
    hash_pc = x + (2000 * y) + ((2000^2) * z);
end

