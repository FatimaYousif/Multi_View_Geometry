function pts_2d_n = add_noise(num_points, pts_2d)
    mean_noise = 0;
    std_deviation = 0.95; 
    
    % Generate Gaussian noise using normrnd
    noise = normrnd(mean_noise, std_deviation, size(pts_2d));
    
    % Ensure that 95% of points are in the range [-1, +1] pixels
    num_points_95 = round(0.95 * num_points);
    noise(1:num_points_95, :) = noise(1:num_points_95, :) * 1; % Adjust scale
    
    pts_2d_n = pts_2d + noise;
end