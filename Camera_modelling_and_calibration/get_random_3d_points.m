% same as step 3

function points_3d = get_random_3d_points(num_points)
    random_points = (480 + 480) * rand(num_points, 3) - 480;
    points_3d = [random_points, ones(num_points, 1)];
end
