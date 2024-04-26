% STEP 6

function P_matrix = get_projection_matrix(total_points, points_3d, points_2d)
    % Initialize the Q matrix
    Q = zeros(2 * total_points, 11);

    % Convert 2D points to (x,y) in cartesian form
    cartesian_points = points_2d(:, 1:2) ./ points_2d(:, 3);

    % Reshape cartesian_points for B
    B = reshape(cartesian_points', [2 * total_points, 1]);

    for i = 1:total_points
        Xi = points_3d(i, 1:3);  % (Xw, Yw, Zw) = 3D
        xi = points_2d(i, 1:2);  % (Xu, Yu) = 2D

        % Construct the A matrix
        Q(2 * i - 1, :) = [Xi, 1, zeros(1, 4), -Xi(1) * xi(1), -Xi(2) * xi(1), -Xi(3) * xi(1)];
        Q(2 * i, :) = [zeros(1, 4), Xi, 1, -Xi(1) * xi(2), -Xi(2) * xi(2), -Xi(3) * xi(2)];
    end

    % Estimate A matrix using the method of Hall
    A = pinv(Q) * B;

    % Add 1s to A in the end for the 4th column of the projection matrix
    A = [A; 1];

    % Reshape A into a 4x3 matrix and transpose to obtain the P matrix
    P_matrix = 1500 * reshape(A, [4, 3])';
end
