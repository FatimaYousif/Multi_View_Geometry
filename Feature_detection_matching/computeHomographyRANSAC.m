function H = computeHomographyRANSAC(cl1uv, cl2uv, model)
    
    % Setting threshold value
    thresh = 1;
    
    % Setting the stopping condition for RANSAC
    max_iterations = 5000;
    des_percent_inliers = 0.85;
    
    % Checking nos of cl1uv and cl2uv
    N = length(cl1uv);
    if length(cl1uv) ~= length(cl2uv)
        error('Length of cl1uv does not match length of cl2uv');
    end
    
    % Start of RANSAC algorithm
    if model == "Projective"
        num_correspond = 4;
    elseif model == "Translation"
        num_correspond = 2;
    elseif model == "Affine"
        num_correspond = 3;
    elseif model == "Similarity"
        num_correspond = 2;
    end
    
    best_inliers = [0];
    
    for i=1:max_iterations
        
        % Randomly select from cl1uv and cl2uv arrays
        selected_indices = randperm(N, num_correspond);
    
        % Format cl1uv and cl2uv arrays 
        cl1uv_RANSAC = cl1uv(selected_indices,:);
        cl2uv_RANSAC = cl2uv(selected_indices,:);
        
        % Compute H for selected features
        H = computeHomographyEmpty(cl1uv_RANSAC, cl2uv_RANSAC, model);
        
        % Calculate errors
        
        % Apply homography 
        Homogeneous_cl2uv = [cl2uv'; ones(1,length(cl2uv))];
        Transformed_cl2uv = H * Homogeneous_cl2uv;
        Transformed_cl2uv = Transformed_cl2uv(1:2,:)./Transformed_cl2uv(3,:);

        % Calculate errors
        Transformed_cl2uv = Transformed_cl2uv(1:2,:)';
        sq_diff = (cl1uv - Transformed_cl2uv).^2;
        sum_sq_diff = sum(sq_diff, 2);
        % Need to square threshold
        thresh_sq = thresh^2;
        
        % Check for inliers and outliers
        inliers = sum_sq_diff < thresh_sq;
        num_inliers = sum(inliers);
        percent_inliers = num_inliers/length(cl1uv);
        if percent_inliers > des_percent_inliers
            best_inliers = inliers;
            break;
        end
        
        if percent_inliers > (sum(best_inliers) / length(cl1uv))
            best_inliers = inliers;
        end   
    end
    
    display(strcat('Num. iterations: ', num2str(sum(i))))
    display(strcat('The highest number of inliers: ', num2str(sum(best_inliers))));

    % Compute final H
    % cl1uv_RANSAC = cl1uv(best_inliers,:);
    % cl2uv_RANSAC = cl2uv(best_inliers,:);
    % H = computeHomographyEmpty(cl1uv_RANSAC, cl2uv_RANSAC, model);
    % mean_error = mean(error(H, cl1uv_RANSAC, cl2uv_RANSAC));
end
