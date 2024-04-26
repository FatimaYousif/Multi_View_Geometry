function H12 = computeHomographyEmpty(CL1uv,CL2uv, Model)
%% computeHomography : estimate the Homography between two images according to Model 
%           Cluv1    set of points on image1 . Each row represents a 2-D point
%                       (u,v). Size: Nx2, with N number of points.
%           Cluv2    set of points on image2 . Each row represents a 2-D point
%                       (u,v). Size: Nx2, with N number of points.
%           Model       type of Homography to estimate. It has to be egual
%                       to one of the following strings: 'Translation',
%                       'Rigid', 'Similarity', 'Affine', 'Projective'.
%   Output
%           H12           estimated Homography of Model type. 3x3 matrix.
%


% warning('This is an empty function just to help you with the switch command.')
% 

num_points = size(CL1uv, 1);

switch (Model)

    case 'Translation'

        % Compute here your 'Translation' homography
        Q = zeros(2*num_points, 2);
        diff_vector = CL1uv - CL2uv;
        B = reshape(diff_vector', [], 1);

        % fill the B with translation distance from X-xu and Y-yu for
        % each point
        for i = 1 : num_points
            Q(2*i - 1, :)  = [1, 0];
            Q(2*i, :)  = [0, 1];
        end
        A = Q\B;
        H12 = [1, 0, A(1);
               0, 1, A(2);
               0, 0, 1];


    case 'Similarity'

        % Compute here your 'Similarity' homography
        %flatten the x, y value of the points in the second image.
        Q = zeros(2*num_points, 4);
        B = reshape(CL1uv', [], 1);
        for i = 1 : num_points
            u = CL2uv(i, 1);
            v = CL2uv(i, 2);
            Q(2*i - 1, :) = [u, -v, 1, 0];
            Q(2*i, :) =     [v, u, 0, 1];
        end
        A = Q\B;              % 3x3 matrix used to store the Homography
        H12 = [A(1), -A(2), A(3);
               A(2), A(1), A(4);
               0,       0,   1;];

    case 'Affine'
        % Compute here your 'Affine' homography
        %flatten the x, y value of the points in the second image.
        Q = zeros(2*num_points, 6);
        B = reshape(CL1uv', [], 1);
        for i =1 : num_points
            u = CL2uv(i, 1);
            v = CL2uv(i, 2);
            Q(2*i - 1, :) = [u, v, 1, 0, 0, 0];
            Q(2*i, :) = [0, 0, 0, u, v, 1];
        end
        A = Q\B;
        H12 = [A(1), A(2), A(3);
               A(4), A(5), A(6);
               0,    0,    1;];


    case 'Projective'

        % Compute here your 'Projective' homography
        Q = zeros(2*num_points, 8);
        B = reshape(CL1uv', [], 1);
        for i =  1: num_points
            u = CL2uv(i, 1);
            v = CL2uv(i, 2);
            X = CL1uv(i, 1);
            Y = CL1uv(i, 2);
            Q(2*i - 1, :) = [u, v, 1, 0, 0, 0, -u*X, -v*X];
            Q(2*i, :) = [0, 0, 0, u, v, 1, -u*Y, -v*Y];
        end
        A = Q\B;              % 3x3 matrix used to store the Homography
        H12 = [A(1), A(2), A(3);
               A(4), A(5), A(6);
               A(7), A(8), 1;];
    otherwise
        warning('Invalid model, returning identity homography');
        % H12 = eye(3);

end
end


