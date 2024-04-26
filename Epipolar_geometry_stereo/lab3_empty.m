% This a empty script to help you move faster on you lab work.
clear all;
close all;
clc;


%% Step 1
% Camera 1
gamma= 0;
au1 = 100; av1 = 120; uo1 = 128; vo1 = 128;
imageSize = [256 256];

%% Step 2
% Camera 2?

au2 = 90; av2 = 110; uo2 = 128; vo2 = 128; 
ax = 0.1; by = pi/4; cz = 0.2; % XYZ EULER 
tx = -1000; ty = 190; tz = 230; 

Rx = [1 0 0; 0 cos(ax) -sin(ax); 0 sin(ax) cos(ax)];
Ry = [cos(by) 0 sin(by); 0 1 0; -sin(by) 0 cos(by)];
Rz = [cos(cz) -sin(cz) 0; sin(cz) cos(cz) 0; 0 0 1 ];

%% STEP 3
% Compute intrinsic matrices and projection matrices

K1 = [au1 0 uo1; 0 av1 vo1; 0 0 1]; % Intrisics matrix for camera 1
wR1c = eye(3);   % identity = rotation of camera 1, from the camera to the world coordinate frame
wt1c = [0 0 0]'; % translation of camera 1, from the camera to the world coordinate frame

% Note: ************** You have to add your own code from here onward ************

K2 = [au2 0 uo2; 0 av2 vo2; 0 0 1];
wR2c =  Rx * Ry * Rz;
wt2c = [tx ty tz]';

P1 = K1 * [wR1c' -wR1c' * wt1c];

P2 = K2 * [wR2c' -wR2c' * wt2c]; % You have to replace this. This is just to serve as input for the drawing functions

% disp(P1)
% disp(P2)

%% STEP 4

% Attention: This is an invented matrix just to have some input for the drawing
% functions. You have to compute it properly 

% F = [3e-05 7e-05 -0.006;2e-05 -2.e-05 0.01;-0.009 -0.01 1];

% slide 28
t_x=[0 -tz ty; tz 0 -tx; -ty tx 0];

% lab guide STEP 4
% sure = wR2c = ?
F= inv(K2)' * wR2c' * t_x * inv(K1);

fprintf('Step 4:\n\ F:\n');
disp(F);

%% STEP 5
V(:,1) = [100;-400;2000];
V(:,2) = [300;-400;3000];
V(:,3) = [500;-400;4000];
V(:,4) = [700;-400;2000];
V(:,5) = [900;-400;3000];
V(:,6) = [100;-50;4000];
V(:,7) = [300;-50;2000];
V(:,8) = [500;-50;3000];
V(:,9) = [700;-50;4000];
V(:,10) = [900;-50;2000];
V(:,11) = [100;50;3000];
V(:,12) = [300;50;4000];
V(:,13) = [500;50;2000];
V(:,14) = [700;50;3000];
V(:,15) = [900;50;4000];
V(:,16) = [100;400;2000];
V(:,17) = [300;400;3000];
V(:,18) = [500;400;4000];
V(:,19) = [700;400;2000];
V(:,20) = [900;400;3000];

%% STEP 6
% Compute the pairs of image points in both image planes by using the camera projection matrices of step 3.
% Projection on image planes
cam1_p2d = mvg_projectPointToImagePlane(V,P1);
cam2_p2d = mvg_projectPointToImagePlane(V,P2);

%% STEP 7
% example of the plotting functions 
% Draw 2D projections on image planes
% Open two windows in matlab, which will be used as both image planes, and draw the 2D points obtained in step 6.
cam1_fig = mvg_show_projected_points(cam1_p2d(1:2,:),imageSize,'F -Projected points on image plane 1');
cam2_fig = mvg_show_projected_points(cam2_p2d(1:2,:),imageSize,'F - Projected points on image plane 2');

% Draw epipolar lines
[~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(cam1_p2d,cam2_p2d,F);
[cam1_fig,cam2_fig] = mvg_show_epipolar_lines(cam1_fig, cam2_fig, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'b');

% Draw epipoles
% TODO 
% ep_1 = [-300 200 1];    % These are invented values just for illustrating. You have to compute them
% ep_2 = [200 50 1];      % These are invented values just for illustrating. You have to compute them
% [~,~] = mvg_show_epipoles(cam1_fig, cam2_fig,ep_1,ep_2);

% COMPUTE EPIPOLES

% Compute the SVD of the fundamental matrix F
[U, S, V] = svd(F);

% Extract the left nullspace (epipole in image 1)
ep_1 = V(:, end);

% Extract the right nullspace (epipole in image 2)
ep_2 = U(:, end);

% Normalize the epipoles
ep_1 = ep_1 / ep_1(3);
ep_2 = ep_2 / ep_2(3);

% Display the computed epipoles
fprintf('F - Epipole 1:\n');
disp(ep_1');
fprintf('F - Epipole 2:\n');
disp(ep_2');

% Draw epipoles
[~,~] = mvg_show_epipoles(cam1_fig, cam2_fig, ep_1', ep_2');

%% STEP 8
% Compute the fundamental matrix with the 8-point method and SVD using of the 2D points obtained in step 6. 
% slide 43 lecture: 6
% YT VID = COMPLETE ALGO

 % U_n = points;
 % [U, D, V] = svd(U_n);
 % F = reshape(V(:,9), 3,3)'; 
 % [U, D, V] = svd(F)
 % F=  U*diag([D(1,1) D(2,2) 0])*V';

% npts = size(cam2_p2d, 2);
npts=length(cam2_p2d);
U_n = [cam2_p2d(1, :)' .* cam1_p2d(1, :)', cam2_p2d(1, :)' .* cam1_p2d(2, :)', cam2_p2d(1, :)', ...
       cam2_p2d(2, :)' .* cam1_p2d(1, :)', cam2_p2d(2, :)' .* cam1_p2d(2, :)', cam2_p2d(2, :)', ...
       cam1_p2d(1, :)', cam1_p2d(2, :)', ones(npts, 1)];

[U, D, V] = svd(U_n);
F_matrix_8point = reshape(V(:, 9), 3, 3)';   

[U,D,V] = svd(F_matrix_8point);
F_matrix_8point = U*diag([D(1,1) D(2,2) 0])*V';

fprintf('Step 8:\n\tFundamental matrix using 8-point algorithm:\n');
disp(F_matrix_8point);

%% STEP 9
% Compare the step 8 matrix with the one obtained in step 4. To obtain a value that is representative of the difference, you can use the sum of absolute differences or any other indicator that you consider adequate.
difference = sum(abs(F - F_matrix_8point), 'all');

fprintf('Step 9:\n\tDifference in F and F_matrix_8point:\n');
disp(difference);

%% STEP 10
% Draw in the windows of step 7 all the epipolar geometry, i.e., epipoles and epipolar lines by using the matrix obtained in step 8. You can represent the location of the epipoles with a cross ( + ); Enlarge the windows if necessary to view the epipoles properly. 
% STEP 7 = F_matrix_8point
[~, ~, c1_l_coeff, c2_l_coeff] = mvg_compute_epipolar_geom_modif(cam1_p2d, cam2_p2d, F_matrix_8point);
[cam1_fig, cam2_fig] = mvg_show_epipolar_lines(cam1_fig, cam2_fig, c1_l_coeff, c2_l_coeff, [-400, 1; 300, 400], 'r');

%% STEP 11
% Add zero-mean Gaussian noise to the 2D points such that 95% of that noise will be the range [−1, +1] pixels.
noise_range = 1;
noise = noise_range * randn(size(cam1_p2d));
cam1_p2d_noisy = cam1_p2d + noise;
cam2_p2d_noisy = cam2_p2d + noise;

% Add zero-mean Gaussian noise to the 2D points such that 95% of that noise will be the range [-1, +1] pixels.
noise_range = 1;
sigma = noise_range / 2;  % Standard deviation for 95% range
noise = sigma * randn(size(cam1_p2d));  % Generate Gaussian noise
cam1_p2d_noisy = cam1_p2d + noise;
cam2_p2d_noisy = cam2_p2d + noise;


%% STEP 12
% Repeat step 8 up to 10 with the noisy 2D points. Compare the epipolar geometry obtained.

% Step 8 with noisy points
U_n_noisy = [cam2_p2d_noisy(1, :)' .* cam1_p2d_noisy(1, :)', cam2_p2d_noisy(1, :)' .* cam1_p2d_noisy(2, :)', cam2_p2d_noisy(1, :)', ...
       cam2_p2d_noisy(2, :)' .* cam1_p2d_noisy(1, :)', cam2_p2d_noisy(2, :)' .* cam1_p2d_noisy(2, :)', cam2_p2d_noisy(2, :)', ...
       cam1_p2d_noisy(1, :)', cam1_p2d_noisy(2, :)', ones(npts, 1)];

[U_noisy, D, V_noisy] = svd(U_n_noisy);
F_matrix_noisy = reshape(V_noisy(:, 9), 3, 3);

[U,D,V] = svd(F_matrix_noisy);
F_matrix_noisy = U*diag([D(1,1) D(2,2) 0])*V';

fprintf('Step 12:\n\tFundamental matrix using 8-point algorithm with noisy points:\n');
disp(F_matrix_noisy);

% Compare the noisy fundamental matrix with the original one
% (F_matrix_8point) = SAD = sum of absolute differences
difference_noisy = sum(abs(F_matrix_noisy - F_matrix_8point), 'all');
fprintf('\tDifference between noisy and original fundamental matrices: %.4f\n', difference_noisy);

% PLOTTING #########################

cam1_fig_n_1 = mvg_show_projected_points(cam1_p2d_noisy(1:2,:),imageSize,'NOISE-1: Projected points on image plane 1');
cam2_fig_n_1 = mvg_show_projected_points(cam2_p2d_noisy(1:2,:),imageSize,'NOISE-1: Projected points on image plane 2');

% Draw epipolar lines
[~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(cam1_p2d_noisy,cam2_p2d_noisy,F_matrix_noisy);
[cam1_fig_n_1,cam2_fig_n_1] = mvg_show_epipolar_lines(cam1_fig_n_1, cam2_fig_n_1, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'b');

% Draw epipoles
% COMPUTE EPIPOLES

% Compute the SVD of the fundamental matrix F
[U, S, V] = svd(F_matrix_noisy);

% Extract the left nullspace (epipole in image 1)
ep_1 = V(:, end);

% Extract the right nullspace (epipole in image 2)
ep_2 = U(:, end);

% Normalize/homog the epipoles
ep_1 = ep_1 / ep_1(3);
ep_2 = ep_2 / ep_2(3);

% Display the computed epipoles
fprintf('NOISE-1 = Epipole 1:\n');
disp(ep_1');
fprintf('NOISE-1 = Epipole 2:\n');
disp(ep_2');

% Draw epipoles
[~,~] = mvg_show_epipoles(cam1_fig_n_1, cam2_fig_n_1, ep_1', ep_2');

%% STEP 13
% Increase the Gaussian noise of step 11 (now in the range [−2, +2] for 95% of points) and repeat steps 8-12.
noise_range_large = 2;
noise_large = noise_range_large * randn(size(cam1_p2d));
cam1_p2d_noisy_large = cam1_p2d + noise_large;
cam2_p2d_noisy_large = cam2_p2d + noise_large;

% Step 8 with larger noisy points
U_n_noisy_large = [cam2_p2d_noisy_large(1, :)' .* cam1_p2d_noisy_large(1, :)', cam2_p2d_noisy_large(1, :)' .* cam1_p2d_noisy_large(2, :)', cam2_p2d_noisy_large(1, :)', ...
       cam2_p2d_noisy_large(2, :)' .* cam1_p2d_noisy_large(1, :)', cam2_p2d_noisy_large(2, :)' .* cam1_p2d_noisy_large(2, :)', cam2_p2d_noisy_large(2, :)', ...
       cam1_p2d_noisy_large(1, :)', cam1_p2d_noisy_large(2, :)', ones(npts, 1)];

[U_noisy_large, D, V_noisy_large] = svd(U_n_noisy_large);
F_matrix_noisy_large = reshape(V_noisy_large(:, 9), 3, 3);

[U,D,V] = svd(F_matrix_noisy_large);
F_matrix_noisy_large = U*diag([D(1,1) D(2,2) 0])*V';

fprintf('Step 13:\n\tFundamental matrix using 8-point algorithm with larger noisy points:\n');
disp(F_matrix_noisy_large);

% Compare the larger noisy fundamental matrix with the original one (F_matrix_8point)
difference_noisy_large = sum(abs(F_matrix_noisy_large - F_matrix_8point), 'all');
fprintf('\tDifference between larger noisy and original fundamental matrices: %.4f\n', difference_noisy_large);

% PLOTTING #########################

cam1_fig_n_2 = mvg_show_projected_points(cam1_p2d_noisy_large(1:2,:),imageSize,'NOISE-2: Projected points on image plane 1');
cam2_fig_n_2 = mvg_show_projected_points(cam2_p2d_noisy_large(1:2,:),imageSize,'NOISE-2: Projected points on image plane 2');

% Draw epipolar lines
[~,~,c1_l_coeff,c2_l_coeff] = mvg_compute_epipolar_geom_modif(cam1_p2d_noisy_large,cam2_p2d_noisy_large,F_matrix_noisy_large);
[cam1_fig_n_2,cam2_fig_n_2] = mvg_show_epipolar_lines(cam1_fig_n_2, cam2_fig_n_2, c1_l_coeff,c2_l_coeff, [-400,1;300,400],'b');

% Draw epipoles
% COMPUTE EPIPOLES

% Compute the SVD of the fundamental matrix F
[U, S, V] = svd(F_matrix_noisy_large);

% Extract the left nullspace (epipole in image 1)
ep_1 = V(:, end);

% Extract the right nullspace (epipole in image 2)
ep_2 = U(:, end);

% Normalize/homog the epipoles
ep_1 = ep_1 / ep_1(3);
ep_2 = ep_2 / ep_2(3);

% Display the computed epipoles
fprintf('NOISE-2 = Epipole 1:\n');
disp(ep_1');
fprintf('NOISE-2 = Epipole 2:\n');
disp(ep_2');

% Draw epipoles
[~,~] = mvg_show_epipoles(cam1_fig_n_2, cam2_fig_n_2, ep_1', ep_2');

%% STEP 14
% % Implement the coordinate normalization method described in the classes slides. 
% % The transformations (T and T' in the slides) are homographies that modify the original projected points in each image so that they become centered at the origin and have standard deviation equal to 1. 
% % Repeat steps 8-12 with the Gaussian noise in the range [−2, +2] for 95% of points.

% Implement coordinate normalization using homographies

% Apply normalization to original points
xn1 = cam1_p2d(1:2, :);
N1 = size(xn1, 2);
t1 = (1/N1) * sum(xn1, 2);  % centroid of the points
xnc1 = xn1 - t1 * ones(1, N1);  % center the points
dc1 = sqrt(sum(xnc1.^2));  % distance of each new point to (0,0); dc is 1xN vector
davg1 = (1/N1) * sum(dc1);
s1 = sqrt(2) / davg1;
T1 = [s1 * eye(2), -s1 * t1; 0 0 1];  % scale factor, so that avg distance is sqrt(2)
p1s = T1 * [cam1_p2d; ones(1, size(cam1_p2d, 2))];

xn2 = cam2_p2d(1:2, :);
N2 = size(xn2, 2);
t2 = (1/N2) * sum(xn2, 2);  % centroid of the points
xnc2 = xn2 - t2 * ones(1, N2);  % center the points
dc2 = sqrt(sum(xnc2.^2));  % distance of each new point to (0,0); dc is 1xN vector
davg2 = (1/N2) * sum(dc2);
s2 = sqrt(2) / davg2;
T2 = [s2 * eye(2), -s2 * t2; 0 0 1];  % scale factor, so that avg distance is sqrt(2)
p2s = T2 * [cam2_p2d; ones(1, size(cam2_p2d, 2))];

%% Repeat steps 8-12 with the normalized points

% Step 8 with normalized points
U_n_normalized = [p2s(1, :)' .* p1s(1, :)', p2s(1, :)' .* p1s(2, :)', p2s(1, :)', ...
       p2s(2, :)' .* p1s(1, :)', p2s(2, :)' .* p1s(2, :)', p2s(2, :)', ...
       p1s(1, :)', p1s(2, :)', ones(npts, 1)];

[U_normalized, D, V_normalized] = svd(U_n_normalized);
F_matrix_normalized = reshape(V_normalized(:, 9), 3, 3);

[U, D, V] = svd(F_matrix_normalized);
% D(3, 3) = 0;  % Enforce rank-2
F_matrix_normalized = U * diag([1 1 0]) * V';

% undo scaling
F_matrix_normalized = T1' * F_matrix_normalized * T2;

fprintf('Step 14:\n\tFundamental matrix using 8-point algorithm with normalized points:\n');
disp(F_matrix_normalized);

% Compare the normalized fundamental matrix with the original one
difference_normalized = sum(abs(F_matrix_normalized - F_matrix_8point), 'all');
fprintf('\tDifference between normalized and original fundamental matrices: %.4f\n', difference_normalized);

% PLOTTING #########################

cam1_fig_n_normalized = mvg_show_projected_points(p1s(1:2, :), imageSize, 'Normalized : Projected points on image plane 1');
cam2_fig_n_normalized = mvg_show_projected_points(p2s(1:2, :), imageSize, 'Normalized : Projected points on image plane 2');

% Draw epipolar lines
[~, ~, c1_l_coeff, c2_l_coeff] = mvg_compute_epipolar_geom_modif(p1s, p2s, F_matrix_normalized);
[cam1_fig_n_normalized, cam2_fig_n_normalized] = mvg_show_epipolar_lines(cam1_fig_n_normalized, cam2_fig_n_normalized, c1_l_coeff, c2_l_coeff, [-400, 1; 300, 400], 'b');

% Draw epipoles
% COMPUTE EPIPOLES
[U, S, V] = svd(F_matrix_normalized);

% Extract the left nullspace (epipole in image 1)
ep_1_normalized = V(:, end);

% Extract the right nullspace (epipole in image 2)
ep_2_normalized = U(:, end);

% Normalize/homog the epipoles
ep_1_normalized = ep_1_normalized / ep_1_normalized(3);
ep_2_normalized = ep_2_normalized / ep_2_normalized(3);

fprintf('Normalized = Epipole 1:\n');
disp(ep_1_normalized');
fprintf('Normalized = Epipole 2:\n');
disp(ep_2_normalized');

% condition number
fprintf('condition number -without coordinate normalization = :\n');
disp(cond(U_n));
fprintf('condition number - coordinate normalization = :\n');
disp(cond(U_n_normalized));

% Draw epipoles
[~, ~] = mvg_show_epipoles(cam1_fig_n_normalized, cam2_fig_n_normalized, ep_1_normalized', ep_2_normalized');
return;
