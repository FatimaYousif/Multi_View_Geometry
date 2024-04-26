function errorVec = projectionerrorvec(H12,CL1uv,CL2uv)
% errorVec = projectionerrorvec(H12,CL1uv,CL2uv)
% 
% Given two list of coordinates (CL1uv and CL2uv) on two images and the
% homography that relates them (H12) this function will compute an error vector
% (errorVec).  
% 
% This vector will contain the Euclidean distance between each
% point in CL1uv and its corresponding point in CL2uv after applying the
% homography  H12.

% insert your code here

% -----------------------------------------
% 
% Apply homography to CL1uv
% CL1uv_homog = [CL1uv, ones(size(CL1uv, 1), 1)] * H12';
% 
% Normalize
% CL1uv_homog = CL1uv_homog(:, 1:2) ./ CL1uv_homog(:, 3);
% 
% Euclidean distances
% errorVec = sqrt(sum((CL1uv_homog - CL2uv).^2, 2));

%---------------------------------------------------

   %change the points to homogeneous coordinate points.
   CL1uv_h = [CL1uv, ones(size(CL1uv, 1), 1)]';
   CL2uv_h = [CL2uv, ones(size(CL2uv, 1), 1)]';
   CL11uv =  H12 * CL2uv_h;  
   CL11uv = CL11uv ./ CL11uv(3, :);  %normalize
   errorVec =  vecnorm(CL11uv - CL1uv_h);  %same as euclidean distance

% -------------------------------------------------------

% remove this line after creating your function
% errorVec = zeros(size(CL1uv,1),1); 
 