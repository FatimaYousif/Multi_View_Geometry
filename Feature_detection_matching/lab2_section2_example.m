% Helper script for lab 2 of MVG section 2
% ctrl + r = comment
% ctrl + t = un-comment

clear all;
close all;
clc;

% image1filename = 'imgl01366.jpg';
% image2filename = 'imgl01386.jpg';

% image1filename = 'imgl01386.jpg';
% image2filename = 'imgl01451.jpg';
% 
% image1filename = 'imgl01366.jpg';
% image2filename = 'imgl01451.jpg';
% 
image1filename = 'imgl01311.jpg';
image2filename = 'imgl01396.jpg';
H12 = [0.923963 0.105144 -41.64614; -0.105144 0.923963 -224.7121;0 0 1];

% 
% image1filename = 'imgl01311.jpg';
% image2filename = 'imgl01386.jpg';
% 
% image1filename = 'imgl01366.jpg';
% image2filename = 'imgl01396.jpg';
% 
%image1filename = 'IMG_1253_small.JPG';
%image2filename = 'IMG_1254_small.JPG';
%
% H12 = [0.991966 0.135529 -85.4565; -0.135529 0.991966 -372.3004;0 0 1];

%image1filename = 'Img00001_small.jpg';
%image2filename = 'Img00025_small.jpg';

% 
% image1filename = 'Img00025_small.jpg';
% image2filename = 'Img00320_small.jpg';

[image1, descriptors1, loc1] = siftprecomputed(image1filename);
%showkeys(image1, loc1)

[image2, descriptors2, loc2] = siftprecomputed(image2filename);
%showkeys(image2, loc2)

% ---------------------------------------
% ------------STEP 1---------------------
[CL1uv,CL2uv] = matchsiftmodif(image1filename, image2filename, 0.4, true);

% [CL1uv,CL2uv] = matchsiftmodif(image1filename, image2filename, 0.6, true);

% [CL1uv,CL2uv] = matchsiftmodif(image1filename, image2filename, 0.8, true);

disp(CL1uv)
disp(CL2uv)

% ------------STEP 2---------------------
% 
% errorVec = projectionerrorvec(H12,CL1uv,CL2uv);
% disp(errorVec)


%-----------------------------------------

% I1 = imread(image1filename);
% I2 = imread(image2filename);
% 
% % Do feature association with the modified match.m function
% distRatio = 0.8;
% drawMatches = true;
% [CL1uv,CL2uv,loc1,des1,loc2,des2] = matchsiftmodif(image1filename, image2filename, distRatio, drawMatches);
% 
% 
% % This is an alternative way of showing the associations, using a toolbox function
% figure; showMatchedFeatures(I1,I2,CL1uv,CL2uv,'Montage');
% 
% 
% drawMatches = false;
% distThreshold = 50;
% 
% % Initialize stacks for plotting. 
% % This can be implemented more elegantly using matlab's table structure
% distRatioStack = [];
% avgRepErrorStack = [];
% maxRepErrorStack = [];
% numPointsStack = [];
% numPointsOverThresholdStack = []; 
% 
% for distRatio = 0.3:0.05:0.9
%     [CL1uv,CL2uv,loc1,des1,loc2,des2] = matchsiftmodif(image1filename, image2filename, distRatio, drawMatches);
% 
%     errorVec = projectionerrorvec(H12,CL1uv,CL2uv);
% 
% 
%     if isempty(errorVec)
%         disp('No features were associated');
%     else
%         distRatioStack = [distRatioStack; distRatio];
%         avgRepErrorStack = [avgRepErrorStack; mean(errorVec)];
%         maxRepErrorStack = [maxRepErrorStack; max(errorVec)];
%         numPointsStack = [numPointsStack; size(CL1uv,1)];
%         numPointsOverThresholdStack = [numPointsOverThresholdStack; sum(errorVec > distThreshold)];
%     end;
% 
% end;
% 
% 
% % Draw two versions of the same plot, one with linear y-axis and the other
% % with log y-axis, to better see the small vs large values
% figure;
% subplot(1,2,1); 
% plot(distRatioStack,avgRepErrorStack,distRatioStack,maxRepErrorStack,distRatioStack,numPointsStack,distRatioStack,numPointsOverThresholdStack);
% xlabel('Distance Ratio');
% ylabel('Error and Num Matches (linear scale)');
% legend('Avg Rep Error','Max Rep Error','Num Matches','Num matches > dist threshold');
% 
% subplot(1,2,2); 
% semilogy(distRatioStack,avgRepErrorStack,distRatioStack,maxRepErrorStack,distRatioStack,numPointsStack,distRatioStack,numPointsOverThresholdStack);
% xlabel('Distance Ratio');
% ylabel('Error and Num Matches (log scale)');
% legend('Avg Rep Error','Max Rep Error','Num Matches','Num matches > dist threshold');
% 



