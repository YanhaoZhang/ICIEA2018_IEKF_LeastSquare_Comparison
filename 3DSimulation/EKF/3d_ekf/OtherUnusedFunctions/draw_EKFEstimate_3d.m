function [] = draw_EKFEstimate_3d(estimateResult)
%draw_EKFEstimate_3d: main function. To show the estimation result of normal EKF: left invariant EKF
%   

if nargin < 1
    load('./EKF_3d_estimation_results.mat');   % two dot means the previous folder, while one dot means the current folder
    estimateResult = estimation_results;
end

%addpath('../3d_datagen/');

% drawEstimatePose(estimateResult);
% drawCovPose(estimateResult);
% 
% drawEstimateLandmark(estimateResult);
% drawCovLandmark(estimateResult);

draw_EKFPoseEstimation_3d(estimateResult);
draw_EKFCovPose_3d(estimateResult);

draw_EKFLandmarkEstimation_3d(estimateResult);
draw_EKFCovLandmark_3d(estimateResult);
end

%%
function [] = draw_EKFPoseEstimation_3d(estimateResult)
%draw_EKFPoseEstimation_2d: Draw EKF estimation of pose, both position and orientation
%   

num_state = size(estimateResult,2);
position = [];
for i=1:num_state
    positionI = estimateResult{i}.position;
    position = [position positionI];
end
%figure;
plot3( position(1,:), position(2, :), position(3,:), 'r' ); hold on;
scatter3( position(1,:), position(2, :), position(3, :) ,'ro', 'filled'); hold on;

%draw axis
axisl = 5;
for i = 1:num_state
    rotationI = estimateResult{i}.orientation;
%    rotationi = poses.orientation((i-1)*2+1:i*2, :);
    xdir = position(:, i)+axisl*rotationI(:, 1);
    xdir = [position(:, i) xdir];
    ydir = position(:, i)+axisl*rotationI(:, 2);
    ydir = [position(:, i) ydir];
    zdir = position(:, i) + axisl*rotationI(:, 3);
    zdir = [position(:, i), zdir];
    plot3(xdir(1,:), xdir(2,:), xdir(3,:), 'Color', 'red','LineStyle','--'); hold on;
    plot3(ydir(1,:), ydir(2,:), ydir(3,:), 'Color', 'blue','LineStyle','--'); hold on;
    plot3(zdir(1,:), zdir(2,:), zdir(3,:), 'Color', 'green','LineStyle','--'); hold on;
end
end

%%
function [] = draw_EKFCovPose_3d(estimateResult)
%draw_EKFCovPose_2d: Draw covariance of EKF estimation of 2D-poses.
%   

num_pose = size(estimateResult,2);
for i=1:num_pose
    positionI = estimateResult{i}.position;
    
    %cov_thetaI = estimateResult{i}.cov(1,1);
    cov_positionI = estimateResult{i}.cov(4:6,4:6);
    CV=GetCov_3d(cov_positionI,positionI(1),positionI(2),positionI(3));
    surf(CV.x,CV.y,CV.z,'FaceColor','r','EdgeColor','none','FaceAlpha',0.1); hold on;
    %plot(CV(1,:),CV(2,:),'r--'); hold on;
end
end

%%
function [] = draw_EKFLandmarkEstimation_3d(estimateResult)
%draw_EKFLandmarkEstimation_2d: Draw estimation of 2D-landmarks by normal EKF
%   

num_state = size(estimateResult,2);
landmarks = estimateResult{num_state}.landmarks;
%num_land = size(landmarks,1);
scatter3(landmarks(:,2),landmarks(:,3),landmarks(:,4),'g','filled'); hold on;
end

%%
function [] = draw_EKFCovLandmark_3d(estimateResult)
%draw_EKFCovLandmark_2d: Draw covariance of EKF estimation of 2D-landmarks.
%   

num_state = size(estimateResult,2); %the number of states (equales the number of poses)
landmarks = estimateResult{num_state}.landmarks;

stateCov = estimateResult{num_state}.cov;          % the cov of last state
num_cov = size(stateCov,1);                        % size of stateCov: num_cov*num_cov
land_cov = stateCov(7:num_cov,7:num_cov);          % the cov of landmark. The top-left 3*3 block is cov of [R p], so use the bottom right block
num_land = size(landmarks,1);                      % the number of landmark
for i=1:num_land
    landmarkI = landmarks(i,2:4);                  % the 2nd-3th columns are the coordinates of landmark 
    cov_landmarkI = land_cov(3*i-2:3*i,3*i-2:3*i);
    CV=GetCov_3d(cov_landmarkI,landmarkI(1),landmarkI(2),landmarkI(3));
    surf(CV.x,CV.y,CV.z,'FaceColor','g','EdgeColor','none','FaceAlpha',0.1); hold on;
%     plot(CV(1,:),CV(2,:),'r--'); hold on;
end

% k=1;
% for i=4:2:(num_cov-1)
%     landmarkI = landmarks(k,1:2); k=k+1;
%     cov_landmarkI = stateCov(i:i+1,i:i+1);
%     CV=GetCov(cov_landmarkI,landmarkI(1),landmarkI(2));
%     plot(CV(1,:),CV(2,:),'g--'); hold on;
% end
end