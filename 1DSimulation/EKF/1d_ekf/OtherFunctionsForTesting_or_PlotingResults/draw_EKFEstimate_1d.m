function [] = draw_EKFEstimate_1d(estimateResult)
%draw_EKFEstimate_1d: This is the main function, which draws the normal EKF estimation result in 1D case.
%   using four subfunctions. And we draw the estimation in the line y=1.5, not y=0

if nargin < 1
    load('./EKF_1d_estimation_results.mat');   % two dot means the previous folder, while one dot means the current folder
    estimateResult = estimation_results;
end

addpath('../1d_datagen/');

draw_EKFPoseEstimation_1d(estimateResult);
draw_EKFCovPose_1d(estimateResult);

draw_EKFLandmarkEstimation_1d(estimateResult);
draw_EKFCovLandmark_1d(estimateResult);

ylim([-9 9]);hold on;         % the range of the figure
end

%%
function [] = draw_EKFPoseEstimation_1d(estimateResult)
%draw_EKFPoseEstimation_1d: draw the pose estimation. 1D: just has position.
%   draw the plots in the line y=1.5, not y=0

num_state = size(estimateResult,2);  % the number of state
position = [];
for i=1:num_state
    positionI = estimateResult{i}.position;
    position = [position positionI];
end
plot( position, 1.5, 'ro' ); hold on;

% axisl = 5;
% for i = 1:num_state
%     rotationi = [1; 0];                     % the local x-axis. In 1D case, it just has one axis
%     positioni = [position(:, i) 1.5]';
%     xdir = positioni + axisl*rotationi;
%     xdir = [positioni, xdir];
%     plot(xdir(1,:), xdir(2,:), 'Color', 'red','LineStyle','--'); hold on;
% end
end

%%
function [] = draw_EKFCovPose_1d(estimateResult)
%draw_EKFCovPose_1d: draw cov of pose estimation.
%   Here, since we get mu and cov, we can also draw the gaussian distribution of each pose
%   draw the plots in the line y=1.5, not y=0

num_pose = size(estimateResult,2);

for i=1:num_pose
    positionI = estimateResult{i}.position;
    cov_positionI = estimateResult{i}.cov(1,1);
    CV=GetCov_1d(cov_positionI,positionI);
    plot(CV(1,:),1.5,'r.'); hold on;            %Since 1D, we can only use plot 'ro' or 'r.' to plot dots, not 'r-' to plot lines
end
end

%%
function [] = draw_EKFLandmarkEstimation_1d(estimateResult)
%draw_EKFLandmarkEstimation_1d: Draw estimation of 1D-landmarks by normal EKF
%   draw the plots in the line y=1.5, not y=0

num_state = size(estimateResult,2);               %number of state: equal to the number of poses
landmarks = estimateResult{num_state}.landmarks;  % the estimation of the final state
plot(landmarks(:,2), 1.5 ,'go'); hold on;              
end

%%
function [] = draw_EKFCovLandmark_1d(estimateResult)
%draw_EKFCovLandmark_1d: Draw covariance of EKF estimation of 1D-landmarks.
%   draw the plots in the line y=1.5, not y=0

num_state = size(estimateResult,2);                %the number of states (equales the number of poses)
landmarks = estimateResult{num_state}.landmarks;

stateCov = estimateResult{num_state}.cov;          % the cov of last state
num_cov = size(stateCov,1);                        % size of stateCov: num_cov*num_cov
land_cov = stateCov(2:num_cov,2:num_cov);          % the cov of landmark. The top-left one is sigma of position, so use the bottom right block
num_land = size(landmarks,1);                      % the number of landmark
for i=1:num_land
    landmarkI = landmarks(i,2:2);                  % the 2nd column is the coordinate of landmark
    cov_landmarkI = land_cov(1*i-0:1*i,1*i-0:1*i);
    CV=GetCov_1d(cov_landmarkI,landmarkI);
    plot(CV(1,:),1.5,'g.'); hold on;
end

% k=1;
% for i=4:2:(num_cov-1)
%     landmarkI = landmarks(k,1:2); k=k+1;
%     cov_landmarkI = stateCov(i:i+1,i:i+1);
%     CV=GetCov(cov_landmarkI,landmarkI(1),landmarkI(2));
%     plot(CV(1,:),CV(2,:),'g--'); hold on;
% end
end