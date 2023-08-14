function [] = Draw_LeastSquareEstimate_2d( Graph )
%Draw_LeastSquareEstimate_2d: draw least square estimation result of 2D case
%   This is the main function of this file. 
%   Input is estimated Graph.

if nargin < 1
    load('./LeastSquare_2d_EstimationResult.mat');   % two dot means the previous folder, while one dot means the current folder
    Graph = LeastSquare_EstimationResult;
end

Pose_Estimation = Graph.Nodes.Pose2.Values;     %estimation of [R p]
Pose_Covariance = Graph.Nodes.Pose2.Covariance; %estimation of pose covariance
Land_Estimation = Graph.Nodes.Landmark2.Values;
Land_Covarinace = Graph.Nodes.Landmark2.Covariance;

drawPoseEstimation_2d(Pose_Estimation);         %draw pose estimation
drawLandmarkEstimation_2d(Land_Estimation);     %draw landmark estimation
drawCovPose_2d(Pose_Estimation,     Pose_Covariance); %draw position 3-sigma circle
drawCovLandmark_2d(Land_Estimation, Land_Covarinace); %draw landmark 3-sigma circle
end

%%
function [] = drawPoseEstimation_2d(pose)
%drawPoseEstimation_2d: draw least square estimation of pose.
%   orientation of each pose is shown by local frame
PoseName = fields(pose);       % the name of all poses
num_pose = size( PoseName,1 );

position = [];
for i=1:num_pose
    pose_i = pose.( PoseName{i} ); %[R p] of current pose
    position_i = pose_i(1:2,3);    %position of current pose
    position = [position position_i];
end
% draw estimated position
%figure;
plot( position(1,:), position(2, :), 'k' ); hold on;
plot( position(1,:), position(2, :), 'ro' ); hold on;
% scatter(position(1,:), position(2,:),[],'r' ,'filled'); hold on; % for recognizing
% draw local frame, which represents estimated orientation
% drawAxisOfLeastSquareEstimate_2d(pose);
end

%%
function [] = drawAxisOfLeastSquareEstimate_2d(pose)
%DrawAxisOfLeastSquareEstimate_2d: draw the axis of 2D least square estimation result
%	input pose [R p]

PoseName = fields(pose);       % the name of all poses
num_pose = size( PoseName,1 );

axisl = 5;                     % length of local frame's axis
for i = 1:num_pose
    pose_i = pose.( PoseName{i} ); %[R p] of current pose
    rotationI = pose_i(1:2,1:2);   %position of current pose
    position_i = pose_i(1:2,3);
    xdir = position_i+axisl*rotationI(:, 1);
    xdir = [position_i xdir];
    ydir = position_i+axisl*rotationI(:, 2);
    ydir = [position_i ydir];
    plot(xdir(1,:), xdir(2,:), 'Color', [255,128,0]/255,'LineStyle','--'); hold on;
    plot(ydir(1,:), ydir(2,:), 'Color', [127,0,255]/255,'LineStyle','--'); hold on;
end
end

%%
function [] = drawLandmarkEstimation_2d(landmark)
%drawLandmarkEstimation_2d: draw least square estimation of landmark.
%   

LandmarkName = fields(landmark); % the name of all landmarks
num_land = size(LandmarkName,1); % number of landmark

Landmark_es = [];                % estimation of landmark
for i=1:num_land
    landmark_i = landmark.(LandmarkName{i});
    Landmark_es = [Landmark_es landmark_i];
end
plot(Landmark_es(1,:), Landmark_es(2,:),'go'); hold on;
% scatter(Landmark_es(1,:), Landmark_es(2,:),[],'g' ,'filled'); hold on;
end

%%
function [] = drawCovPose_2d(pose,covariance)
%drawCovPose_2d: Draw the 3sigma of pose according to the covariance matrix of each pose
%	input: pose represents [R p] of each pose. covariance represents cov of each pose. They are all structure

PoseName = fields(pose);       % the name of all poses
num_pose = size( PoseName,1 );
for i=1:num_pose
    pose_i = pose.( PoseName{i} ); %[R p] of current pose
    position_i = pose_i(1:2,3);    %position of current pose
    cov_poseI = covariance.( PoseName{i} ); % cov of poseI 3*3
    cov_positionI = cov_poseI(2:3,2:3);     % the right bottom of cov
    
    CV=GetCov_2d(cov_positionI,position_i(1),position_i(2));
    plot(CV(1,:),CV(2,:),'k--'); hold on;
end
end

%%
function [] = drawCovLandmark_2d(landmark,covariance)
%drawCovLandmark_2d: Similar of drawCovPose_2d
%   

LandmarkName = fields(landmark); % the name of all landmarks
num_land = size(LandmarkName,1);

for i=1:num_land
    landmark_i = landmark.(LandmarkName{i}); % the coordinate of current landmark
    cov_landmarkI = covariance.(LandmarkName{i});
    
    CV=GetCov_2d(cov_landmarkI,landmark_i(1),landmark_i(2));
    plot(CV(1,:),CV(2,:),'k--'); hold on;
    
end
end
