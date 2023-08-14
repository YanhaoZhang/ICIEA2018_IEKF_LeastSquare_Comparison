function [] = Draw_LeastSquareEstimate_3d( Graph )
%Draw_LeastSquareEstimate_2d: draw least square estimation result of 2D case
%   This is the main function of this file. 
%   Input is estimated Graph.

if nargin < 1
    load('./LeastSquare_3d_EstimationResult.mat');   % two dot means the previous folder, while one dot means the current folder
    Graph = LeastSquare_EstimationResult;
end

Pose_Estimation = Graph.Nodes.Pose3.Values;     %estimation of [R p]
Pose_Covariance = Graph.Nodes.Pose3.Covariance; %estimation of pose covariance
Land_Estimation = Graph.Nodes.Landmark3.Values;
Land_Covarinace = Graph.Nodes.Landmark3.Covariance;

drawPoseEstimation_3d(Pose_Estimation);         %draw pose estimation
drawLandmarkEstimation_3d(Land_Estimation);     %draw landmark estimation
drawCovPose_3d(Pose_Estimation,     Pose_Covariance); %draw position 3-sigma circle
drawCovLandmark_3d(Land_Estimation, Land_Covarinace); %draw landmark 3-sigma circle
end

%%
function [] = drawPoseEstimation_3d(pose)
%drawPoseEstimation_2d: draw least square estimation of pose.
%   orientation of each pose is shown by local frame
PoseName = fields(pose);       % the name of all poses
num_pose = size( PoseName,1 );

position = [];
for i=1:num_pose
    pose_i = pose.( PoseName{i} ); %[R p] of current pose
    position_i = pose_i(1:3,4);    %position of current pose
    position = [position position_i];
end
% draw estimated position
%figure;
plot3( position(1,:), position(2, :), position(3,:),'k' ); hold on;
scatter3( position(1,:), position(2, :), position(3,:), 'ro' ); hold on;
scatter3(position(1,:), position(2,:), position(3,:), [],'k' ,'filled'); hold on; % for recognizing
% draw local frame, which represents estimated orientation
drawAxisOfLeastSquareEstimate_3d(pose);
end

%%
function [] = drawAxisOfLeastSquareEstimate_3d(pose)
%DrawAxisOfLeastSquareEstimate_2d: draw the axis of 2D least square estimation result
%	input pose [R p]

PoseName = fields(pose);       % the name of all poses
num_pose = size( PoseName,1 );

axisl = 5;                     % length of local frame's axis
for i = 1:num_pose
    pose_i = pose.( PoseName{i} ); %[R p] of current pose
    rotationI = pose_i(1:3,1:3);   %position of current pose
    position_i = pose_i(1:3,4);
    xdir = position_i+axisl*rotationI(:, 1);
    xdir = [position_i xdir];
    ydir = position_i+axisl*rotationI(:, 2);
    ydir = [position_i ydir];
    zdir = position_i+axisl*rotationI(:, 3);
    zdir = [position_i zdir];
    plot3(xdir(1,:), xdir(2,:), xdir(3,:), 'Color', [255,128,0]/255,'LineStyle','--'); hold on;
    plot3(ydir(1,:), ydir(2,:), ydir(3,:),'Color', [128,0,255]/255,'LineStyle','--'); hold on;
    plot3(zdir(1,:), zdir(2,:), zdir(3,:),'Color', [0,204,102]/255,'LineStyle','--'); hold on;
end
end

%%
function [] = drawLandmarkEstimation_3d(landmark)
%drawLandmarkEstimation_2d: draw least square estimation of landmark.
%   

LandmarkName = fields(landmark); % the name of all landmarks
num_land = size(LandmarkName,1); % number of landmark

Landmark_es = [];                % estimation of landmark
for i=1:num_land
    landmark_i = landmark.(LandmarkName{i});
    Landmark_es = [Landmark_es landmark_i];
end
scatter3(Landmark_es(1,:), Landmark_es(2,:),Landmark_es(3,:),[],'g'); hold on;
scatter3(Landmark_es(1,:), Landmark_es(2,:),Landmark_es(3,:),[],'k' ,'filled'); hold on;
end

%%
function [] = drawCovPose_3d(pose,covariance)
%drawCovPose_2d: Draw the 3sigma of pose according to the covariance matrix of each pose
%	input: pose represents [R p] of each pose. covariance represents cov of each pose. They are all structure

PoseName = fields(pose);       % the name of all poses
num_pose = size( PoseName,1 );
for i=1:num_pose
    pose_i = pose.( PoseName{i} ); %[R p] of current pose
    position_i = pose_i(1:3,4);    %position of current pose
    cov_poseI = covariance.( PoseName{i} ); % cov of poseI 3*3
    cov_positionI = cov_poseI(4:6,4:6);     % the right bottom of cov
    
%     CV=GetCov_3d(cov_positionI,position_i(1),position_i(2));
%     plot(CV(1,:),CV(2,:),'k--'); hold on;
    
    CV=GetCov_3d(cov_positionI,position_i(1),position_i(2),position_i(3));
    surf(CV.x,CV.y,CV.z,'FaceColor','k','EdgeColor','none','FaceAlpha',0.1); hold on;
end
end

%%
function [] = drawCovLandmark_3d(landmark,covariance)
%drawCovLandmark_2d: Similar of drawCovPose_2d
%   

LandmarkName = fields(landmark); % the name of all landmarks
num_land = size(LandmarkName,1);

for i=1:num_land
    landmark_i = landmark.(LandmarkName{i}); % the coordinate of current landmark
    cov_landmarkI = covariance.(LandmarkName{i});
    
%     CV=GetCov_2d(cov_landmarkI,landmark_i(1),landmark_i(2));
%     plot(CV(1,:),CV(2,:),'k--'); hold on;
    
    CV=GetCov_3d(cov_landmarkI,landmark_i(1),landmark_i(2),landmark_i(3));
    surf(CV.x,CV.y,CV.z,'FaceColor','k','EdgeColor','none','FaceAlpha',0.1); hold on;
    
end
end
