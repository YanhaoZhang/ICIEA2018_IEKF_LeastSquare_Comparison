function [LS_SingleTime_estimation_result] = LS_SLAM_1d(data,initialGuess)
%LeastSquare_SLAM_1d: Least square method in 1D case
%   data are generated in the folder /EKF-SLAM-SmallExample/datagen_1d/
%   the structure of this function is: initializeGraph, addEdges, addNodes, optimize, get covariance (sigma in 1D case), and show
%   the result.

PrintResult = false;
%% Load pre-given data: odometry and observations
if nargin < 1
%     load('../../EKF-SLAM-SmallExample/1d_datagen/data_1d.mat');   % two dot means the previous folder, while one dot means the current folder
%     load('./data_1d.mat'); 
%     load('../../1d_datagen/data_1d/data_1d_1.mat');
    load('./data_1d_1.mat');
    load('./NoiseDataInPose0Frame_1d_1.mat');
    initialGuess = NoiseDataInPose0Frame;
    PrintResult = true;
end
%% Add least square path
addpath('../Factor_1d/');
% addpath('../Factor/');
addpath('../Math/');
addpath('../g2o_files_1d/');

%% Initialize graph and set fixed node
[ Graph ] = InitializeGraph_1d;
Graph.Fixed.IDname.pose0 = 1;

%% Add edge of odometry and observation
data_edges = data.state;
noise_type = data.noise_type;
odom_sigma = data.odom_sigma;
obsv_sigma = data.obsv_sigma;

% add odometry
AllOdometry = data_edges(data_edges(:,2)==1,:);
if ~isempty(AllOdometry)
    num_odom = size(AllOdometry,1) / 1; % number of odometry
    for i = 1:num_odom
        OdometryFromThis2Next = AllOdometry(1*i-0:1*i,:);
        %theta_This2Next = OdometryFromThis2Next(1,1);
        %R_This2Next = so2_exp(theta_This2Next);   %[cos(theta_This2Next), -sin(theta_This2Next); sin(theta_This2Next), cos(theta_This2Next)];
        p_This_Next2This = OdometryFromThis2Next(1:1,1);
        IndexOfThisPose = OdometryFromThis2Next(1,4);
        IndexOfNextPose = OdometryFromThis2Next(1,3);
        
        NodeArray=cell(2,2);
        NodeArray{1,1}='Pose1';
        NodeArray{1,2}=['pose' num2str(IndexOfThisPose)];   %i-2 to i-1, not i-1 to i
        NodeArray{2,1}='Pose1';
        NodeArray{2,2}=['pose' num2str(IndexOfNextPose)];
        Measurement.value = [p_This_Next2This];
        Measurement.inf   = eye(1)/(odom_sigma^2);
        
        [ Graph ] = AddComplexEdge_1d(Graph, 'RelativePose1_Factor', NodeArray, Measurement);
    end
end

% add observation
AllObservation = data_edges(data_edges(:,2)==2,:);
if ~isempty(AllObservation)
    num_obsv = size(AllObservation,1) / 1;   % number of all observation
    for i = 1:num_obsv
        CurrentObservation = AllObservation(1*i-0:1*i,:);
        IndexOfCurrentLandmark = CurrentObservation(1,3);
        IndexOfCurrentPose = CurrentObservation(1,4);
        
        NodeArray=cell(2,2);
        NodeArray{1,1}='Pose1';
        NodeArray{1,2}=['pose' num2str(IndexOfCurrentPose)];
        NodeArray{2,1}='Landmark1';
        NodeArray{2,2}=['landmark' num2str(IndexOfCurrentLandmark)];
        Measurement.value = CurrentObservation(:,1);
        Measurement.inf   = eye(1)/(obsv_sigma^2);
            
        [ Graph ] = AddComplexEdge_1d(Graph, 'RGBD_1D_Factor', NodeArray, Measurement); 
    end
end

% n_steps = max(data_edges(:,4));  % step instead of pose,  hence, it does not include pose 0
% for i = 0:n_steps
%     if ( mod(i, 50) == 0 )                           % mod: find the remainder 77/50 = 1...27, mod(77,50) =27
%         disp(['Processing pose ', int2str(i)]);      % display which step we are in, every 50 steps
%     end
%     
%     edgeOfCurrentStep = data_edges(data_edges(:,4)==i,:);  % odometory and observation of current step
%     OdometryFromThis2Next = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==1,:);
%     CameraMeasurementThis = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==2,:);
% 
% % add odometry    
%     if ~isempty(OdometryFromThis2Next)
% %         theta_This2Next = OdometryFromThis2Next(1,1);   % no rotation in 1D case
% %         R_This2Next = so2_exp(theta_This2Next);         % no rotation in 1D case
%         p_This_Next2This = OdometryFromThis2Next(1:1,1);
%         
%         IndexOfThisPose = OdometryFromThis2Next(1,4);
%         IndexOfNextPose = OdometryFromThis2Next(1,3);
%         
%         NodeArray=cell(2,2);
%         NodeArray{1,1}='Pose1';
%         NodeArray{1,2}=['pose' num2str(IndexOfThisPose)];   %i-2 to i-1, not i-1 to i
%         NodeArray{2,1}='Pose1';
%         NodeArray{2,2}=['pose' num2str(IndexOfNextPose)];
% %         Measurement.value = [R_This2Next, p_This_Next2This];
%         Measurement.value = [p_This_Next2This];
%         Measurement.inf   = eye(1)/(odom_sigma^2);
%         
%         [ Graph ] = AddComplexEdge_1d(Graph, 'RelativePose1_Factor', NodeArray, Measurement);
%     end
% % add observation
%     if ~isempty(CameraMeasurementThis)
%         NumberOfLandmarksObInThisStep = size(CameraMeasurementThis,1)/1; % dimension=2
%         for j=1:NumberOfLandmarksObInThisStep
%             IndexOfCurrentLandmark = CameraMeasurementThis(1*j,3);
%             IndexOfCurrentPose = CameraMeasurementThis(1*j,4); % can also juse use i
%             
%             NodeArray=cell(2,2);
%             NodeArray{1,1}='Pose1';
%             NodeArray{1,2}=['pose' num2str(IndexOfCurrentPose)];
%             NodeArray{2,1}='Landmark1';
%             NodeArray{2,2}=['landmark' num2str(IndexOfCurrentLandmark)];
%             Measurement.value = CameraMeasurementThis(1*j-0:1*j,1);
%             Measurement.inf   = eye(1)/(obsv_sigma^2);
%             
%             [ Graph ] = AddComplexEdge_1d(Graph, 'RGBD_1D_Factor', NodeArray, Measurement); 
%         end
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Here are some instructions: a pose can observe a next pose, it can also observe some landmarks. 
%%% but this two things does not need to be achieve at the same time. Therefore, we need to check them separately
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Add nodes initial guess to graph
%%%%%%%%%%%%%%
%%% The initial guess of landmarks is the frist observation of each landmark. They are alculated to ground truth frame
%%% The initial guess of poses is calcualted from the odometry. They are calculated to ground truth frame.
%%% All of them are calcualted from the data.state: [landX; landY] or [theta; poseX; poseY]
%%%%%%%%%%%%%%

InitialPosition = initialGuess.poses.position;
InitialLandmark = initialGuess.landmarks;

num_pose = size(InitialPosition,1); %number of pose
num_land = size(InitialLandmark,1); %number of landmarks
% add pose initial guess [R p]: just odometryToGround
for i=1:num_pose
    Graph.Nodes.Pose1.Values.(['pose' num2str(i-1)]) = [InitialPosition(i,1:1)']; % [R p]. remember i-1
end
% add landmark initial guess: the first observation of each landmark
for i=1:num_land
    landmarkIndex = InitialLandmark(i,1);         % the index of landmark
    landmarkCoordinate = InitialLandmark(i,2:end)'; % the coordinate of landmark
    Graph.Nodes.Landmark1.Values.(['landmark' num2str(landmarkIndex)])=landmarkCoordinate;
end

%% Optimize G-N
if PrintResult
    tic
    [ Graph ] = PerformGO_1d( Graph );
    toc
else
    [ Graph ] = PerformGO_1d( Graph );
end

%% Get Covariance of estimated nodes:
% covariance of pose
num_UnfixedPose = num_pose - Graph.Fixed.Pose1;
poseCov = Graph.cov(1:1*num_UnfixedPose,1:1*num_UnfixedPose);   %NOTICE is the top-left block represent pose-pose covatiance?
poses_id = fields(Graph.Nodes.Pose1.Values); %the name of all poses
%posesCov = cell(num_pose,1);
poses_Cov = struct;  % the struct to contain the covariance of each poses
k=1;                 % the index of estimateCov. actually = num_pose - number of fixed poses
for i=1:num_pose
    if ~isfield( Graph.Fixed.IDname,  poses_id{i}) % if the current pose is not fixed node, then it has coovariance.
        poses_Cov.(['pose' num2str(i-1)]) = full(poseCov(1*k-0:1*k,1*k-0:1*k)); % convert to full matrix for reading
        k=k+1;                                     % k for the next unfixed pose
    else
        poses_Cov.(['pose' num2str(i-1)]) = zeros(1,1);
    end
end
Graph.Nodes.Pose1.Covariance = poses_Cov;

% debug = full(poses_Cov)
% debug = full(Graph.cov)

%covariance of landmarks
num_UnfixedLandmark = num_land - Graph.Fixed.Landmark1;
tem_CovIndexBegin = 1*num_UnfixedPose+1;
tem_CovIndexEnd   = 1*num_UnfixedPose+1*num_UnfixedLandmark;
LandCov = Graph.cov(tem_CovIndexBegin:tem_CovIndexEnd,tem_CovIndexBegin:tem_CovIndexEnd); %Why: is the right bottom block represent cov of landmarks? 

landmarks_id = fields(Graph.Nodes.Landmark1.Values); %the name of all landmarks
landmarks_Cov = struct;
k=1;
for i=1:num_land
    if ~isfield( Graph.Fixed.IDname,  landmarks_id{i}) % if the current landmark is not fixed node, then it has coovariance.
        landmarks_Cov.(landmarks_id{i}) = full(LandCov(1*k-0:1*k,1*k-0:1*k));
        k=k+1;                                     % k for the next unfixed pose
    else
        landmarks_Cov.(landmarks_id{i}) = zeros(1,1);
    end
end
Graph.Nodes.Landmark1.Covariance = landmarks_Cov;
%% Show result
if PrintResult
    fprintf('The result of pose: poseID, GroundTruth, EstimatedResult, InitialGuess\n')
    GroundPosition = data.poses.position;
    %InitialPosition;

    %num_pose
    NodeTypeArray = fields(Graph.Nodes);   %the type of the node
    % position
    for i=1:num_pose
        G_pi = GroundPosition(i,:);
        I_pi = InitialPosition(i,:);
        E_pi = Graph.Nodes.(NodeTypeArray{1}).Values.(['pose' num2str(i-1)])(1:1,1);
        fprintf('%d, %f, %f, %f \n', i, G_pi(1),E_pi(1),I_pi(1)  );
    end
    % landmark
    GroundLandmark = data.landmarks;
    %InitialLandmark
    fprintf('The result of landmark: landmarkID, GroundTruth, EstimatedResult, InitialGuess\n')
    for i=1:num_land
        Id_li = InitialLandmark(i,1);   %Id of landmark
        G_li = GroundLandmark(Id_li,:); % need to use index of landmark to get the ground truth
        I_li = InitialLandmark(i,2:end); 
        E_li = Graph.Nodes.(NodeTypeArray{2}).Values.(['landmark' num2str(Id_li)]);
        fprintf('%d, %f, %f, %f \n', Id_li, G_li(1),E_li(1),I_li(1)  );
    end
    
    % draw
%     Draw_LeastSquareEstimate_1d( Graph );
end

%% output
% Here we make it similar to the result of EKF for comparison

LastPose = Graph.Nodes.Pose1.Values.(['pose' num2str(num_pose-1)]);  %[R p] of last pose
LastPoseCov = Graph.Nodes.Pose1.Covariance.(['pose' num2str(num_pose-1)]); %Cov of last pose
% LS_SingleTime_estimation_result.orientation = LastPose(1:2,1:2);
LS_SingleTime_estimation_result.position = LastPose;
LS_SingleTime_estimation_result.poseCov  = LastPoseCov;
LS_SingleTime_estimation_result.landmarks = Graph.Nodes.Landmark1.Values;
LS_SingleTime_estimation_result.landCov  = Graph.Nodes.Landmark1.Covariance;

%% save result
if PrintResult
    %save LS_SingleTime_estimation_result_1d_1 LS_SingleTime_estimation_result
    LeastSquare_EstimationResult = Graph;
    save LeastSquare_1d_EstimationResult LeastSquare_EstimationResult;
%     clearvars -except LeastSquare_EstimationResult;   
end
% LeastSquare_EstimationResult = Graph;
% clearvars -except LeastSquare_EstimationResult;
% save LeastSquare_1d_EstimationResult LeastSquare_EstimationResult;
end