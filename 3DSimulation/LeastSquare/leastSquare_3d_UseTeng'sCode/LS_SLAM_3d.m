function LS_SingleTime_estimation_result = LS_SLAM_3d(data,initialGuess)
PrintResult = false;
%% For debuging Load pre-given data: odometry and observations
if nargin < 1
    load('../../3d_datagen/data_3d/data_3d_1.mat'); % two dot means the previous folder, while one dot means the current folder
    load('../../3d_datagen/data_3d_InitialGuess/NoiseDataInPose0Frame_3d_1.mat');
    initialGuess = NoiseDataInPose0Frame;
    PrintResult = true;
end
%% Add least square path
addpath('../Factor/');
addpath('../Math/');
addpath('../g2o_files/');

%% Initialize graph and set fixed node
[ Graph ] = InitializeGraph;
Graph.Fixed.IDname.pose0 = 1;

%% Add edge of odometry and observation
data_edges = data.state;
noise_type = data.noise_type;
odom_sigma = data.odom_sigma;
obsv_sigma = data.obsv_sigma;

% add odometry
AllOdometry = data_edges(data_edges(:,2)==1,:);
if ~isempty(AllOdometry)
    num_odom = size(AllOdometry,1) / 6; % number of odometry
    for i = 1:num_odom
        OdometryFromThis2Next = AllOdometry(6*i-5:6*i,:);
        theta_This2Next = OdometryFromThis2Next(1:3,1);
        R_This2Next = so3_exp(theta_This2Next);   %[cos(theta_This2Next), -sin(theta_This2Next); sin(theta_This2Next), cos(theta_This2Next)];
        p_This_Next2This = OdometryFromThis2Next(4:6,1);
        IndexOfThisPose = OdometryFromThis2Next(1,4);
        IndexOfNextPose = OdometryFromThis2Next(1,3);
        NodeArray=cell(2,2);
        NodeArray{1,1}='Pose3';
        NodeArray{1,2}=['pose' num2str(IndexOfThisPose)];   %i-2 to i-1, not i-1 to i
        NodeArray{2,1}='Pose3';
        NodeArray{2,2}=['pose' num2str(IndexOfNextPose)];
        Measurement.value = [R_This2Next, p_This_Next2This];
        
        switch noise_type
            case 0
                odom_Sigma = eye(6)*(odom_sigma^2);
            case 1
                odom_Sigma = diag([theta_This2Next; p_This_Next2This].^2)*odom_sigma^2; %WHY?
            otherwise
                sprintf('Error in EKF_update_2d: the noise_type has not been defined. Please check config_2d')
                return;
        end 
        Measurement.inf   = eye(6)/odom_Sigma;
        
        [ Graph ] = AddComplexEdge(Graph, 'RelativePose3_Factor', NodeArray, Measurement);
    end
end

% add observation
AllObservation = data_edges(data_edges(:,2)==2,:);
if ~isempty(AllObservation)
    num_obsv = size(AllObservation,1) / 3;   % number of all observation
    for i = 1:num_obsv
        CurrentObservation = AllObservation(3*i-2:3*i,:);
        IndexOfCurrentLandmark = CurrentObservation(1,3);
        IndexOfCurrentPose = CurrentObservation(1,4);
        
        NodeArray=cell(2,2);
        NodeArray{1,1}='Pose3';
        NodeArray{1,2}=['pose' num2str(IndexOfCurrentPose)];
        NodeArray{2,1}='Landmark3';
        NodeArray{2,2}=['landmark' num2str(IndexOfCurrentLandmark)];
        Measurement.value = CurrentObservation(:,1);
        switch noise_type
            case 0
                obsv_Sigma = eye(3)*(obsv_sigma^2);
            case 1
                obsv_Sigma = diag(CurrentObservation(:,1).^2)*obsv_sigma^2; %WHY?
            otherwise
                sprintf('Error in EKF_update_2d: the noise_type has not been defined. Please check config_2d')
                return;
        end 
        Measurement.inf   = eye(3)/obsv_Sigma;
            
        [ Graph ] = AddComplexEdge(Graph, 'RGBD_Factor', NodeArray, Measurement); 
    end
end



%{
n_steps = max(data_edges(:,4));  % step instead of pose,  hence, it does not include pose 0
for i = 0:n_steps
    if ( mod(i, 50) == 0 )                           % mod: find the remainder 77/50 = 1...27, mod(77,50) =27
        disp(['Processing pose ', int2str(i)]);      % display which step we are in, every 50 steps
    end
    
    edgeOfCurrentStep = data_edges(data_edges(:,4)==i,:);  % odometory and observation of current step
    OdometryFromThis2Next = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==1,:);
    CameraMeasurementThis = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==2,:);

% add odometry    
    if ~isempty(OdometryFromThis2Next)
        theta_This2Next = OdometryFromThis2Next(1:3,1);
        R_This2Next = so3_exp(theta_This2Next);   %[cos(theta_This2Next), -sin(theta_This2Next); sin(theta_This2Next), cos(theta_This2Next)];
        p_This_Next2This = OdometryFromThis2Next(4:6,1);
        
        IndexOfThisPose = OdometryFromThis2Next(1,4);
        IndexOfNextPose = OdometryFromThis2Next(1,3);
        
        NodeArray=cell(2,2);
        NodeArray{1,1}='Pose3';
        NodeArray{1,2}=['pose' num2str(IndexOfThisPose)];   %i-2 to i-1, not i-1 to i
        NodeArray{2,1}='Pose3';
        NodeArray{2,2}=['pose' num2str(IndexOfNextPose)];
        Measurement.value = [R_This2Next, p_This_Next2This];
        Measurement.inf   = eye(6)/(odom_sigma^2);
        
        [ Graph ] = AddComplexEdge(Graph, 'RelativePose3_Factor', NodeArray, Measurement);
    end
% add observation
    if ~isempty(CameraMeasurementThis)
        NumberOfLandmarksObInThisStep = size(CameraMeasurementThis,1)/3; % dimension=2
        for j=1:NumberOfLandmarksObInThisStep
            IndexOfCurrentLandmark = CameraMeasurementThis(3*j,3);
            IndexOfCurrentPose = CameraMeasurementThis(3*j,4); % can also juse use i
            
            NodeArray=cell(2,2);
            NodeArray{1,1}='Pose3';
            NodeArray{1,2}=['pose' num2str(IndexOfCurrentPose)];
            NodeArray{2,1}='Landmark3';
            NodeArray{2,2}=['landmark' num2str(IndexOfCurrentLandmark)];
            Measurement.value = CameraMeasurementThis(3*j-2:3*j,1);
            Measurement.inf   = eye(3)/(obsv_sigma^2);
            
            [ Graph ] = AddComplexEdge(Graph, 'RGBD_Factor', NodeArray, Measurement); 
        end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Here are some instructions: a pose can observe a next pose, it can also observe some landmarks. 
%%% but this two things does not need to be achieve at the same time. Therefore, we need to check them separately
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
%}
%% Add nodes initial guess to graph
%%%%%%%%%%%%%%
%%% The initial guess of landmarks is the frist observation of each landmark. They are alculated to ground truth frame
%%% The initial guess of poses is calcualted from the odometry. They are calculated to ground truth frame.
%%% All of them are calcualted from the data.state: [landX; landY] or [theta; poseX; poseY]
%%%%%%%%%%%%%%
% load('../../3d_datagen/NoiseDataInPose0Frame_3d.mat');
%load('GroundNoiseData_2d.mat');
InitialPosition = initialGuess.poses.position;
InitialRotation = initialGuess.poses.orientation;
InitialLandmark = initialGuess.landmarks;

num_pose = size(InitialPosition,1); %number of pose
num_land = size(InitialLandmark,1); %number of landmarks
% add pose initial guess [R p]: just odometryToGround
for i=1:num_pose
    Graph.Nodes.Pose3.Values.(['pose' num2str(i-1)]) = [InitialRotation(3*i-2:3*i,1:3), InitialPosition(i,1:3)']; % [R p]. remember i-1
end
% add landmark initial guess: the first observation of each landmark
for i=1:num_land
    landmarkIndex = InitialLandmark(i,1);         % the index of landmark
    landmarkCoordinate = InitialLandmark(i,2:end)'; % the coordinate of landmark
    Graph.Nodes.Landmark3.Values.(['landmark' num2str(landmarkIndex)])=landmarkCoordinate;
end

%% Optimize G-N
if PrintResult
    tic
    [ Graph ] = PerformGO( Graph );
    toc
else
    [ Graph ] = PerformGO( Graph );
end

%% Get Covariance of estimated nodes:
% covariance of pose
num_UnfixedPose = num_pose - Graph.Fixed.Pose3;
poseCov = Graph.cov(1:6*num_UnfixedPose,1:6*num_UnfixedPose);   %NOTICE is the top-left block represent pose-pose covatiance?
poses_id = fields(Graph.Nodes.Pose3.Values); %the name of all poses
%posesCov = cell(num_pose,1);
poses_Cov = struct;  % the struct to contain the covariance of each poses
k=1;                 % the index of estimateCov. actually = num_pose - number of fixed poses
for i=1:num_pose
    if ~isfield( Graph.Fixed.IDname,  poses_id{i}) % if the current pose is not fixed node, then it has coovariance.
        poses_Cov.(['pose' num2str(i-1)]) = full(poseCov(6*k-5:6*k,6*k-5:6*k)); % convert to full matrix for reading
        k=k+1;                                     % k for the next unfixed pose
    else
        poses_Cov.(['pose' num2str(i-1)]) = zeros(6,6);
    end
end
Graph.Nodes.Pose3.Covariance = poses_Cov;

%covariance of landmarks
num_UnfixedLandmark = num_land - Graph.Fixed.Landmark3;
tem_CovIndexBegin = 6*num_UnfixedPose+1;
tem_CovIndexEnd   = 6*num_UnfixedPose+3*num_UnfixedLandmark;
LandCov = Graph.cov(tem_CovIndexBegin:tem_CovIndexEnd,tem_CovIndexBegin:tem_CovIndexEnd); %Why: is the right bottom block represent cov of landmarks? 

landmarks_id = fields(Graph.Nodes.Landmark3.Values); %the name of all landmarks
landmarks_Cov = struct;
k=1;
for i=1:num_land
    if ~isfield( Graph.Fixed.IDname,  landmarks_id{i}) % if the current landmark is not fixed node, then it has coovariance.
        landmarks_Cov.(landmarks_id{i}) = full(LandCov(3*k-2:3*k,3*k-2:3*k));
        k=k+1;                                     % k for the next unfixed pose
    else
        landmarks_Cov.(landmarks_id{i}) = zeros(3,3);
    end
end
Graph.Nodes.Landmark3.Covariance = landmarks_Cov;
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
        E_pi = Graph.Nodes.(NodeTypeArray{1}).Values.(['pose' num2str(i-1)])(1:3,4);
    
        fprintf('%d, %f, %f, %f \n', i, G_pi(1),E_pi(1),I_pi(1)  );
        fprintf('%d, %f, %f, %f \n', i, G_pi(2),E_pi(2),I_pi(2)  );
        fprintf('%d, %f, %f, %f \n', i, G_pi(3),E_pi(3),I_pi(3)  );
    end
    % landmark
    GroundLandmark = data.landmarks;
    %InitialLandmark
    fprintf('The result of landmark: landmarkID, GroundTruth, EstimatedResult, InitialGuess\n')
    for i=1:num_land
        Id_li = InitialLandmark(i,1);   %Id of landmark
        G_li = GroundLandmark(Id_li,:);
        I_li = InitialLandmark(i,2:end);
        E_li = Graph.Nodes.(NodeTypeArray{2}).Values.(['landmark' num2str(Id_li)]);
    
        fprintf('%d, %f, %f, %f \n', Id_li, G_li(1),E_li(1),I_li(1)  );
        fprintf('%d, %f, %f, %f \n', Id_li, G_li(2),E_li(2),I_li(2)  );
        fprintf('%d, %f, %f, %f \n', Id_li, G_li(3),E_li(3),I_li(3)  );
    end

%     Draw_LeastSquareEstimate_3d( Graph );
end
% PlotTrajectoryLandmark( Graph )
% hold on
% PlotTrajectoryLandmark_InitialGuess(PoseInitialGuess,LandmarkInitialGuess,'LandmarkGraph2')
% hold on
% PlotTrajectoryLandmark_Ground( GroundPose, GroundLandmark, 'LandmarkGraph2')
% grid on
%% output
% Here we make it similar to the result of EKF for comparison

LastPose = Graph.Nodes.Pose3.Values.(['pose' num2str(num_pose-1)]);  %[R p] of last pose
LastPoseCov = Graph.Nodes.Pose3.Covariance.(['pose' num2str(num_pose-1)]); %Cov of last pose
LS_SingleTime_estimation_result.orientation = LastPose(1:3,1:3);
LS_SingleTime_estimation_result.position = LastPose(1:3,4);
LS_SingleTime_estimation_result.poseCov  = LastPoseCov;
LS_SingleTime_estimation_result.landmarks = Graph.Nodes.Landmark3.Values;
LS_SingleTime_estimation_result.landCov  = Graph.Nodes.Landmark3.Covariance;

%% save result
% LeastSquare_EstimationResult = Graph;
% clearvars -except LeastSquare_EstimationResult;
% save LeastSquare_3d_EstimationResult LeastSquare_EstimationResult;
%% draw
% Draw_LeastSquareEstimate_3d( LeastSquare_EstimationResult );
end