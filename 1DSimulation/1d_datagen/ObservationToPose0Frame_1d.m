%% This is an extended version of OdometryToGround.m
% NOTICE the point of this function is to get Nodes of pose and observation
% as initial guess.
%{
   I change the way of data storage. In OdometryToGround, I use cell and
   sparse cell (sparse matrix) to store data, but it is too slow for
   practice. For applying the code to victoria example, we need to use
   another way. Here I just choose the combined way according to the data
   of victoria park example.
   Input odometry is m*4 matrix; feature is n*4 matrix

Data field instrument: file Zstate_VicPark_6898_loops.mat contains the
    lazor data of poses and features (2-D), all in the local frame. The
    first column is the local coordinates of features and the poses. the
    second column is a observation flag, 2 means the coordinate is
    pose-feature, while 1 means the coordiante is pose-pose. The third
    column is the number of landmarks while the forth column is the number
    of poses.
    For pose-pose: [x;y;theta]
    for pose-observation [x;y]
dimention is used to calculate how many numbers should be combined together
    as the coordinate of feature/landmark and odometry/pose
graphtype: 
    'PoseGraph2'    2-D pose-pose graph, [x;y;theta]
    'LandmarkGraph' 3-D pose-landmark graph [x;y]
    'PoseGraph3'    Should be set according to the data storage of odometry
    'PoseGraph3'    .. data storage of odometry and feature
landmarkNode is the averaged grandtruth coordination of observation, n*2
Edge: we don't need the Edge_pose and Edge_observation, since we can get
    them directly from data. 

posePosition: [x,y] for pose. poseRotation [cos -sin;sin cos] for pose
landmark:     [x,y] in the first two column and the corresponding pose_id and
              landmark_id in the forth and third column
This is a faster way than Teng's using struct graph-----I assume


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

GroundLandmark: [x y times indexLand indexPose]

%}

function [NoiseDataInPose0Frame] = ObservationToPose0Frame_1d(data)
if nargin < 1
    load('./data_1d.mat');
    %state = data.state;
end
state = data.state;
poition0 = data.poses.position(1,:);
% orientation0 = data.poses.orientation(1:2,1:2);


% addpath('../Lee/');

GroundPosition = [];
GroundRotation = [];
GroundLandmark = [];

n_steps = max(state(:,4));  % step instead of pose,  hence, it does not include pose 0

GroundPosition0 = poition0;    %position of pose0
%GroundRotation0 = eye(2);   %rotation of pose0
GroundPosition = [GroundPosition;GroundPosition0];
%GroundRotation = [GroundRotation;GroundRotation0];

for i = 0:n_steps
%     if ( mod(i, 50) == 0 )                           % mod: find the remainder 77/50 = 1...27, mod(77,50) =27
%         disp(['Processing pose ', int2str(i)]);      % display which step we are in, every 50 steps
%     end
    
    edgeOfCurrentStep = state(state(:,4)==i,:);  % odometory and observation of current step
    OdometryFromThis2Next = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==1,1);
    CameraMeasurementThis = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==2,[1,3,4]); %[coordinate indexLand indexPose]
    %% Calculate landmarks
    if ~isempty(CameraMeasurementThis)
        num_LandmarkThis = size(CameraMeasurementThis,1)/1; %number of landmarks observed by this time. 1D dimension is 1
        for j=1:num_LandmarkThis
            l_I_KI = CameraMeasurementThis(1*j-0:1*j,1);    %observation in the local frame. 1D:1*j-0:1*j; 2D:2*j-1:2*j; 3D:3*j-2:3*j
            index_landK = CameraMeasurementThis(1*j,2);     
            index_poseI = CameraMeasurementThis(1*j,3);
            
             k = i+1;                                 % we need to calculate k in edges of both observation and pose, since there can be only pose-pose or pose-landmark
%             R_0I = GroundRotation(2*k-1:2*k,1:2);    % use last frame
            p_0_I0 = GroundPosition(k,:)';
%             l_0_K0 = R_0I*l_I_KI + p_0_I0;             % observation in ground truth
            l_0_K0 = l_I_KI + p_0_I0;             % observation in ground truth. 1D, l_I_KI = l_0_KI
            
            GroundObservationK = [l_0_K0', 0, index_landK, index_poseI]; % the first one values are the coordinates, the second one 
                                                                         % is times the same landmark has been observed, the last 
                                                                         % two are the index of landmark and current pose.
            GroundLandmark = [GroundLandmark; GroundObservationK];       % put the result in GroundLandmark firstly, then check how many times
            % Check how many times this landmark has been observed:      Here is a trick i figure it out!
            m = find(GroundLandmark(:,3)==index_landK);
            ObservedTimes = size(m,1);
            GroundLandmark(end,2) = ObservedTimes;
        end 
    end
    %% Calculate pose
    if ~isempty(OdometryFromThis2Next)
%         theta_LastThis = OdometryFromThis2Next(1); %the first value of each odometry is theta
%         R_LastThis = Exp_2d(theta_LastThis);   % ../Lee/
        p_Last_ThisLast = OdometryFromThis2Next;  % pose1->pose2: p_1_21
        
        k = i+1;                                       % the index of last pose. 1 represents pose0.
%         R_0Last = GroundRotation(2*k-1:2*k,1:2);
        p_0_Last0 = GroundPosition(k,:)';
%         R_0This = R_0Last*R_LastThis;                     % R_03 = R_02*R_23
%         p_0_ThisO = R_0Last*p_Last_ThisLast + p_0_Last0; % p_0_30 = R_02'*p_2_32 + p_0_20
        p_0_ThisO = p_Last_ThisLast + p_0_Last0; % p_0_30 = R_02'*p_2_32 + p_0_20
        
%         GroundRotation = [GroundRotation;R_0This];
        GroundPosition = [GroundPosition;p_0_ThisO'];
    end
end
% GroundNoiseData.GroundRotation = GroundRotation;
NoiseDataInPose0Frame.GroundPosition = GroundPosition;
NoiseDataInPose0Frame.GroundLandmark = GroundLandmark;

% for initializing and drawing
NoiseDataInPose0Frame.poses.position = GroundPosition;
% GroundNoiseData.poses.orientation = GroundRotation;
GroundLandmarkIndex = GroundLandmark(GroundLandmark(:,2)==1,3);       %The index of observed landmark
GroundLandmarkCoordinate = GroundLandmark(GroundLandmark(:,2)==1,1);  %the coordinate of the observed landmark
NoiseDataInPose0Frame.landmarks = [GroundLandmarkIndex, GroundLandmarkCoordinate]; % just use the first-time observation. [index and coordinate]
% save NoiseDataInPose0Frame_1d NoiseDataInPose0Frame
% 
% clearvars -except NoiseDataInPose0Frame;


% for i = 0:n_steps
%     if ( mod(i, 50) == 0 )                           % mod: find the remainder 77/50 = 1...27, mod(77,50) =27
%         disp(['Processing pose ', int2str(i)]);      % display which step we are in, every 50 steps
%     end
%     
%     edgeOfCurrentStep = data_edges(data_edges(:,4)==i,:);  % odometory and observation of current step
%     OdometryFromThis2Next = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==1,1);
%     CameraMeasurementThis = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==2,[1,3]);
%     
%     if ~isempty(CameraMeasurementThis)
%         [estimation_x] = EKF_update_2d(estimation_x, CameraMeasurementThis, obsv_sigma, noise_type);
%     end
%         estimation_results{i+1} = estimation_x;    % we need to update firstly and put it into the estimation_result. the propagate uses the last state
%         
%     % propagation using odometry info
%     if ~isempty(OdometryFromThis2Next)
%         [estimation_x] = EKF_propagate_2d(estimation_x, OdometryFromThis2Next, odom_sigma, noise_type);
%     end
% end
% 
% 
% 
% 
%     %observation of pose and landmarks at poseI
%     IndexOfCurrentStepInDataMatrix = find(state(:,4) == i);
%     m = size(IndexOfCurrentStepInDataMatrix, 1);
%     
%     if i ~= n_steps
%         OdometryFromThis2Next = state(IndexOfCurrentStepInDataMatrix(m-2):IndexOfCurrentStepInDataMatrix(m),1);                %pose-pose odometry
%         CameraMeasurementThis = [ data_edges( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(m-3) , 1 ),... % pose-landmark observation
%                                   data_edges( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(m-3) , 3 )];
%         
%         theta_LastThis = OdometryFromThis2Next(1);     % pose1->pose2: theta_12
%         R_LastThis = skew_2d(theta_LastThis);          % ../Math/
%         p_Last_ThisLast = OdometryFromThis2Next(2:3);  % pose1->pose2: p_1_21
%         
%         k = i+1;                                       % the index of last pose. 1 represents pose0.
%         R_0Last = GroundRotation(2*k-1:2*k,1:2);
%         p_0_Last0 = GroundRotation(i,:)';
%         R_0This = R_0Last*R_LastThis;                     % R_03 = R_02*R_23
%         p_0_ThisO = R_0Last'*p_Last_ThisLast + p_0_Last0; % p_0_30 = R_02'*p_2_32 + p_0_20
%         
%         GroundRotation = [GroundRotation;R_0This];
%         GroundPosition = [GroundPosition;p_0_ThisO'];
%         
%         
%         
%         
% end
% 
% 
% 
% 
% odometry = state(state(:,2)==1,:);    % pose-pose odometry
% observation = state(state(:,2)==2,:); % pose-landmark observation
% num_state = size(odometry,1)/3;       % number of state. = num_pose + 1. pose1, pose2,...,pose4 in the local frame
% num_land = max(state(:,3));           % number of landmarks
% GroundPosition = zeros(num_state+1,2);   % pose in the frame of ground truth
% GroundRotation = zeros((num_state+1)*2,2);
% GroundLandmark = zeros(num_land,2);   % observation in the frame of ground truth
% 
% GroundRotation(1:2,1:2) = eye(2);
% 
% k = 0;    % index of the last pose, 0 means pose0
% 
% for i=1:3:num_state*3
%     theta_LastThis = odometry(i,1);         %pose1->pose2: theta_12
%     R_LastThis = skew_2d(theta_LastThis);   % ../Math/
%     p_Last_ThisLast = odometry(i+1:i+2,1);  %pose1->pose2: p_1_21
%     
%     R_
% end

%{

graphtypeflag = 0;
switch graphtype
    case 'PoseGraph2'
        %% Just calcualte the 2-D pose-pose
        pose_num = max(odometry(:,3)) + 1;     % the number of pose. form pose0 to poseLast. Plus one is for the index of vector in matlab starts from 1.
        dimention = 2;
        %pose_step = dimention + 1;                         % [x;y;theta] every 3 numbers are the coordinate of one pose
        posePosition = zeros(pose_num,dimention);           % initialize pose, and the first coordination is [0 0]. 
        poseRotation = zeros(pose_num*dimention,dimention); % initialize Rotation 
                                                            % posePosition m*2; poseRotation 2m*2

        p_0_this0 = [0; 0];                    % pointer is now at pose0. The parameter of last coordinate in the frame of pose0
                                               % NOTICE: in the process of calculation, we use coordination [x;y], while we change it into [x y] when outputing it
        R_0this = expm(skew2(0));              % The first rotation is [0 0;0 0]
        poseRotation(1:2,:) = R_0this;
        for i=1:pose_num                       % from the second pose but is the first odometry
            R_0last   = R_0this;               % pose of last coordinate in the frame of pose0. I do this to record the rotation matrix without re-using the function expm(skew2(theta))
            p_0_last0 = p_0_this0;
            p_last_thisLast = [odometry(i,1); odometry(i+1,1);];
            R_lastThis = expm(skew2( odometry(i+2,1) ));  
            R_0this   = R_0last * R_lastThis;
            p_0_this0 = R_0last*p_last_thisLast + p_0_last0;
            posePosition(i+1,:) = p_0_this0';

            temIndex_1 = i*dimention+1;
            temIndex_2 = i*dimention+2;
            poseRotation(temIndex_1:temIndex_2,:) = R_0this;
        end
        graphtypeflag = 1;
   
    case 'LandmarkGraph2'
        %% calculate 2-D pose-pose and pose-landmark
        % pose (same as case 'PoseGraph2')
        pose_num = max(odometry(:,3)) + 1;   
        dimention = 2;
        posePosition = zeros(pose_num,dimention);           
        poseRotation = zeros(pose_num*dimention,dimention); 
        p_0_this0 = [0; 0];                                        
        R_0this = expm(skew2(0));             
        poseRotation(1:2,:) = R_0this;
        for i=1:pose_num            
            R_0last   = R_0this;           
            p_0_last0 = p_0_this0;
            p_last_thisLast = [odometry(i,1); odometry(i+1,1);];
            R_lastThis = expm(skew2( odometry(i+2,1) ));  
            R_0this   = R_0last * R_lastThis;
            p_0_this0 = R_0last*p_last_thisLast + p_0_last0;
            posePosition(i+1,:) = p_0_this0';
            temIndex_1 = i*dimention+1;
            temIndex_2 = i*dimention+2;
            poseRotation(temIndex_1:temIndex_2,:) = R_0this;
        end
        
        % landmark
        land_num = max(observation(:,3));    % the number of landmarks
        
        
        
        for i=2:pose_num
            R_0i   = pose{i}(1:dimention,1:dimention);
            p_0_i0 = pose{i}(1:dimention,(dimention+1));
            for j=1:land_num
                if ~isempty(observation{i,j})
                    landmark{i,j} = R_0i*observation{i,j} + p_0_i0;
                end
            end
        end
        graphtypeflag = 1;
    case 'PoseGraph3'
        %% Calcualte the 2-D pose-pose
        graphtypeflag =0;
    case 'LandmarkGraph3'
       %% calculate 3-D pose-pose and pose-landmark 
        graphtypeflag =0;
end

if graphtypeflag ==0
    sprintf('Warning: graphtype has not been setup in ObservationToGround.m')
    pose = -1;  % -1 represents an error
end

%}
end