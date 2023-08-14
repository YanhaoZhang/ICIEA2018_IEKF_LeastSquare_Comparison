function estimation_results = EKF_SLAM_3d(data)
% left invariant ekf slam

% load pre-given data: odometry and observations
if nargin < 1
    load('../data_3d_1.mat');   % two dot means the previous folder, while one dot means the current folder
end

% addpath('../lie_utils/')
addpath('../Lee/');

% % load pre-given data: odometry and observations
% if nargin < 1
%     load('../datagen_3d/data.mat');
% end

% addpath('Math_Liegroup/');

% load pre-given data: odometry and observations
data_edges = data.state;
odo_cov = data.odom_cov;   % constant variable
obs_cov = data.obse_cov;   % constant variable

noise_type = data.noise_type;
odom_sigma = data.odom_sigma;
obsv_sigma = data.obsv_sigma;
% data_matrix = data.state;
% odo_cov = data.odom_cov;   % constant variable
% obs_cov = data.obse_cov;   % constant variable
% 
% 
% odom_sigma = data.odom_sigma;
% obsv_sigma = data.obsv_sigma;


%%%%%%%%%%%%%%%%%%%% Estimation_X is used to save the state in each step %%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%% In every step, all elements of Estimation_X will be changed %%%%%%%%%%%%
estimation_x.orientation = data.poses.orientation(1:3,1:3);
estimation_x.position    = data.poses.position(1,:)';
estimation_x.cov         = sparse(6,6);
estimation_x.landmarks   = [];       % the landmarks observed until this step (included), N*4 format, the 1-th column is the index
%Estimation_X.IndexOfFeature=[];     % the names(indexes) of the landmarks observed until this step (included)
%%%%%%%%%%%%%%%%%%%% Estimation_X is used to save the state in each step %%%%%%%%%%%%%%%%%%%%  


% Initialize
% n_steps = max(data_matrix(:,4));  % step instead of pose,  hence, it does not include pose 0
% estimation_results = cell(1, n_steps+1);
% estimation_results{1} = estimation_x;
temOdometry = data_edges(data_edges(:,2)==1,:); %all the odometry
num_poses = max(temOdometry(:,3));  % We need to use number of pose, instead of number of steps.
estimation_results = cell(1, num_poses+1); % x0 x11 x22 x33 x44. 
% Estimation_results = cell(1, num_poses+1); % for output
%{
    x0
    x1|0 <- x0
    x1|1 <- update (just initialize)
    x2|1 <- propragate
    x2|2 <- update
    ...
x1|1 has no propragate Since x0 is equalivant of x1|0, we need to update firstly, and then do the propragate for the next step.
%}

for i = 0:num_poses
%     if ( mod(i, 50) == 0 )                           % mod: find the remainder 77/50 = 1...27, mod(77,50) =27
%         disp(['Processing pose ', int2str(i)]);      % display which step we are in, every 50 steps
%     end
%     if i==100
%         pause;
%     end
    
    if i==0  % The first update at pose0
        edgeOfCurrentStep = data_edges(data_edges(:,4)==i,:);  % odometory and observation of current step
        CameraMeasurementThis = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==2,[1,3]); % both coordinate and index of the observed landmark in this step
        if ~isempty(CameraMeasurementThis)
            [estimation_x] = RI_EKF_update_3d(estimation_x, CameraMeasurementThis, obsv_sigma, noise_type);
        end
%         estimation_results{i+1} = estimation_x;
        Estimation_x = estimation_x;
        Estimation_x.cov = estimation_x.cov(1:6,1:6);
        estimation_results{i+1} = Estimation_x;
    else    % The second to end propragate and update
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%% Here we need to find the observation of current step, 
        %%% and odometry from last step to this step.
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        edgeOfCurrentStep = data_edges(data_edges(:,4)==i,:);  % observation of current step and odometry to next pose
        CameraMeasurementThis = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==2,[1,3]); % observation of current step. both coordinate and index of the observed landmark in this step
        edgeOfLastStep = data_edges(data_edges(:,4)==i-1,:);   % observation of current step and odometry to current pose
        OdometryFromLast2This = edgeOfLastStep(edgeOfLastStep(:,2)==1,1);
        
        % propagation using odometry info
        if ~isempty(OdometryFromLast2This)
            [estimation_x] = RI_EKF_propagate_3d(estimation_x, OdometryFromLast2This, odom_sigma, noise_type);
        end
        % update using observation info
        if ~isempty(CameraMeasurementThis)
            [estimation_x] = RI_EKF_update_3d(estimation_x, CameraMeasurementThis, obsv_sigma, noise_type);
        end
        Estimation_x = estimation_x;
        Estimation_x.cov = estimation_x.cov(1:6,1:6);
        estimation_results{i+1} = Estimation_x;
%         Estimation_results{i+1}.cov = estimation_results{i+1}.cov(1:6,1:6);
    end
end
        
        
        
%     IndexOfCurrentStepInDataMatrix = find(data_matrix(:,4) == i); 
%     m = size(IndexOfCurrentStepInDataMatrix, 1);
%     if ( mod(i, 50) == 0 )                           % mod: find the remainder 77/50 = 1...27, mod(77,50) =27
%         disp(['Processing pose ', int2str(i)]);      % display which step we are in, every 50 steps
%     end
%     
%     if i==100
%         pause;
%     end
%     
%     % det(Estimation_X.cov)
%     if i ~= n_steps
%         OdometryFromThis2Next = data_matrix(IndexOfCurrentStepInDataMatrix(m-5):IndexOfCurrentStepInDataMatrix(m),1); % the last one is poseI-pose(I+1) odometry. 
%                                                                                                                       %NOTICE There is a better way, use the logic matrix! data_matrix((data_matrix(:,3)==1), 1)
%         if m > 6                                     % means contains edge of both pose-pose(m=6) and pose-observation(m=3). but what if just observe 2 landmark? No, we put at least pose-pose edge in data.
%             CameraMeasurementThis = [ data_matrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(m-6) , 1 ),...
%                                       data_matrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(m-6) , 3 )];    
%             [estimation_x] = EKF_update(estimation_x, CameraMeasurementThis, obsv_sigma );
%         end
%         
%         estimation_results{i+1} = estimation_x;
%         
%         % propagation using odometry info
%         [estimation_x] = EKF_propagate(estimation_x, OdometryFromThis2Next, odom_sigma );
% 
%     else
%         if m > 6
%             CameraMeasurementThis = [ data_matrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(end) , 1 ) , data_matrix( IndexOfCurrentStepInDataMatrix(1): IndexOfCurrentStepInDataMatrix(end) , 3 )];
%            [estimation_x] = EKF_update(estimation_x, CameraMeasurementThis, obsv_sigma );
%         end
%         estimation_results{i+1} = estimation_x;
%     end
% end
% clearvars -except estimation_results;
%% save result
% save EKF_3d_estimation_results estimation_results
%% plot estimated trajectory
% PlotTrajectory;	