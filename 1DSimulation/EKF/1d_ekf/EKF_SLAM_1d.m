function estimation_results = EKF_SLAM_1d(data)
%EKF_SLAM_1d normal EKF for 1D data
%   the data are generated by function 1d_datagen
%   data.pose, data.landmarks, data.state are position, landmarks, and odometry&observation. There is no rotation in 1D case.
%   NOTICE: left invariant ekf slam 

% load pre-given data: odometry and observations
if nargin < 1
    load('../../1d_datagen/data_1d/data_1d_1.mat');   % two dot means the previous folder, while one dot means the current folder
end

% addpath('Math_Liegroup/');
addpath('../Lee/');

% load pre-given data: odometry and observations
data_edges = data.state;
odo_cov = data.odom_cov;   % constant variable
obs_cov = data.obse_cov;   % constant variable

noise_type = data.noise_type;
odom_sigma = data.odom_sigma;
obsv_sigma = data.obsv_sigma;


%%%%%%%%%%%%%%%%%%%% Estimation_X is used to save the state in each step %%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%% In every step, all elements of Estimation_X will be changed %%%%%%%%%%%%
%estimation_x.orientation = data.poses.orientation(1:2,1:2);   % 1D there is no rotation. 
estimation_x.position    = data.poses.position(1,:)';          % estimation_x is the state of each EKF step.
estimation_x.cov         = 0;                                  % 0: The covariance of pose0 is zero. 1D covariance is just one number.
                                                               % 2D: covariance is 3*3 matrix(theta first); and 3D: 6*6.
estimation_x.landmarks   = [];                                 % the landmarks observed until this step (included)(EXCLUDED?), 
                                                               % N*2 format, the 1-th column is the index of landmark.
%%%%%%%%%%%%%%%%%%%% Estimation_X is used to save the state in each step %%%%%%%%%%%%%%%%%%%%  


% Initialize
% n_steps = max(data_edges(:,4));  % step instead of pose,  hence, it does not include pose 0
% estimation_results = cell(1, n_steps+1); % x0 x11 x22 x33 x44. 
temOdometry = data_edges(data_edges(:,2)==1,:); %all the odometry
maxIndex_poses = max(temOdometry(:,3));  % We need to use the maximum index of pose, instead of number of steps.
estimation_results = cell(1, maxIndex_poses+1); % x0 x11 x22 x33 x44. 
%{
    x0
    x1|0 <- x0
    x1|1 <- update (just initialize)
    x2|1 <- propragate
    x2|2 <- update
    ...
x1|1 has no propragate. Since x0 is equalivant of x1|0, we need to update firstly, and then do the propragate for the next step.
%}

for i = 0:maxIndex_poses
    if ( mod(i, 50) == 0 )                           % mod: find the remainder 77/50 = 1...27, mod(77,50) =27
        disp(['Processing pose ', int2str(i)]);      % display which step we are in, every 50 steps
    end
    if i==0  % The first update at pose0
        edgeOfCurrentStep = data_edges(data_edges(:,4)==i,:);  % odometory and observation of current step
        CameraMeasurementThis = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==2,[1,3]); % both coordinate and index of the observed landmark in this step
        if ~isempty(CameraMeasurementThis)
            [estimation_x] = EKF_update_1d(estimation_x, CameraMeasurementThis, obsv_sigma, noise_type);
        end
        estimation_results{i+1} = estimation_x;
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
            [estimation_x] = EKF_propagate_1d(estimation_x, OdometryFromLast2This, odom_sigma, noise_type);
        end
        % update using observation info
        if ~isempty(CameraMeasurementThis)
            [estimation_x] = EKF_update_1d(estimation_x, CameraMeasurementThis, obsv_sigma, noise_type);
        end
        estimation_results{i+1} = estimation_x;
    end
    
%     edgeOfCurrentStep = data_edges(data_edges(:,4)==i,:);  % odometory and observation of current step
%     OdometryFromThis2Next = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==1,1);
%     CameraMeasurementThis = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==2,[1,3]); % both coordinate and index of the observed landmark in this step
%     
%     if ~isempty(CameraMeasurementThis)
%         [estimation_x] = EKF_update_1d(estimation_x, CameraMeasurementThis, obsv_sigma, noise_type);
%     end
%         estimation_results{i+1} = estimation_x;    % we need to update firstly and put it into the estimation_result. the propagate uses the last state
%         
%     % propagation using odometry info
%     if ~isempty(OdometryFromThis2Next)
%         [estimation_x] = EKF_propagate_1d(estimation_x, OdometryFromThis2Next, odom_sigma, noise_type);
%     end
end
    
clearvars -except estimation_results;
%% save result
save EKF_1d_estimation_results estimation_results
%% plot estimated trajectory
% PlotTrajectory;
end