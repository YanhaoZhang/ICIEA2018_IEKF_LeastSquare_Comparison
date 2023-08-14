function EKF_estimation_results = EKF_SLAM_2d(data)
% right invariant ekf slam 

% load pre-given data: odometry and observations
if nargin < 1
    load('../data_2d_1.mat');   % two dot means the previous folder, while one dot means the current folder
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
estimation_x.orientation = data.poses.orientation(1:2,1:2);
estimation_x.position    = data.poses.position(1,:)';
estimation_x.cov         = sparse(3,3);     % zeros(3,3)  The covariance of pose0 is zero
%estimation_x.landmarks   = [];       % the landmarks observed until this step (included)(EXCLUDED?), 4*N format, the 4-th row is the index
%Estimation_X.IndexOfFeature=[];     % the names(indexes) of the landmarks observed until this step (included)
estimation_x.landmarks   = [];       % the landmarks observed until this step (included)(EXCLUDED?), N*3 format, the 1-th column is the index
%%%%%%%%%%%%%%%%%%%% Estimation_X is used to save the state in each step %%%%%%%%%%%%%%%%%%%%  


% Initialize
% n_steps = max(data_edges(:,4));  % step instead of pose,  hence, it does not include pose 0
% estimation_results = cell(1, n_steps+1); % x0 x11 x22 x33 x44. 
temOdometry = data_edges(data_edges(:,2)==1,:); %all the odometry
num_poses = max(temOdometry(:,3));  % We need to use number of pose, since instead of number of steps.
EKF_estimation_results = cell(1, num_poses+1); % x0 x11 x22 x33 x44. 
% EKF_estimation_results = cell(1, num_poses+1);
%{
    x0
    x1|0 <- x0
    x1|1 <- update (just initialize)
    x2|1 <- propragate
    x2|2 <- update
    ...
x1|1 has no propragate Since x0 is equalivant of x1|0, we need to update firstly, and then do the propragate for the next step.
%}

%% debug
% PropragateX = zeros(2,num_poses+1);
% UpdateX =  zeros(2,num_poses+1);
% GroundX = data.poses.position';
% num_land = size(data.landmarks,1);
% GroundL = data.landmarks;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
for i = 0:num_poses
%     if ( mod(i, 50) == 0 )                           % mod: find the remainder 77/50 = 1...27, mod(77,50) =27
%         disp(['Processing pose ', int2str(i)]);      % display which step we are in, every 50 steps
%     end
    if i==0  % The first update at pose0
        edgeOfCurrentStep = data_edges(data_edges(:,4)==i,:);  % odometory and observation of current step
        CameraMeasurementThis = edgeOfCurrentStep(edgeOfCurrentStep(:,2)==2,[1,3]); % both coordinate and index of the observed landmark in this step
        if ~isempty(CameraMeasurementThis)
            
% PropragateX(:,i+1) = estimation_x.position;
            
            [estimation_x] = EKF_update_2d(estimation_x, CameraMeasurementThis, obsv_sigma, noise_type);
            
% UpdateX(:,i+1) = estimation_x.position;
        end
        Estimation_x = estimation_x;
        Estimation_x.cov = estimation_x.cov(1:3,1:3);
        EKF_estimation_results{i+1} = Estimation_x;
        
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
            [estimation_x] = EKF_propagate_2d(estimation_x, OdometryFromLast2This, odom_sigma, noise_type);
        end
        % update using observation info
        if ~isempty(CameraMeasurementThis)
% PropragateX(:,i+1) = estimation_x.position;

            [estimation_x] = EKF_update_2d(estimation_x, CameraMeasurementThis, obsv_sigma, noise_type);
            
% UpdateX(:,i+1) = estimation_x.position;
        end
        Estimation_x = estimation_x;
        Estimation_x.cov = estimation_x.cov(1:3,1:3);
        EKF_estimation_results{i+1} = Estimation_x;
    end
end
% clearvars -except estimation_results;
%% save result
% save EKF_2d_estimation_results EKF_estimation_results
% save EKF_2d_estimation_results estimation_results -v7.3
%% plot estimated trajectory
% PlotTrajectory;
end