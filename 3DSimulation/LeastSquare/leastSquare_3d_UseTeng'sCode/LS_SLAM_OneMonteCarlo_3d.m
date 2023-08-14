function [LS_estimation_result] = LS_SLAM_OneMonteCarlo_3d(data,InitialGuess)
%LS_SLAM_OneMonteCarlo_2d: calculate each state incrementally, for result comparison with EKF
%   start from pose0 to the end of state.

num_state = max(data.state(:,4))+1; %number of state
LS_estimation_result = cell(1,num_state);

for i=0:num_state-1
%     i=17;
    disp(['       Processing Time Step ', int2str(i)]);
    Initialguess_thistime = getInitialGuessThisTime(InitialGuess,i);       % the initial guess by the state untill current poseI
    data_thistime         = getDateThisTime(data,i);                       % data by the state untill current poseI
    LS_estimation_result{i+1} = LS_SLAM_3d(data_thistime,Initialguess_thistime);
end
end

%% Get initial guess of this time, since we need to start the observation from pose0
function [InitialGuessThisTime] = getInitialGuessThisTime(InitialGuess,step)
%   InitialGuess: the initial guess of whole graph of one MC, step current corresponding to EKF

% get landmark
AllObservation = InitialGuess.GroundLandmark;
ObservedLandmark = AllObservation(AllObservation(:,6)<=step,:);
GroundLandmarkIndex = ObservedLandmark(ObservedLandmark(:,4)==1,5);           %The index of observed landmark
GroundLandmarkCoordinate = ObservedLandmark(ObservedLandmark(:,4)==1,1:3);  %the coordinate of the observed landmark
InitialGuessThisTime.landmarks = [GroundLandmarkIndex, GroundLandmarkCoordinate]; % just use the first-time observation

% get pose
InitialGuessThisTime.poses.position = InitialGuess.poses.position(1:(step+1),:);
InitialGuessThisTime.poses.orientation = InitialGuess.poses.orientation(1:3*(step+1),:);
end

%% Get data of this time.
function [DataThisTime] = getDateThisTime(data,step)
%getDataThisTime: start from pose0 and get data

% get state
AllState = data.state;
CurrentState = AllState(AllState(:,4)<=step,:);
if CurrentState(end,2)==1      % check if the final edge is pose-pose. We do not put pose-pose to current state, 
                               % since it represents the next propragate in EKF
    CurrentState = CurrentState(1:end-6,:);
end

% get pose
AllPosition = data.poses.position;
AllEuler = data.poses.euler;
AllOrientation = data.poses.orientation;

CurrentPosition = AllPosition(1:step+1,:);
CurrentEuler = AllEuler(:,1:step+1);
CurrentOrientation = AllOrientation(1:3*(step+1),:);
CurrentPoses.position = CurrentPosition;
CurrentPoses.euler    = CurrentEuler;
CurrentPoses.orientation = CurrentOrientation;

% get landmarks
% CurrentObservation = CurrentState(CurrentState(:,2)==2,:);
% 
% Index_ObservedLandmarks = CurrentObservation(1:3:end,3);     % the index of all abserved landmarks
% Index_CurrentLandmarks = unique(Index_ObservedLandmarks);    % remove complete data, and put it in ascending order.
% 
% CurrentLandmarks = data.landmarks(Index_CurrentLandmarks,:); % This is such a good way hahahahahahahaha

% output data
DataThisTime.state      = CurrentState;
DataThisTime.obse_cov   = data.obse_cov;
DataThisTime.odom_cov   = data.odom_cov;
DataThisTime.add_noise  = data.add_noise;
DataThisTime.noise_type = data.noise_type;
DataThisTime.odom_sigma = data.odom_sigma;
DataThisTime.obsv_sigma = data.obsv_sigma;
% DataThisTime.landmarks  = CurrentLandmarks;
DataThisTime.landmarks  = data.landmarks;
DataThisTime.poses      = CurrentPoses;
end