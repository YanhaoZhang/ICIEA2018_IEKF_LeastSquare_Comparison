function [DataThisTime] = debug_getDataThisTime(data,step)
%debug_getDataThisTime: start from pose0 and get data


AllState = data.state;
CurrentState = AllState(AllState(:,4)<=step,:);
if CurrentState(end,2)==1      % check if the final edge is pose-pose. We do not put pose-pose to current state, 
                               % since it represents the next propragate in EKF
    CurrentState = CurrentState(1:end-3,:);
end

% get pose
AllPosition = data.poses.position;
AllEuler = data.poses.euler;
AllOrientation = data.poses.orientation;

CurrentPosition = AllPosition(1:step+1,:);
CurrentEuler = AllEuler(:,1:step+1);
CurrentOrientation = AllOrientation(1:2*(step+1),:);
CurrentPoses.position = CurrentPosition;
CurrentPoses.euler    = CurrentEuler;
CurrentPoses.orientation = CurrentOrientation;

% get landmarks
CurrentObservation = CurrentState(CurrentState(:,2)==2,:);

Index_ObservedLandmarks = CurrentObservation(1:2:end,3);     % the index of all abserved landmarks
Index_CurrentLandmarks = unique(Index_ObservedLandmarks);    % remove complete data, and put it in ascending order.

CurrentLandmarks = data.landmarks(Index_CurrentLandmarks,:); % This is such a good way hahahahahahahaha

% output data
DataThisTime.state      = CurrentState;
DataThisTime.obse_cov   = data.obse_cov;
DataThisTime.odom_cov   = data.odom_cov;
DataThisTime.add_noise  = data.add_noise;
DataThisTime.noise_type = data.noise_type;
DataThisTime.odom_sigma = data.odom_sigma;
DataThisTime.obsv_sigma = data.obsv_sigma;
DataThisTime.landmarks  = CurrentLandmarks;
DataThisTime.poses      = CurrentPoses;
end