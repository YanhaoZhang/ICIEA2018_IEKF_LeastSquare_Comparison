function [InitialGuessThisTime] = debug_getInitialGuessThieTime(InitialGuess,step)
%   InitialGuess: the initial guess of whole graph of one MC, step current corresponding to EKF


%% get landmark
AllObservation = InitialGuess.GroundLandmark;
ObservedLandmark = AllObservation(AllObservation(:,5)<=step,:);

GroundLandmarkIndex = ObservedLandmark(ObservedLandmark(:,3)==1,4);           %The index of observed landmark
GroundLandmarkCoordinate = ObservedLandmark(ObservedLandmark(:,3)==1,[1 2]);  %the coordinate of the observed landmark
InitialGuessThisTime.landmarks = [GroundLandmarkIndex, GroundLandmarkCoordinate]; % just use the first-time observation

%% get pose
% ObservedPosition = InitialGuess.poses.position;
InitialGuessThisTime.poses.position = InitialGuess.poses.position(1:(step+1),:);
InitialGuessThisTime.poses.orientation = InitialGuess.poses.orientation(1:2*(step+1),:);
end

% getInitialGuessThieTime(NoiseDataInPose0Frame,1)