function [] = drawGroundTruth_1d(data)
if nargin < 1
    load('../../1d_datagen/data_1d.mat');   % two dot means the previous folder, while one dot means the current folder
end

%% plot pose position [x y]
poses = data.poses;
%figure;
plot( poses.position, 0, 'r*' ); hold on;

% draw 1d local axis
axisl = 5;
for i = 1:size(poses.position, 1)
    rotationi = [1; 0];                     % the local x-axis. In 1D case, it just has one axis
    positioni = [poses.position(i,:) 0]';
    xdir = positioni + axisl*rotationi;
    xdir = [positioni, xdir];
    plot(xdir(1,:), xdir(2,:), 'Color', 'red'); hold on;
end

%% draw landmarks
landmarks = data.landmarks;
plot( landmarks, 0, 'go' ); hold on;
%% draw observations
observedLandmark = [];
state = data.state;
for i=1:size(poses.position, 1)
    statePoseI = state(state(:,4)==i,:);        %All edges from poseI
    obsers = statePoseI(statePoseI(:,2)==2,:);  %edges from poseI to the observed landmarks by poseI
    
    if ~isempty (obsers)
        obserIndex_poseI = obsers(:,3);          % which landmarks are ovserved by poseI
        obser_poseI = landmarks(obserIndex_poseI,:); % coordinate of the observed landmarks
        observedLandmark = [observedLandmark;obser_poseI];
    end
end
plot( observedLandmark, 0, 'g.','MarkerSize',20 ); hold on;

%% draw observation edges
%draw_edge_2d(data)
end