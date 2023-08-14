function [] = drawGroundTruth_3d(data)
if nargin < 1
    load('../../3d_datagen/data_3d.mat');   % two dot means the previous folder, while one dot means the current folder
end

%% plot pose position [x y]
poses = data.poses;
%figure;
scatter3( poses.position(:,1), poses.position(:,2), poses.position(:,3),'go' ); hold on;
plot3( poses.position(:,1), poses.position(:,2),poses.position(:,3), '.','Color', [204,153,0]/255 ,'MarkerSize',20);
hold on;

%draw axis
axisl = 5;
for i = 1:size(poses.position, 1)
    rotationi = poses.orientation((i-1)*3+1:i*3, :);
    positioni = poses.position(i,:)';
    xdir = positioni + axisl*rotationi(:, 1);
    xdir = [positioni, xdir];
    ydir = positioni + axisl*rotationi(:, 2);
    ydir = [positioni, ydir];
%     plot(xdir(1,:), xdir(2,:), 'Color', 'red'); hold on;
%     plot(ydir(1,:), ydir(2,:), 'Color', 'blue'); hold on;
    zdir = positioni + axisl*rotationi(:, 3);
    zdir = [positioni, zdir];
    plot3(xdir(1,:), xdir(2,:), xdir(3,:), 'Color', 'red','LineStyle','-'); hold on;
    plot3(ydir(1,:), ydir(2,:), ydir(3,:), 'Color', 'blue','LineStyle','-'); hold on;
    plot3(zdir(1,:), zdir(2,:), zdir(3,:), 'Color', 'green','LineStyle','-'); hold on;
    
end

%% draw landmarks
landmarks = data.landmarks;
scatter3( landmarks(:, 1), landmarks(:, 2), landmarks(:, 3),'go' ); hold on;
%% draw observations
observedLandmark = [];
state = data.state;
for i=1:size(poses.position, 1)
    statePoseI = state(state(:,4)==i,:);        %All edges from poseI
    obsers = statePoseI(statePoseI(:,2)==2,:);  %edges from poseI to the observed landmarks by poseI
    
    if ~isempty (obsers)
        obserIndex_poseI = obsers(:,3);              % which landmarks are ovserved by poseI
        obser_poseI = landmarks(obserIndex_poseI,:); % coordinate of the observed landmarks
        observedLandmark = [observedLandmark;obser_poseI];
    end
end
plot3( observedLandmark(:, 1), observedLandmark(:, 2), observedLandmark(:, 3),'.','Color', [204,153,0]/255 ,'MarkerSize',20 ); hold on;

%% draw observation edges
%draw_edge_2d(data)
end