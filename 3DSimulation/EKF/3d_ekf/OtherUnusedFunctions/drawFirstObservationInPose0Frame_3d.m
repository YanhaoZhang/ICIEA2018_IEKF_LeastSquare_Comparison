function [] = drawFirstObservationInPose0Frame_3d(firstObservation_pose0)
if nargin < 1
    load('../../3d_datagen/NoiseDataInPose0Frame_3d.mat');   % two dot means the previous folder, while one dot means the current folder
    firstObservation_pose0 = NoiseDataInPose0Frame;
end
poses = firstObservation_pose0.poses;

scatter3( poses.position(:,1), poses.position(:,2), poses.position(:,3),'k*' ); hold on;
%draw axis
axisl = 5;
for i = 1:size(poses.position, 1)
    rotationi = poses.orientation((i-1)*3+1:i*3, :);
    positioni = poses.position(i,:)';
    xdir = positioni + axisl*rotationi(:, 1);
    xdir = [positioni, xdir];
    ydir = positioni + axisl*rotationi(:, 2);
    ydir = [positioni, ydir];
%     plot(xdir(1,:), xdir(2,:), 'Color', [153,0,0]/255,'LineStyle','-'); hold on;
%     plot(ydir(1,:), ydir(2,:), 'Color', [0,0,153]/255,'LineStyle','-'); hold on;
    
    
    zdir = positioni + axisl*rotationi(:, 3);
    zdir = [positioni, zdir];
    plot3(xdir(1,:), xdir(2,:), xdir(3,:), 'Color', [153,0,0]/255,'LineStyle','-'); hold on;
    plot3(ydir(1,:), ydir(2,:), ydir(3,:), 'Color', [0,0,153]/255,'LineStyle','-'); hold on;
    plot3(zdir(1,:), zdir(2,:), zdir(3,:), 'Color', [0,153,0]/255,'LineStyle','-'); hold on;
    
end

%% draw landmarks
landmarks = firstObservation_pose0.landmarks;
%plot(landmarks(:, 1), landmarks(:, 2), 'Color',[51,102,0]/255,'LineStyle','o' ); hold on;
scatter3( landmarks(:, 2), landmarks(:, 3), landmarks(:, 3),[],[0,153,0]/255 ,'filled'); hold on;

end