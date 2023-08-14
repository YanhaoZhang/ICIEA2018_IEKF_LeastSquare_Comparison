function [] = drawNoiseDataInPose0Frame_1d(NoiseDataInPose0Frame)
%drawNoisedGroundTruth_1d: draw the noise data calcualted by the noise odometry and observation.
%   draw the plots in the line y=1.0, not y=0

if nargin < 1
    load('../../1d_datagen/NoiseDataInPose0Frame_1d.mat');   % two dot means the previous folder, while one dot means the current folder
end

poses = NoiseDataInPose0Frame.poses;
plot( poses.position, 1, 'k*' ); hold on;

%draw axis
% axisl = 5;
% for i = 1:size(poses.position, 1)
%     rotationi = [1; 0];                     % the local x-axis. In 1D case, it just has one axis
%     positioni = [poses.position(i,:) 1]';
%     xdir = positioni + axisl*rotationi;
%     xdir = [positioni, xdir];
%     plot(xdir(1,:), xdir(2,:), 'Color', [153,0,0]/255,'LineStyle','-'); hold on;
% end
%% draw landmarks
landmarks = NoiseDataInPose0Frame.landmarks(:,2);
%plot(landmarks(:, 1), landmarks(:, 2), 'Color',[51,102,0]/255,'LineStyle','o' ); hold on;
%scatter( landmarks(:, 1), landmarks(:, 2), [],[0,153,0]/255 ,'filled'); hold on;

plot( landmarks, 1, 'Color',[0,153,0]/255 ,'Marker','.','MarkerSize',20 ); hold on;
ylim([-9 9]);hold on

end