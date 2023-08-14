function [data] = gen_data_1d( do_vis )
%gen_data_1d: Generate 1D data for 1D normal EKF
%   the parameters are set up in the file config_1d.m
close all;
%dim = 2;     % dimension
% generate data 
if nargin < 1
    do_vis = 1;
end

% addpath('../Lee/');
config_1d;



% step size
%t = 0:1:500;
t = 0:1:99;
poses = gen_trajectory_1d(t);

% draw 1d position
% plot(poses.position,0,'rx','markersize',10);hold on;
% if do_vis == 1
%     plot(poses.position,0,'rx','markersize',10);hold on;
%     % draw 1d local axis
%     axisl = 5;
%     for i = 1:size(poses.position, 1)
%         rotationi = [1; 0];                     % the local x-axis. In 1D case, it just has one axis
%         positioni = [poses.position(i,:) 0]';
%         xdir = positioni + axisl*rotationi;
%         xdir = [positioni, xdir];
% %         ydir = positioni + axisl*rotationi(:, 2);
% %         ydir = [positioni, ydir];
% %         plot(xdir(1), xdir(2), 'Color', 'red'); hold on;
% %         plot([0 xdir(1)],[0 xdir(2)], 'Color', 'red'); hold on;
%          plot(xdir(1,:), xdir(2,:), 'Color', 'red'); hold on;
%     end
% end

%% add random points
if ~exist('./data_1d/landmarks_1d.mat')
    landmarks = gen_landmark_1d;
    save('./data_1d/landmarks_1d', 'landmarks');
else
    load('./data_1d/landmarks_1d.mat');
end

% n_landmarks = N_LANDMARKS;
% if ~exist('./landmarks_1d.mat')
%     % generate new landmarks
%     border = 30;                    % the boder represents the coordinates lower and exceed min and max poses' coordinate
%     minpos = min(poses.position);        % min of poses' both coordinates x y
%     maxpos = max(poses.position);
%     minlm = minpos - border;
%     maxlm = maxpos + border;
%     landmarks(:, 1) = minlm(1)+rand(n_landmarks, 1)*(maxlm(1)-minlm(1));
%     %landmarks(:, 2) = minlm(2)+rand(n_landmarks, 1)*(maxlm(2)-minlm(2));
%     save('landmarks_1d', 'landmarks');
% else
%     % load landmarks
%     load('./landmarks_1d.mat');
% end

%% draw landmarks
% if do_vis == 1
%     plot(landmarks(:, 1), 0.5, 'go'); hold on;
% end

%% generate odom
odoms = [];
for i = 1:size(poses.position, 1)-1
    
    p_0_Next0 = poses.position(i+1,:)';
    p_0_I0    = poses.position(i,:)';
    translation_diff = p_0_Next0 - p_0_I0; %p_0_NextI = p_0_Next0 - p_0_I0
    
    if ADD_NOISE == 1
       noise_rand=randn;
        while(max(abs(noise_rand))> 1.5)
           noise_rand=randn;
        end
%%%%%%%%%%%%    Two types of noise    %%%%%%%%%%%%%%
            switch NOISE_TYPE
                case 1
                    noise_odo = [translation_diff]*SIGMA_ODOM.*noise_rand; % [theta;x;y] WHY time noises, not add them?
                case 0
                    noise_odo =  SIGMA_ODOM.*noise_rand; % [theta;x;y]
                otherwise
                        sprintf('Warning! Error occure in NOISE_TYPE. Please check config_2d.m.')
                        return;
            end
%%%%%%%%%%%%    Two types of noise    %%%%%%%%%%%%%%                                                                                                                                                   
       translation_diff=translation_diff+noise_odo;
    end
   
    odomi = [translation_diff]';   % just for looking the same as 2D or 3D cases. Actually we don't need to do this in 1D case
    odoms = [odoms; odomi];
end

%% generate observations
obsers = {};

for i = 1:size(poses.position, 1) % for pose
    position_0_i0 = poses.position(i,:)';                %p_0_i0 means in the frame pose0 the vetor of poseI->pose0
    %rotation_0i = poses.orientation((i-1)*2+1:i*2, :);   %R_0i   the rotation of poseI
    
    %framePoseI_x = rotation_0i(:, 1)';                   % x-axis of the poseI's frame. Here, it is assumed that orientation of camera is faced to x-axis
                                                         % I change z-axis to x-axis since in 2-D there is no z-axis
    
    obseri = [];
    for k = 1:size(landmarks, 1)               
        l_0_ki = landmarks(k, :)' - position_0_i0;       %landmarks n*2: the coordinate is in the frame of pose0. l_0_k0: vector from landmark_k->pose0
                                                         % l_0_ki means the vector from landmark_k->poseI in the frame of pose0
                                                         % Here, in 1D case, there is no local frame. since there is no rotation,
                                                         % the frame is just transpose in 1d direction
                                                         %In 1d case, norm is the same as abs. 
                                                       %Here, we cannot use abs or norm, since + or - represents the direction
                                                       %of observation. The orientation of local axis is backword in this case.
                                                       %just as 2D case.
%         if l_0_ki>0 && norm(l_0_ki)<MAX_RANGE          % >0 means it is on the right direction of local axis
        if norm(l_0_ki)<MAX_RANGE
            l_i_ki = l_0_ki;                            % in 1d case, l_i_ki is just l_0_ki, since 'ki' has represented all chagne from global frame to local frame.
            obserij = [k, l_i_ki'];                     % [k x]: k<- Index of landmarks; x<- coordinate of the observation in the frame of poseI, namely, l_i_ki
            if ADD_NOISE == 1
            noise_rand=randn;
             while(max(abs(noise_rand))> 1.5)
                noise_rand=randn;
             end
                %noise_ob=OBSV_NOISE^(1/2)*noise_rand;
                switch NOISE_TYPE
                    case 1
                        noise_ob = obserij(2)*SIGMA_OBSV.*noise_rand;
                    case 0
                        noise_ob =  SIGMA_OBSV.*noise_rand; % [x y]
                    otherwise
                        sprintf('Warning! Error occure in NOISE_TYPE. Please check config_2d.m.')
                        return;
                end
                obserij(2)=obserij(2)+noise_ob;  %change
            end
            obseri = [obseri; obserij];
        end
    end
    obsers{i} = obseri;
end
%title('3D Simulation Data Generator');
%% draw observations
% if do_vis == 1
%     observedLandmark = [];
%     for i=1:size(poses.position, 1)
%         if ~isempty (obsers{i})
%             obserIndex_poseI = obsers{i}(:,1);          % which landmarks are ovserved by poseI
%             obser_poseI = landmarks(obserIndex_poseI,:); % coordinate of the observed landmarks
%             observedLandmark = [observedLandmark;obser_poseI];
%         end
%     end
%     plot( observedLandmark, 0, 'g.','MarkerSize',20 ); hold on;
%     %plot( observedLandmark(:, 1), observedLandmark(:, 2), 'g.','MarkerSize',20 ); hold on;
% end

%% convert data format
% the format of data is the same as that of victoria park
indy = 1;
for i = 1:size(odoms, 1)
    % set observation
    for j = 1:size(obsers{i}, 1)
        ztfile(indy, 1) = obsers{i}(j, 2);        %obsers{i}: [landmarkI x] landmarkI<- index of landmark; x<- coordinate of landmark
        ztfile(indy, 2) = 2;
        ztfile(indy, 3) = obsers{i}(j, 1);
        ztfile(indy, 4) = i-1;
        indy = indy+1;
    end
    % set odometry
    for j = 1:1
        ztfile(indy, 1) = odoms(i, j); 
        ztfile(indy, 2) = 1;
        ztfile(indy, 3) = i;
        ztfile(indy, 4) = i-1;
        indy = indy+1;
    end  
end

% set observation for the last frame
index = length(obsers);
for j = 1:size(obsers{index}, 1)
    ztfile(indy, 1) = obsers{index}(j, 2); 
    ztfile(indy, 2) = 2;
    ztfile(indy, 3) = obsers{index}(j, 1);
    ztfile(indy, 4) = index-1;
    indy = indy+1;
end
data.state = ztfile;        % state vector   state stores the edges: pose-pose and pose-landmarks
data.obse_cov = OBSV_NOISE; % (Unused) observation covariance matrix
data.odom_cov = ODOM_NOISE; % (Unused) odometry covariance matrix
data.add_noise = ADD_NOISE;

data.noise_type = NOISE_TYPE;
data.odom_sigma = SIGMA_ODOM;
data.obsv_sigma = SIGMA_OBSV;

data.landmarks=landmarks;
data.poses=poses;
% save data_1d data

% clearvars -except data;
end



