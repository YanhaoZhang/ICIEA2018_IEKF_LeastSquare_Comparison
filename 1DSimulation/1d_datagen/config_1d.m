%%%%%%%%%%%%%%%
%%% This is nearly a copy from Teng's EKF-Manifold code: see datagen_3d/conf.m
%%%%%%%%%%%%%%%
% configuration 


N_LANDMARKS = 10;      % number of landmarks

ADD_NOISE = 1;          % add noise or not to odometry and observation: 1 add noise, 0 dont add noise

% percentage of noise
NOISE_TYPE = 0;        % if 0 then noise = sigma*randn; if 1 then noise = sigma*randn*dataValue.
SIGMA_ODOM = 0.20;
SIGMA_OBSV = 0.20;

ODOM_NOISE = 2*diag([0.05]);        % odometry noise cov matrix should be 3*3
OBSV_NOISE = 2*0.05^2*eye(1);       % observation noise, assume RGB-D sensor and dx, dy measurement


% sensor config FOV
%MAX_DEGREE = 1*pi/2;        % maximum field of view        (maximum angle)  represent half of the angel of the visual cone (maixmum pi/2)
MAX_RANGE  = 3.0;          % maximum range of the sensor  (maximum radius)