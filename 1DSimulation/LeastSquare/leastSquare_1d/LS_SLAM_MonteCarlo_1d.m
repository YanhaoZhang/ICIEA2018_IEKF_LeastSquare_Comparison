function [] = LS_SLAM_MonteCarlo_1d(t)
%LS_SLAM_MonteCarlo_1d: Reading data and output estimation result.
%   Using function LS_SLAM_OneMonteCarlo_1d.

if nargin < 1
    t = 1;     % default is fifty times of monte carlo
end

for i=1:t
    disp(['Processing Monte Carlo ', int2str(i)]);
    filenameLoad_data = ['../../1d_datagen/data_1d/data_1d_',num2str(i), '.mat']; %load data file
    load(filenameLoad_data);
    filenameLoad_inl = ['../../1d_datagen/data_1d_InitialGuess/NoiseDataInPose0Frame_1d_',num2str(i), '.mat']; %load initial guess
    load(filenameLoad_inl); 

    LS_estimation_result = LS_SLAM_OneMonteCarlo_1d(data,NoiseDataInPose0Frame);
    filenameSave = ['./LS_result_1d/LS_estimation_result_', num2str(i),'.mat'];
    save (filenameSave, 'LS_estimation_result');
end



end