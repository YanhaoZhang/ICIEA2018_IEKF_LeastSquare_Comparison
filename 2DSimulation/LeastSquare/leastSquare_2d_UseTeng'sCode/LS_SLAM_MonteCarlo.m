function [] = LS_SLAM_MonteCarlo(t)


if nargin < 1
    t = 1;     % default is fifty times of monte carlo
end

for i=1:t
    disp(['Processing Monte Carlo ', int2str(i)]);
    filenameLoad_data = ['../../2d_datagen/data_2d/data_2d_',num2str(i), '.mat']; %load data file
    load(filenameLoad_data);
    filenameLoad_inl = ['../../2d_datagen/data_2d_InitialGuess/NoiseDataInPose0Frame_2d_',num2str(i), '.mat']; %load initial guess
    load(filenameLoad_inl); 
%     LS_SingleTime_estimation_result = LS_SLAM_2d(data,NoiseDataInPose0Frame);
%     filenameSave = ['./LS_SingleTime_result_2d/LS_SingleTime_estimation_result_', num2str(i),'.mat'];
%     save (filenameSave, 'LS_SingleTime_estimation_result');



    LS_estimation_result = LS_SLAM_OneMonteCarlo_2d(data,NoiseDataInPose0Frame);
    filenameSave = ['./LS_result_2d/LS_estimation_result_', num2str(i),'.mat'];
    save (filenameSave, 'LS_estimation_result');
end



end