function [] = gen_MonteCarlo_InitialGuess_1d(t)
%gen_MonteCarlo_InitialGuess_2d: 
%   generate a bunch of initial guess using function ObservationToPose0Frame_2d

if nargin < 1
    t = 1;     % default is 50 times of monte carlo
end

for i=1:t
    disp(['Processing Monte Carlo ', int2str(i)]);
    filenameLoad = ['./data_1d/data_1d_', num2str(i),'.mat'];   %read file
    load(filenameLoad);
    
    NoiseDataInPose0Frame = ObservationToPose0Frame_1d(data);
    filenameSave = ['./data_1d_InitialGuess/NoiseDataInPose0Frame_1d_', num2str(i),'.mat'];
    save (filenameSave, 'NoiseDataInPose0Frame');
end

end