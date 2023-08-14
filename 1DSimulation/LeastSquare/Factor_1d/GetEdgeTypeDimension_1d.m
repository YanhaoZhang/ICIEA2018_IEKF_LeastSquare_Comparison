function [ dimension ] = GetEdgeTypeDimension_1d( EdgeTypeName )
%GetEdgeTypeDimension_1d: This is a simplify version of Teng's code for 1D case.
%   input'EdgeTypeName' is a string.

dimension=0;
switch EdgeTypeName
    case 'RelativePose1_Factor'    % 1D pose-pose edge, namely odometry
        dimension = 1;
    case 'RGBD_1D_Factor'          % 1D pose-landmark edge, namely, observation
        dimension = 1;
    otherwise
        sprintf('Warning: you have not defined the new edge dimension in  GetEdgeTypeDimension.m')
end

% switch EdgeTypeName 
%     case 'PriorPose3_Factor'
%         dimension=6;
%     case 'PriorPose2_Factor'
%         dimension=3;    
%     case 'RelativePose3_Factor' 
%         dimension=6;
%     case 'RelativePose2_Factor' 
%         dimension=3;
%     case 'LinearVision_Factor'
%         dimension=2;
%     case 'IMU_Factor'
%         dimension=9;
%     case 'RGBD_Factor'
%         dimension=3;
%     case 'RGBD_2D_Factor'
%         dimension=2;
%     case 'Vision_Factor'
%         dimension=2;
%     case 'VisionPoseFixed_Factor'
%         dimension=2;       
%     case 'VisionTest_Factor'
%         dimension=3;
%     case 'VisionTestPoseFixed_Factor'
%         dimension=3;        
%     case 'IMUbias_Factor'
%         dimension=6;
%     case 'PriorVelAndbias_Factor'
%         dimension=9;
%     case 'VisionNoLandmark_Factor'
%         dimension=2;
%     case 'LinearVisionNoLandmark_Factor'
%         dimension=2;
%     case 'VisionPBA_Factor'     
%         dimension=2;
%     case 'VisionPBA_MainAnchor_Factor'     
%         dimension=2;
%     case 'VisionPBA_AssAnchor_Factor' 
%         dimension=2;
%     case 'VisionTestPBA_Factor'     
%         dimension=3;
%     case 'VisionTestPBA_MainAnchor_Factor'     
%         dimension=3;
%     case 'VisionTestPBA_AssAnchor_Factor' 
%         dimension=3;    
% end
% 
% 
% if dimension==0
%    
%     sprintf('Warning: you have not defined the new edge dimension in  GetEdgeTypeDimension.m')
%     
% end
end

