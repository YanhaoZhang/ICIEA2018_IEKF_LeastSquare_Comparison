function [ dimension ] = GetNodeTypeDimension_1d( NodeTypeName )
%GetNodeTypeDimension_1d: This is a simplify version of Teng's code for 1D case.
%   input'EdgeTypeName' is a string. It is used in function AddComplexEdge_1d

dimension=0;

switch NodeTypeName
    case 'Pose1'
        dimension = 1;
    case 'Landmark1'
        dimension = 1;
    otherwise
        sprintf('Warning: you have to define the new Node dimension in  GetNodeTypeDimension.m')
end

% switch NodeTypeName 
%     case 'Pose3'
%         dimension=6;
%     case 'LPose3'
%         dimension=6;    
%     case 'Pose2' 
%         dimension=3;
%     case 'Landmark3'
%         dimension=3;
%     case 'Landmark2'
%         dimension=2;    
%     case 'Velocity3'
%         dimension=3;
%     case 'IMUbias'
%         dimension=6;
%     case 'ParallaxPoint'
%         dimension=3;
% end
% 
% 
% if dimension==0
%    
%     sprintf('Warning: you have to define the new Node dimension in  GetNodeTypeDimension.m')
%     
% end

end

