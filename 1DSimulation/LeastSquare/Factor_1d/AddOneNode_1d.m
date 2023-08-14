function [ Graph ] = AddOneNode_1d( Graph, NodeTypeName, NodeID )
%AddOneNode_1d: This is a simple version of Teng's code. It is used in AddComplexEdge_1d line 44.
%   If a node has been put into the graph, then we do nothing. If a edge condains new node, then we
%   put it into the graph, and set default values. It is useful since we add edges firstly and then we put the value of each node
%   into the graph
%   Inputs:
%         struct Graph
%         string NodeTypeName: 3-D or 2-D graph. 'Pose3' 'Landmark3' or 'Pose2' 'Landmark2'. see function SetNodeDefaultValue()
%         string NodeID:       the name of the pose and the landmarks that is observed by the pose. e.g. pose0 landmark1; pose1 landmark1 landmark2 ...

if  isfield(Graph.Nodes, NodeTypeName)                       % check if the type of the node is one of the following: Pose2 Pose3; Landmark2 Landmark3
    if  isfield(Graph.Nodes.(NodeTypeName).Values, NodeID)   % the Values are pose0 pose1 pose2 pose3; landmark1 landmark2 ... landmark10     
    else
        Graph.Nodes.(NodeTypeName).Values.(NodeID) = SetNodeDefaultValue_1d( NodeTypeName );
        
        if isfield(Graph.Fixed.IDname, NodeID)               % If this new node of this edge is set as fixed, then we need to add the number of times of the fixed node of this type (Pose1, Pose2, Pose3)
            Graph.Fixed.(NodeTypeName)= Graph.Fixed.(NodeTypeName)+1; %
        end
    end
else
    Graph.Nodes.(NodeTypeName).Dimension=GetNodeTypeDimension_1d( NodeTypeName );
    Graph.Nodes.(NodeTypeName).Jacobian.currentNumber_AllElementsInJacobian=0;
    Graph.Nodes.(NodeTypeName).Jacobian.ValueVector=[];
    Graph.Nodes.(NodeTypeName).Jacobian.RowVector=[];
    Graph.Nodes.(NodeTypeName).Jacobian.ColVector=[];    
    Graph.Nodes.(NodeTypeName).Values.(NodeID) = SetNodeDefaultValue_1d( NodeTypeName );
    Graph.Fixed.(NodeTypeName)= 0;
    
    if isfield(Graph.Fixed.IDname, NodeID)            
       Graph.Fixed.(NodeTypeName)= Graph.Fixed.(NodeTypeName)+1;      %NOTICE this process is used as Loop closing for pose? if a pose's fixedtypename is larger than one, it means it has been rereached more than one times, 
                                                                      %for a landmark, this means how many times the landmark has been observed
    end
end
end

function [ NodeValue ] = SetNodeDefaultValue_1d( NodeTypeName )
%SetNodeDefaultValue_1d: A simple version of Teng's code. To set the default value of a node in 1D case
%   

switch NodeTypeName
    case 'Pose1'         %1D pose
        NodeValue = [0];
    case 'Landmark1'
        NodeValue = [0]; %1D landmark
    otherwise
        sprintf('Warning: you have not defined the default value for the new Node in SetNodeDefaultValue_1d.m')
        sprintf('         Please check file AddOneNode_1d.m')
end

% flag=0; %The idea of using 'flag' is owesome
% 
% switch NodeTypeName 
%     case 'Pose3'
%         NodeValue=[eye(3 ) zeros(3,1)]; flag=1;
%     case 'LPose3'
%         NodeValue=[eye(3 ) zeros(3,1)]; flag=1;    
%     case 'Pose2' 
%         NodeValue=[eye(2) zeros(2,1)]; flag=1;
%     case 'Landmark3'
%         NodeValue=zeros(3,1); flag=1;
%     case 'Velocity3'
%         NodeValue=zeros(3,1); flag=1;
%     case 'IMUbias'
%         NodeValue=zeros(6,1); flag=1;
%     case 'Landmark2'
%         NodeValue=zeros(2,1); flag=1;
%     case 'ParallaxPoint'
%         NodeValue=[0;0;1; 0]; flag=1;
% end
% 
% if flag==0
%     sprintf('Warning: you have not defined the default value for the new Node in  SetNodeDefaultValue.m')
% end
end
