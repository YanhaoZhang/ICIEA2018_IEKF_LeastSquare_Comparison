%% intruduction
%{
This function is used in the function AddComplexEdge() line 44.
Inputs
    struct Graph
    string NodeTypeName: 3-D or 2-D graph. 'Pose3' 'Landmark3' or 'Pose2' 'Landmark2'. see function SetNodeDefaultValue()
    string NodeID:       the name of the pose and the landmarks that is observed by the pose. e.g. pose0 landmark1; pose1 landmark1 landmark2 ...
%}


function [ Graph ] = AddOneNode( Graph, NodeTypeName, NodeID )



if  isfield(Graph.Nodes, NodeTypeName)                       % check if the type of the node is one of the following: Pose2 Pose3; Landmark2 Landmark3
    if  isfield(Graph.Nodes.(NodeTypeName).Values, NodeID)   % the Values are pose0 pose1 pose2 pose3; landmark1 landmark2 ... landmark10     
    else
        Graph.Nodes.(NodeTypeName).Values.(NodeID) = SetNodeDefaultValue( NodeTypeName );
        
        if isfield(Graph.Fixed.IDname, NodeID)
            
            Graph.Fixed.(NodeTypeName)= Graph.Fixed.(NodeTypeName)+1; %NOTICE WHAT IS IT FOR? how many times that this type of Node has been observed?
        end    
        
    end
else
    Graph.Nodes.(NodeTypeName).Dimension=GetNodeTypeDimension( NodeTypeName );
    Graph.Nodes.(NodeTypeName).Jacobian.currentNumber_AllElementsInJacobian=0;
    Graph.Nodes.(NodeTypeName).Jacobian.ValueVector=[];
    Graph.Nodes.(NodeTypeName).Jacobian.RowVector=[];
    Graph.Nodes.(NodeTypeName).Jacobian.ColVector=[];    
    Graph.Nodes.(NodeTypeName).Values.(NodeID) = SetNodeDefaultValue( NodeTypeName );
    Graph.Fixed.(NodeTypeName)= 0;
    
    if isfield(Graph.Fixed.IDname, NodeID)            
       Graph.Fixed.(NodeTypeName)= Graph.Fixed.(NodeTypeName)+1;      %NOTICE this process is used as Loop closing for pose? if a pose's fixedtypename is larger than one, it means it has been rereached more than one times, 
                                                                      %for a landmark, this means how many times the landmark has been observed
     end
end


end

