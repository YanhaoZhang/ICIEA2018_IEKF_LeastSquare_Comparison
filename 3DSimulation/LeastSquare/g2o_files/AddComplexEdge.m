%% 
%{ 
Introduction
input
    struct Graph:                 see initializeGraph
    string EdgeTypeName:          'LinearVision_Factor' or 'Vision_Factor'
    cell   NodeArray_TypeIncluded cell(2,2)
    struct Measurement            see initializeGraph (value inf inf_sqrt)


%}


function [ Graph ] = AddComplexEdge( Graph, EdgeTypeName, NodeArray_TypeIncluded,  Measurement )


    Measurement.inf_sqrt=(Measurement.inf)^(1/2) ;
    dimension_edge  = GetEdgeTypeDimension( EdgeTypeName );  %NOTICE WHAT IS THE DEIMENTION OF EDGE ? 
    SizeEdges=size(Graph.Edges,2);                           %cell Graph.Edges 1*40, size(,2) return 40
                                                             %if debuging, you will figure out that after initializing, the figure edge is
                                                             %an empty struct. here Teng is using a way like push_back() of an vector of C++
    ThisEdgeOrder=SizeEdges+1;
    Graph.Edges{ThisEdgeOrder}.EdgeType=EdgeTypeName;                           %begin: step: end
    Graph.Edges{ThisEdgeOrder}.ErrorVectorIndex=  Graph.TotalDimensionOfEdges + [1    : 1   : dimension_edge   ]'; % NOTICE WHAT IS THIS FOR? It is for debuging, see PerformGO() Total_ErrorVector( edge.ErrorVectorIndex  )=   edge.Measurement.inf_sqrt* ErrorVector;
    Graph.Edges{ThisEdgeOrder}.Measurement=Measurement;
    Graph.Edges{ThisEdgeOrder}.OrderInEdges=SizeEdges+1;    % SizeEdges is currently = 0
                                                            %NOTICE form debugging we can figure out that in matlab, we dont need to
                                                            %design the object of a structure at the begining, we can add it
                                                            %later,while in C++, we have to settle down the whole objects in a class or a struct.
    
    
CurrentTotalDimensionOfEdges=Graph.TotalDimensionOfEdges;    
num_Node=size( NodeArray_TypeIncluded, 1 );                 % vector<struct> NodeArray_TypeIncluded (1+n)*2. In the
                                                            % NodeArray_TypeIncluded, the first row is the current pose (3-D or 2-D, and the pose's name)
                                                            % the following row is the landmarks observed by this pose (3-D or 2-D, and the landmark's name)
                                                            % num_Node is how many rows in the NodeArray_TypeIncluded. The first one is
                                                            % the pose, and the following is the landmarks observed by the pose.
num_fixNodes_inthisedge_now = 0;
for i=1: num_Node
   
    
    Node_typename_i=NodeArray_TypeIncluded{i,1};            % 'Pose3' or 'Pose2'; 'Landmark3' or 'Landmark2'
      Node_id_i=NodeArray_TypeIncluded{i,2};                % the name of the pose and the landmarks. e.g. pose0, landmark1
     [ Graph ] = AddOneNode( Graph, Node_typename_i, Node_id_i );    
     Graph.Edges{ThisEdgeOrder}.withNodes.(Node_id_i).NodeTypeName=Node_typename_i; % pose0 is Pose3 (3-D) landmark1 is Landmark3(3-D)
    
   field_array_i=fields(Graph.Nodes.(Node_typename_i).Values);
   NodeID_order_i= find(strcmp(field_array_i, Node_id_i));  % the order of the current node. e.g. the input is pose3 then the NodeID_order_i=4
   %NodeID_order_i = NodeID_order_i - Graph.Fixed.(Node_typename_i); %%  the order in this edge   
   
%    sprintf( Node_id_i);
%    fprintf( NodeID_order_i);
   
   FixedNumberInthisEdgeInThisNodeType = GetFixedNumber(Graph.Fixed.IDname, field_array_i, NodeID_order_i );
   NodeID_order_i = NodeID_order_i - FixedNumberInthisEdgeInThisNodeType;
   
   %Graph.Edges{ThisEdgeOrder}.withNodes.(Node_id_i).OrderInThisType=NodeID_order_i;
   
   
   
   if ~isfield(Graph.Fixed.IDname,Node_id_i)
   
   dimension_node_i  = GetNodeTypeDimension( Node_typename_i );
   
    [ RowVecotr_Node_i, ColVector_Node_i ] = GenerateIndexVector( CurrentTotalDimensionOfEdges, dimension_edge, dimension_node_i, NodeID_order_i );
   
    if min(RowVecotr_Node_i)<0 || min(ColVector_Node_i)<0
        
       fprintf('warning! RowVecotr_Node_i or ColVector_Node_i < 0 ');
        
    end
    
    
    Graph.Nodes.(Node_typename_i).Jacobian.RowVector=[ Graph.Nodes.(Node_typename_i).Jacobian.RowVector;  RowVecotr_Node_i];
    Graph.Nodes.(Node_typename_i).Jacobian.ColVector=[Graph.Nodes.(Node_typename_i).Jacobian.ColVector;  ColVector_Node_i ];        
    num= Graph.Nodes.(Node_typename_i).Jacobian.currentNumber_AllElementsInJacobian;
    index_i = num+[1:1: dimension_edge*dimension_node_i ]';    
    Graph.Edges{SizeEdges+1}.withNodes.(Node_id_i).JacobianIndex=index_i;    
    Graph.Nodes.(Node_typename_i).Jacobian.currentNumber_AllElementsInJacobian=Graph.Nodes.(Node_typename_i).Jacobian.currentNumber_AllElementsInJacobian+dimension_edge*dimension_node_i;
    
   end
end




Graph.TotalDimensionOfEdges=Graph.TotalDimensionOfEdges+dimension_edge;

end

function    FixedNumberInthisEdgeInThisNodeType = GetFixedNumber(GraphFixedIDname, field_array_i, NodeID_order_i )

if NodeID_order_i == 1
    FixedNumberInthisEdgeInThisNodeType = 0;
else    
    FixedNumberInthisEdgeInThisNodeType = 0;
    for  j = 1: NodeID_order_i-1
        
        if isfield( GraphFixedIDname, field_array_i{j})
            FixedNumberInthisEdgeInThisNodeType = FixedNumberInthisEdgeInThisNodeType+1;
        end
    end
end

end


