function [ Graph ] = PerformGO( Graph )
%%%%%%%%%%%%%% generate the sparse Jaocbian matrix and the error vecotr
Total_ErrorVector=zeros( Graph.TotalDimensionOfEdges, 1 );       %%%%%  Initialize the Error Vector 
Number_NodeTypes=size( fields(Graph.Nodes) ,1 );
Total_Jacobian=struct;                                           
NodeTypeNamesArray = fields(Graph.Nodes);                        % Pose3 Landmark3
                                                                 % test the obove and will know it
% 00

num_update=0;
for i=1:Number_NodeTypes 
    Total_Jacobian.( NodeTypeNamesArray{i} )=[];
    dimension= Graph.Nodes.( NodeTypeNamesArray{i}  ).Dimension;
    num_node_thisKind = size( fields (Graph.Nodes.( NodeTypeNamesArray{i}  ).Values) , 1) ; % how many nodes of Pose2 or Landmark2
    
     if isfield(Graph.Fixed, NodeTypeNamesArray{i})           % if this node is set to be a fixed node
        num_node_thisKind = num_node_thisKind - Graph.Fixed.(NodeTypeNamesArray{i});
    end
    
    Graph.Nodes.( NodeTypeNamesArray{i}  ).updateIndex = num_update+ [1:1: dimension* num_node_thisKind ]';  %NOTICE WHAT IS IT FOR?
    num_update=num_update+ dimension* num_node_thisKind;
end

NumberofEdges=size(Graph.Edges, 2);
NeedOptimization=true;
iter_optimization=0;

while ( NeedOptimization  && iter_optimization < Graph.Parameters.iter   )   %NOTICE learn this way to set up maximum iteration steps
    for iter_edge=1:NumberofEdges
        NeedOptimization=false;
    
    
        edge =Graph.Edges{iter_edge};              % the edge from first to last
        edgeType=edge.EdgeType;
    
    
    
                                                                 % There is no difference between "fieldnames(edge.withNodes)" and "fields(edge.withNodes)"
        num_nodes_ThisEdge= size( fieldnames(edge.withNodes),1); % The number of nodes connected by this edge
        Nodes_array=cell(num_nodes_ThisEdge, 1 );  
        nodes_id = fields(edge.withNodes);
        for i = 1: num_nodes_ThisEdge
           node_id  = nodes_id{i};                               % the point of edge.withNodes.(node_id).NodeTypeName is 
           node_type= edge.withNodes.(node_id).NodeTypeName;    
           Nodes_array{i}=Graph.Nodes.(node_type).Values.(node_id); % to connect node_id (from edge) and the node value (in node), namely, Graph.Nodes.(node_type).Values.(node_id)
        end
        FuncFactor=str2func(edgeType);       % here, before it, edgeType is a string, e.g. 'RGBD_Factor' 'Vision_Factor' 'RelativePose3_Factor'...
                                             % str2func()changes the string to the name of a function. The
                                             % defination of the function are all include in the folder Factor
        temp_value = edge.Measurement.value; % The value of edge (odometry, feature, pixel, ... )
        [ ErrorVector, Jacobian_Node ]=FuncFactor( Nodes_array, temp_value  ); % Here, the "ErrorVector" is for debuging
    
        Total_ErrorVector( edge.ErrorVectorIndex  )=   edge.Measurement.inf_sqrt* ErrorVector; % Here we can see the usuage of edge.ErrorVectorIndex. just record the error. 
                                                                                               % Total_ErrorVector is also used as cost.
                                             % Also from here, we can see that the Jacobian is calculated in "edge FuncFactor" function

        %% debug
        error_s =  edge.Measurement.inf_sqrt* ErrorVector;
        Error_s = error_s'*error_s;
   
        %% Dealing with fixed node         % here, we calcualte the Jacobian_Node for all nodes, no matter whether it is fixed.
        for i = 1: num_nodes_ThisEdge      % but when put into: Graph.Nodes.(node_type).Jacobian.ValueVector( Jacobianindex_thisEdge_thisNode )
            node_id  = nodes_id{i};        % we do not add fixed nodes. Also, edge.withNodes.(node_id).JacobianIndex do not consider fixed node
            node_type= edge.withNodes.(node_id).NodeTypeName;    
    
            if ~isfield( Graph.Fixed.IDname,  node_id)    % if the current node is not fixed node, then its edge has edge.withNodes.(node_id).JacobianIndex
                Jacobian_Node_i_infsqrt=edge.Measurement.inf_sqrt*Jacobian_Node{i};     % edge.Measurement.inf_sqrt is calculated by edge.Measurement.inf which is set in the step of add edge, e.g. Measurement.inf   = eye(6);
                Jacobianindex_thisEdge_thisNode=edge.withNodes.(node_id).JacobianIndex; % NOTICE WHERE IS IT COME FROM edge.withNodes.(node_id).JacobianIndex? see AddComplexEdge function
                Graph.Nodes.(node_type).Jacobian.ValueVector( Jacobianindex_thisEdge_thisNode )=Jacobian_Node_i_infsqrt(:);
            end   
    
        end      
   
    end


%       sprintf('cost =  %f\n',Total_ErrorVector'*Total_ErrorVector )



    Whole_Jacobian=[];   % Xiang.Gao's book Page251 Fig-10-6 J. The whole Jacobian
    for i=1:Number_NodeTypes 
        ThisTypeName=NodeTypeNamesArray{i};
        all_ids_thisType= fields(  Graph.Nodes.( ThisTypeName ).Values  );
        num_ids_thisType= size ( all_ids_thisType, 1 );
    
        num_ids_thisType  =  num_ids_thisType - Graph.Fixed.(ThisTypeName);    % new

        num_cols=num_ids_thisType*Graph.Nodes.(ThisTypeName).Dimension;
        num_rows=Graph.TotalDimensionOfEdges;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Graph.Nodes.(node_type).Jacobian.ValueVector: the Jacobian of Pose2 and Landmark2
%%% The order of it is according to the order we observe this node, namely the order we add the edge.
%%% e.g. (pose0 landmark1) (pose0 landmark2) (pose0 pose1) (pose1 landmark2)
%%% then the order of Graph.Nodes.Pose2.Jacobian.ValueVector: J_ep0p1_p1(9), J_ep1l2_p1(3)
    
    
        Total_Jacobian.( NodeTypeNamesArray{i} ) = sparse( Graph.Nodes.(ThisTypeName).Jacobian.RowVector,...
                                                           Graph.Nodes.(ThisTypeName).Jacobian.ColVector,...
                                                           Graph.Nodes.(ThisTypeName).Jacobian.ValueVector,...
                                                           num_rows, num_cols   );
        Whole_Jacobian=[Whole_Jacobian  Total_Jacobian.( NodeTypeNamesArray{i} )];
    end
%  a = sparse( Graph.Nodes.(ThisTypeName).Jacobian.RowVector,...
%      Graph.Nodes.(ThisTypeName).Jacobian.ColVector,...
%      Graph.Nodes.(ThisTypeName).Jacobian.ValueVector);
%  
%  
%  , num_rows, num_cols   )
%  b = full(Total_Jacobian.( NodeTypeNamesArray{2} ));
%  a = full(Whole_Jacobian)

    if strcmp(Graph.Parameters.OptimizationMethod,'GN')
        dX= -  (Whole_Jacobian'*Whole_Jacobian)\(Whole_Jacobian'*Total_ErrorVector);  % this is the incremental vector
    else
         H = Whole_Jacobian'*Whole_Jacobian;  mm=size(H,1); 
        %lambda = 0.2;
         dX = (speye(mm)*lambda-H)\(Whole_Jacobian'*Total_ErrorVector);   %LM method
    end    
    




%%%%% update_state
    for i=1:Number_NodeTypes 
        ThisTypeName=NodeTypeNamesArray{i};
        dx = dX (Graph.Nodes.( ThisTypeName ).updateIndex);
        dimension_thisType= Graph.Nodes.( ThisTypeName ).Dimension;
    
        all_ids_thisType= fields(  Graph.Nodes.( ThisTypeName ).Values  );
        num_ids_thisType= size ( all_ids_thisType, 1 );
    
        PreFixed_num = 0;
        for k=1:num_ids_thisType
            thisNode_id = all_ids_thisType{k};
    
            if ~isfield(Graph.Fixed.IDname,  thisNode_id)  
                %%increment =  dx ( dimension_thisType*(k-1)+1  : dimension_thisType*k  );    %%%       
                increment =  dx ( dimension_thisType*(k-1-PreFixed_num)+1  : dimension_thisType*(k-PreFixed_num)  );           
    
                Z= abs( increment )-Graph.Parameters.LinearizedError.(ThisTypeName);
                if any(Z>0)
                   NeedOptimization=true;
                end   
    
                Graph.Nodes.( ThisTypeName ).Values.( thisNode_id ) = update_state( ThisTypeName, Graph.Nodes.( ThisTypeName ).Values.( thisNode_id ) , increment); 
            else
                PreFixed_num = PreFixed_num+1;
            end      
        end
   
    end

    iter_optimization=iter_optimization+1;
end

%% Get covariance of the whole graph. It does not contain that of fixed node, say pose0
Graph.cov = inv(Whole_Jacobian'*Whole_Jacobian);

end
