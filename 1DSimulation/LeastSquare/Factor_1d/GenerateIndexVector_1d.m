function [ RowVecotr_ThisNode, ColVector_ThisNode ] = GenerateIndexVector_1d( CurrentTotalDimensionOfEdges, dimension_edge, dimension_node, NodeID_order )
%GenerateIndexVector_1d: This is the same as Teng's code. 
%   This function is for record the index of jacobian when calcualte in function PerformGO


 A= 1:1:dimension_edge;
 A= A';

 B= 1:1:dimension_node;
 
 RowMatrix=  repmat(A, 1, dimension_node )+ CurrentTotalDimensionOfEdges ;
 VectorMatrix = repmat( B, dimension_edge,1 ) +   (NodeID_order-1)*dimension_node;
 
 
 RowVecotr_ThisNode=RowMatrix(:);
 ColVector_ThisNode=VectorMatrix(:);
 
end

