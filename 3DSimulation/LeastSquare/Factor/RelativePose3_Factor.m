%% 
%{
RelativePose3_Factor needs 6*6 infromation matrix. You cannot get it here,
but will catch a little bit hint in the PerformGO.m function. Actually it
just use the edge.{...}.measurement.values here. You can get the direct
instruction in the file GetEdgeTypeDimension
%}

function [ ErrorVector, Jacobian_Node ] = RelativePose3_Factor( Nodes_array , Measurement_values )

NodeA=Nodes_array{1};         % The ground truth coordinate of the node in the first end
NodeB=Nodes_array{2};         % The ground truth coordinate of the node in the other end


R_A=NodeA(1:3,1:3); p_A=NodeA(1:3,4);
R_B=NodeB(1:3,1:3); p_B=NodeB(1:3,4);
R_u=Measurement_values(1:3,1:3); p_u=Measurement_values(1:3,4);


dR= R_A'*R_B*R_u';
dP= -R_A'*R_B*R_u'*p_u + R_A'*( p_B-p_A );


dtheta=so3_log( dR  );
dp = jacor_inverse(-dtheta)*dP;

ErrorVector=[ dtheta;  dp ];

JrE=Jacobian_Lie_inverse(-ErrorVector);
%JrE=eye(6);


X_Ainv=[R_A'  -R_A'*p_A ];
adX_Ainv= adjoint( X_Ainv ) ;

Jacobian_NodeA=-JrE*adX_Ainv;
Jacobian_NodeB=JrE*adX_Ainv;

Jacobian_Node{1}=Jacobian_NodeA;
Jacobian_Node{2}=Jacobian_NodeB;


end

