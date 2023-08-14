function [ ErrorVector, Jacobian_Node ] = RelativePose1_Factor( Nodes_array , Measurement_values )
%RelativePose1_Factor: Calculate the jacobian of 1D pose-pose edge
%   This is a simple version of RelativePose2_Factor for 1D data. 
%{
Here is the instruction of 2D case:

This is a second version of Teng's PriorPose2_Factor.m
For reconciling with the 3D measurement_value, I change a little of theta and R_u

For more details, NodeA and NodeB are two poses (rotation and position).
R_A: R_01, p_A: p_0_10; R_B: R_02, p_B: p_0_20; R_u: R_12, v: p_1_21

the small change deltaR dR = R_01'*R_02*R_12, and equals 0 if there is no
error. deltaP = R_01'*(p_0_20-p_0_10)-p_1_21, and equals 0 if there is no
error.

Nodes_array: the value of initial pose in ground truth: [R_00 p_0_00]->[R_01 p_0_10]; [R_01 p_0_10]->[R_02 p_0_20];...
Measurement_values: the value of odometry:              [R_01 p_0_10];                [R_12 p_1_21];  
%}

NodeA=Nodes_array{1};
NodeB=Nodes_array{2};
% R_A=NodeA(1:2,1:2); p_A=NodeA(1:2,3);
% R_B=NodeB(1:2,1:2); p_B=NodeB(1:2,3);
% R_u=Measurement_values(1:2,1:2);
% v=Measurement_values(1:2,3);
% % theta=Measurement_values(3); v=Measurement_values(1:2);
% % R_u=[cos(theta)  -sin(theta ); sin(theta)  cos(theta) ];
% 
% dR= R_A'*R_B*R_u';        %This is the e_ij'e_ij
% dP= R_A'*( p_B-p_A )-v; 

% 1D has no rotation
p_A=NodeA(1);
p_B=NodeB(1);
v=Measurement_values(1);       
dP= -p_A + p_B - v;         %This is the e_ij'e_ij
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Here it is use initial guess to minums odometry: p_1_21 - p_1_21. The first one is calculated from initial guess, the second
%%% one is what we get from the odometry. Similarly, R_A'*R_B calculates the R_12 from the initial guess, and R_12*R_u means we
%%% calcualte the difference of R_12-R_u, not R_u-R_12.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% dtheta= atan2( dR(2,1), dR(1,1)  )  ;

% ErrorVector=[ dP ; dtheta  ];  % different from 3D case. And p is the first two
% 
% J=[0  -1; 1 0];
% Jacobian_NodeA =  [ -J*R_A'*( p_B-p_A )   -R_A';  -1   zeros(1,2) ];
% Jacobian_NodeB =   [  zeros(2,1)  R_A';  1   zeros(1,2)  ];


%%%%%%%%%%% change
% ErrorVector=[dtheta;  dP ];  % different from 3D case. And p is the first two
% J=[0  -1; 1 0];
% Jacobian_NodeA = [-1, zeros(1,2); -J*R_A'*( p_B-p_A ), -R_A'];
% Jacobian_NodeB = [ 1, zeros(1,2);     zeros(2,1),       R_A'];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ErrorVector=[dP ];  % different from 3D case. And p is the first two
Jacobian_NodeA = [-1];
Jacobian_NodeB = [ 1];

Jacobian_Node{1}=Jacobian_NodeA;
Jacobian_Node{2}=Jacobian_NodeB;
end

