%% Function: Initialize Graph
%% Introductions:
%{
1. struct Graph:
    struct Parameters            
    struct Fixed                 
    struct Nodes                 
    cell   Edges                 
	int    TotalDimensionOfEdges 
    double Edge_error            
    int    iter_actual           
    
    1.1 struct Parameters: 
         string OptimizationMethod 'GN'
         struct LinearizedError     
         double tau                 1e-5
         double eplison             1e-3
         double eplison_1           1e-4
         double eplison_2           1e-4
         double eplison_3           1e-4
         int    delta               1     
         int    iter                250

         1.1.1 struct LinearizedError
                %for 3-D
                vector<double> Pose3       [1e-4;1e-4;1e-4;1e-3;1e-3;1e-3]
                vector<double> LPose3
                vector<double> Velocity3   [1e-3;1e-3;1e-3]
                vector<double> IMUbias
                vector<double> Landmark3   [1e-2;1e-2;1e-2]
                
                %for 2-D
                vector<double> Pose2
                vector<double> Landmark2
                vector<double> ParallaxPoint

     1.2 struct Fixed
          struct IDname
          int    Pose3      1
          int    Landmark3  0

          1.2.1 struct IDname
                 int pose0  1

     1.3 struct Nodes
          struct Pose3
          struct Landmark3

          1.3.1 struct Pose3
                 int         Dimension   6
                 struct      Jacobian
                 struct      Values
                 vector<int> updateIndex [1:18]'
                    
                 1.3.1.1 struct jacobian
                          int currentNumber_AllElementsInJacobian 360
                          vector<double> ValueVector
                          vector<int>    RowVector
                          vector<int>    ColVector
                 1.3.1.2 struct Values
                          matrix<double> pose0 (3*4 represents Rotation and Translation(location)) 
                          matrix<double> pose1
                          matrix<double> pose2
                          matrix<double> pose3

     1.4 cell Edges  %each element is a struct that contains:
          string      EdgeType         'Vision_Factor'
          vector<int> ErrorVectorIndex [i,i+1] (i=1,2,...,80)
          struct      Measurement
          int         OrderInEdges     1,2,...,40
          struct      withNodes

          1.4.1 struct Measurement
                 vector<double> value    see Vision_Example_Small.m 
                                         value (vector<double> 2*1) is the image coordinate of observed landmark (f{i}) by the camera at current pose (pose0)
                 matrix<int>    inf      [1 0;0 1]
                 matrix<int>    inf_sqrt [1 0;0 1]

          1.4.2 struct withNodes   
                 struct pose0        which pose is changing
                 struct landmark1    which landmark is changing
                 
                 1.4.2.1 struct pose1
                          string      NodeTypeName 'Pose3' a 3-D pose
                          vector<int> jacobianIndex (12*1)

                 1.4.2.2 struct landmark1
                          string      NodeTypeName 'Landmark3' a 3-D ~
                          vector<int> jacobianIndex (6*1)
%}
%%
function [ Graph ] = InitializeGraph_1d
%InitializeGraph_1d: initialize graph for 1D data.
%   this is a simplified version of Teng's InitializeGraph



Graph.Parameters.OptimizationMethod='GN';
% Graph.Parameters.LinearizedError.Pose3 = [0.0001; 0.0001; 0.0001; 0.001; 0.001; 0.001 ] ;
% Graph.Parameters.LinearizedError.LPose3 = [0.0001; 0.0001; 0.0001; 0.001; 0.001; 0.001 ] ;
% 
% 
% 
% Graph.Parameters.LinearizedError.Velocity3 = [0.001; 0.001; 0.001 ] ;
% Graph.Parameters.LinearizedError.IMUbias = [0.0001; 0.0001; 0.0001; 0.001; 0.001; 0.001 ] ;
% 
% 
% Graph.Parameters.LinearizedError.Landmark3= [0.01; 0.01 ; 0.01 ];
% 
% Graph.Parameters.LinearizedError.Pose2 = [0.0001; 0.001; 0.001 ] ;
% Graph.Parameters.LinearizedError.Landmark2= [0.001; 0.001 ];
% Graph.Parameters.LinearizedError.ParallaxPoint= [0.0001; 0.0001;0.0001 ];

Graph.Parameters.LinearizedError.Pose1 = [ 0.001 ] ;
Graph.Parameters.LinearizedError.Landmark1= [ 0.001 ];


% Graph.Parameters.tau = 1e-5;
% Graph.Parameters.eplison = 1e-3;
% 
% Graph.Parameters.eplison_1 = 1e-4;
% Graph.Parameters.eplison_2 = 1e-4;
% Graph.Parameters.eplison_3 = 1e-4;
% Graph.Parameters.delta = 1;


Graph.Parameters.iter= 250; %The maximum step of optimization iteration

Graph.Fixed=[];
Graph.Fixed.IDname=[];

Graph.Nodes=[];
Graph.Edges=[];
Graph.TotalDimensionOfEdges=0;
end

