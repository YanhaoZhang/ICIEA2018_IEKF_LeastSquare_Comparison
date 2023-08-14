%%
%{
input Nods_array pose and landmark in the ground truth frame pose = [R_0i p_0_i0]
                                                             f    = [p_0_k0]
      Measurement_values feature in the frame of current pose relative_measurement = [p_i_ki]

%}


function [ ErrorVector, Jacobian_Node ] = RGBD_Factor( Nodes_array , Measurement_values )

pose=Nodes_array{1};
f=Nodes_array{2};

relative_measurement= Measurement_values;

R=pose(1:3,1:3);
p=pose(1:3,4);

ErrorVector =  R'*( f- p) - relative_measurement; % "14 Talk Note" P213 e_ij

Jacobian_Node{1}= [ R'*skew( f-p )  -R' ]; % Jacobian_Node{1} is a 3*6 matrix, [R'*skew( f )] and [-R']
                               % R'*skew( f ) the jacobian of e_ij to R
                               % -R'          the jacobian of e_ij to f
                               % R'           the jacobian of e_ij to relative_measurement

Jacobian_Node{2}= R';



end

