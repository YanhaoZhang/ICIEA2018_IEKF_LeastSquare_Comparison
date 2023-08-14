function [ zi ] = ObservationModel_1d( position, fi )
%ObservationModel_1d: this is the observation model for 1D slam
%   in 1d slam, there is no rotation. 
%   input: fi:landmark coordiantion l_0_k0; position: p_0_i0

zi=(fi-position);


end

