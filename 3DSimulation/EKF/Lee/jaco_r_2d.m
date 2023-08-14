function [result] = jaco_r_2d( dtheta )
%jaco_r_2d: Calculate the right jacobian in 2D case.
%   This function is a sinple version of Teng's jaco_r which is for 3D and higher cases
%   For more details, please refer to paper: 'Convergence and consistency analysis for a 3D invariant-EKF SLAM' Eq. 16,
%   and paper 'An EKF-SLAM algorithm with consistency properties' Eq. 22, where B(alpha) equal to left jacobian = jaco_r_2d( -alpha )

if dtheta < 0.00000001
    result = eye(2);
else
    result = sin(dtheta)/dtheta *eye(2) - (1-cos(dtheta))/dtheta*[0 -1;1 0];
    
    % This is equal to: eye(2,2) - (1-cos(dtheta))*skew(dtheta)/dtheta^2 + (dtheta-sin(dtheta))*skew(dtheta)^2/norm(dtheta)^3;
    
end
end

