function [Estimation_X] = RI_EKF_propagate_3d(Estimation_X, OdometryFromThis2Next, odom_sigma, Noise_type )

% v = OdometryFromThis2Next(1:3);
theta = OdometryFromThis2Next(1:3);   %axis-angle   
v = OdometryFromThis2Next(4:6);

NumberOfLandmarks = size(Estimation_X.landmarks, 1);
%% Calculate jacobian

Jrw = jaco_r(-theta);


temp = repmat({Estimation_X.orientation}, NumberOfLandmarks+2,1 );
A = blkdiag(temp{:});
A(4:6,1:3)=skew(Estimation_X.position)*Estimation_X.orientation;
if NumberOfLandmarks>0
   for i=1:NumberOfLandmarks
    A(6+3*i-2:6+3*i,1:3)=skew(Estimation_X.landmarks(i,2:4))*Estimation_X.orientation;
   end
end

B1=[-Jrw  zeros(3,3); -skew(v)*Jrw -eye(3)];
B=[B1; sparse(3*NumberOfLandmarks,6)];
B=sparse(B);
adA=A*B;

switch Noise_type
    case 0
        odoCov = eye(6)*odom_sigma^2;
    case 1
        odoCov=diag([theta.^2;v.^2])*odom_sigma^2;           %WHY? Qn. The bigger the data, the larger the error
    otherwise
        sprintf('Error in EKF_propagate_3d: the noise_type has not been defined. Please check config_3d')
        return;
end

Estimation_X.cov = Estimation_X.cov+ adA*odoCov*adA';

% update position and orientation
Estimation_X.position = Estimation_X.position+Estimation_X.orientation*v;
Estimation_X.orientation = Estimation_X.orientation*so3_exp(theta);

end