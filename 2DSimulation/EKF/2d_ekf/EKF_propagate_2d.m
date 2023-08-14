function [Estimation_X] = EKF_propagate_2d(Estimation_X, OdometryFromThis2Next, odom_sigma, Noise_type)

%% retrieve v and w
theta = OdometryFromThis2Next(1);
v = OdometryFromThis2Next(2:3);

NumberOfLandmarks = size(Estimation_X.landmarks, 1);

%% Calculate F
% F = eye(3+2*NumberOfLandmarks);

%% Calculate adA
J = [0 -1;1 0];
G = zeros(3+2*NumberOfLandmarks, 3);
G(1,1) = 1;
G(2:3,1:3) = [-J*Estimation_X.position, Estimation_X.orientation];

if NumberOfLandmarks>0
    for i=1:NumberOfLandmarks
        G(3+2*i-1:3+2*i,1) = -J*Estimation_X.landmarks(i,2:3)';
    end
end

% Jrw = jaco_r_2d(-theta);
% 
% temp = repmat({Estimation_X.orientation}, NumberOfLandmarks+2,1 );
% A = blkdiag(temp{:});
% A(3:4,1:2)=skew_2d(Estimation_X.position)*Estimation_X.orientation;
% if NumberOfLandmarks>0
%    for i=1:NumberOfLandmarks
%     A(4+2*i-1:4+2*i,1:2)=skew(Estimation_X.landmarks(i,2:3)')*Estimation_X.orientation;
%    end
% end
% 
% B1=[-Jrw  zeros(2,2); -skew(v)*Jrw -eye(2)];
% B=[B1; sparse(2*NumberOfLandmarks,4)];
% B=sparse(B);
% 
% adA=A*B;


%% calculate odoCov
switch Noise_type
    case 0
        odoCov = eye(3)*odom_sigma^2;
    case 1
        odoCov=diag([theta.^2;v.^2])*odom_sigma^2;           %WHY? Qn. The bigger the data, the larger the error
    otherwise
        sprintf('Error in EKF_propagate_2d: the noise_type has not been defined. Please check config_2d')
        return;
end
%% final propragate
% propragate the covariance
Estimation_X.cov = Estimation_X.cov + G*odoCov*G';
% Estimation_X.cov = Estimation_X.cov+ adA*odoCov*adA';

% propragate position and orientation
Estimation_X.position = Estimation_X.position + Estimation_X.orientation*v;
Estimation_X.orientation = Estimation_X.orientation * so2_exp(theta);           

% R_0Last.                       theta_n-1|n-1
% p_0_31 = p_0_21 + R_02*p_2_32. x_n-1|n-1
% R_0i = R_0Last*R_LastI         theta_n|n-1
%                                P_n|n-1 = P_n-1|n-1
end




