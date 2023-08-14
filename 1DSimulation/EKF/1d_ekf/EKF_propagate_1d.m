function [Estimation_X] = EKF_propagate_1d(Estimation_X, OdometryFromThis2Next, odom_sigma, Noise_type)
%EKF_propagate_1d: this is the for propagation process in 1D normal EKF-SLAM (left invariant slam)
%{
    Here, OdometryFromThis2Next is just 1d value, represent l_I_KI, which is equivalent to l_0_KI in 1D case.
%}

v = OdometryFromThis2Next(1);
Estimation_X.position = Estimation_X.position + v                              % p_0_31 = p_0_21 + R_02*p_2_32. x_n-1|n-1
                                                                               % there is no rotation in 1d case.
NumberOfLandmarks = size(Estimation_X.landmarks, 1);

%% Calculate jacobian
% F
F11 = 1;                          F12 = zeros(1,NumberOfLandmarks);
F21 = zeros(NumberOfLandmarks,1); F22 =eye(NumberOfLandmarks);
F = [F11,F12;F21,F22];              % (1+k)*(1+k)

% G
G11 = 1;
G21 = zeros(NumberOfLandmarks,1);
G = [G11;G21];                      % (1+k)*1

%% Noise type
switch Noise_type
    case 0
        odoCov = eye(1)*odom_sigma^2;
    case 1
        odoCov=diag([v.^2])*odom_sigma^2;           %WHY? Qn. The bigger the data, the larger the error
    otherwise
        sprintf('Error in EKF_propagate_2d: the noise_type has not been defined. Please check config_2d')
        return;
end
W = G*odoCov*G';

%% final propragate the covariance
Estimation_X.cov = F*Estimation_X.cov*F'+W;

end




