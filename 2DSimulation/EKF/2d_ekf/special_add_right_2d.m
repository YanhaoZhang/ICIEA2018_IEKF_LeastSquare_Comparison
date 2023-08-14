function [ X ] = special_add_right_2d( X,S )
%special_add_right_2d: RI-EKF in 2D case
%    according to Teng's code and peper 'An EKF-SLAM algorithm with consistency properties' Eq. 22

    s_theta=S(1);
    s_p=S(2:3);

    sizeS=size(S,1);
    NumberOfLandmarks=(sizeS-3)/2;

    Exps=so2_exp(s_theta);
    
    B = [sin(s_theta)/s_theta,      -(1-cos(s_theta))/s_theta;...
        (1-cos(s_theta))/s_theta,   sin(s_theta)/s_theta];
    X.position= Exps*X.position + B*s_p;

%      X.position= Exps*X.position+jaco_r_2d(-s_theta)*s_p;



    if NumberOfLandmarks>=1
        s_landmarksMatrix=reshape(S(4:end),2,NumberOfLandmarks);
        X.landmarks(:,2:3)=(Exps*X.landmarks(:,2:3)' + B*s_landmarksMatrix)';
%          X.landmarks(:,2:3)=(Exps*X.landmarks(:,2:3)'+jaco_r_2d(-s_theta)*s_landmarksMatrix)';
    end

    X.orientation=Exps*X.orientation;


end