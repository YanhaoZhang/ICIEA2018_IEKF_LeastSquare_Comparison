function [ X ] = SpecialAdd_3d( X,S )

s_theta=S(1:3);
s_p=S(4:6);

sizeS=size(S,1);
NumberOfLandmarks=(sizeS-6)/3;


X.position=X.position+s_p;



if NumberOfLandmarks>=1
    s_landmarksMatrix  = (reshape(S(7:end),3,NumberOfLandmarks))';
    X.landmarks(:,2:4) = X.landmarks(:,2:4) + s_landmarksMatrix;
end

X.orientation=Exp_3d(s_theta)*X.orientation;


end



