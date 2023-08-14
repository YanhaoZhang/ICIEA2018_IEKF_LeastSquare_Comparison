function [ X ] = SpecialAdd_1d( X,S )
%SpecialAdd_1d: 
%   S[theta;x;y;[]';[]';[]';...;[]'] the first three are the coordinates of pose while the rest are that of landmarks
%   In 1d case, there is no rotation.

s_p=S(1:1);                    % In 1d case, there is no rotation.

sizeS=size(S,1);
NumberOfLandmarks=(sizeS-1)/1; % dimension =1


X.position=X.position+s_p;



if NumberOfLandmarks>=1
    s_landmarksMatrix=(reshape(S(2:end),1,NumberOfLandmarks))';   %NumberOfLandmarks*2
    X.landmarks(:,2:2)=X.landmarks(:,2:2)+s_landmarksMatrix;      %X.landmarks: 1-st column is landmark index, the following is the coordinate.
                                                                  % in 1D case, X.landmarks: N*2
end

%X.orientation=Exp_2d(s_theta)*X.orientation;     %WHY?
%X.orientation=X.orientation*Exp_2d(s_theta);      %2D no difference between left rotation or right rotation

end

%To see what is reshap
% a = 1:10
% reshape(a, 2,5)

