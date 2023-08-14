%%%%%%%%%%
%%% this function convert Rotation matrix to eular axis. For adding turbulance to rotation matrix
%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%
%%% This function convert a rotation matrix to Euler Axis: 
%%% in 3D case, the output is a vector(eulerAxis), while in 2D case, the output is rotation angle


function eulerAxis = so3_log(R)

dim = size(R,2); % calculate dimension
errorFlag = -1;

switch dim
    case 2
        if(norm(R-eye(2),'fro') < 2*eps)    % test whether R almost = I, so eular vector = [0 0 0]
            eulerAxis = 0;
            return
        end
        phi = atan2( R(2,1), R(1,1)  );       %WHY?
        if (abs(phi)) < 1e-10
            eulerAxis = 0;
            return
        end
        if norm(R-eye(2))>0.00001
            eulerAxis = atan2( R(2,1), R(1,1)  ); % 2-D eulerAxins is just theta
        else
            eulerAxis = 0;
        end
        errorFlag = 1;
    case 3
        if (norm(R-eye(3),'fro') < 2*eps)  % R almost = I, so eular vector = [0 0 0]
            eulerAxis = zeros(3,1);
            return
        end
        phi = acos(1/2*(trace(R)-1));       %WHY?    It just work in 3D case
        if (abs(phi) < 1e-10)
            eulerAxis = zeros(3,1);
            return
        end
        if norm(R-eye(3))>0.00001
            eulerAxis = so3_hatinv(phi/(2*sin(phi))*(R-R'));  % wiki: Axis–angle representation
        else
            eulerAxis=[0;0;0];
        end
        errorFlag = 1;
end
if (errorFlag == -1)
    sprintf('Warning: an error occure in so3_log.m')
    eulerAxis = errorFlag;
    return
end
end