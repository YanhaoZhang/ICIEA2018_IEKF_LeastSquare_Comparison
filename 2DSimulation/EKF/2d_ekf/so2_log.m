function eulerAxis = so2_log(R)
%so2_log: log mapping, calculate theta from R for 2D case


if(norm(R-eye(2),'fro') < 2*eps)    % test whether R almost = I, so eular vector = [0 0 0]
    eulerAxis = 0;
    return
end
phi = atan2( R(2,1), R(1,1)  );       %WHY?
if (abs(phi)) < 1e-8
    eulerAxis = 0;
    return
end
if norm(R-eye(2))>0.00001
    eulerAxis = atan2( R(2,1), R(1,1)  ); % 2-D eulerAxins is just theta
else
    eulerAxis = 0;
end
end