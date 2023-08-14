function [ result ] = so2_exp( theta )
%so2_exp: This function convert rotation angle to rotation matrix
%   compute the exponential mapping of R^2 to SO(2)
%   same as [cos -sin; sin cos]

if theta==0
    result=eye(2);
else
    result = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end
%result = expm(skew_2d(theta));
    
end

