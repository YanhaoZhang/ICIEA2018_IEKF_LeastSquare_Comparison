function [cv]=GetCov_3d(P,x0,y0,z0)
%GetCov_3d: Get 3-sigma sphere of pose's position and landmarks in 3D case.
%   sphere formula: 1. wiki https://en.wikipedia.org/wiki/Sphere
%                   2. matlab function: sphere
%   Input: P:3*3 covariance matrix of estimated robot or landmark location;
%          x0 y0 z0: coordinate of estimated robot or landmark location.
%   Other parameters: s is scale factor, s=1 represents 3-sigma region.
%   To plot the ellipsoid: CV = GetCov(P,X0,Y0,Z0); 
%                          surf(CV.x,CV.y,CV.z); h = surfl(x, y, z); set(h, 'FaceAlpha', 0.5); shading interp; hold on;
%                          scatter3(X0,Y0,Z0);
%   Another way is to use matlab function: ellipsoid 

s=1; k=40;   % original: k=20, changed by zhan

if trace(P)<1e-5
    r = zeros(3,3);
else
    %P
    r = real(sqrtm(P));  %r'*r = P
end

i = 1:k+1;
j = 1:k+1;

theta = 2*pi*(i'-1)/k;
phi   = 1*pi*(j-1)/k;
x = cos(theta)*cos(phi);    % coordinate of sphere. wiki's formulas are not correct.
y = cos(theta)*sin(phi);
z = sin(theta)*ones(1,k+1);

%%%%%%%%% changing the coordinate according to cov (r). %%%%%%%%%%%%%%%
%%% Just as 2D case. it is equal to confert a circle radius=1 to a ellipse. 
%%% where r represents the rotation matrix (sim(2)). since the diag is sigma1 and sigma2, it represent 1 sigma.
%%% In 3D case, it is equivalent to change a sphere with radius=1, which represent the 1-sigma region of randn(3,1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tempx = x(:)';
tempy = y(:)';
tempz = z(:)';
%for debuging
% surf(x,y,z)
% rx = [1 0 0]';
% ry = [0 2 0]';
% rz = [0 0 3]';
% Ro = [rx, ry, rz];
% r = Ro;
% x0 = 1;
% y0 = 2;
% z0 = 3;

temp = 3*r*s*[tempx;tempy;tempz] + [x0;y0;z0];  % transfer sphere according to r
CV_x = reshape(temp(1,:),k+1,k+1);
CV_y = reshape(temp(2,:),k+1,k+1);
CV_z = reshape(temp(3,:),k+1,k+1);

cv = struct;  % output
cv.x = CV_x;
cv.y = CV_y;
cv.z = CV_z;
% surf(CV_x,CV_y,CV_z)
end
