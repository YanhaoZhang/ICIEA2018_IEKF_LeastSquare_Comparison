function [cv]=GetCov_1d(P,x)
%GetCov_1d: draw 3-sigma of cov in 1D case.
%   since 1D, 3-sigma is just a line.
%   Input: P: sigma 1*1; x: mu, namely estimated robot or landmark location.


s=1; k=80;   % original: k=20, changed by zhan

if trace(P)<1e-5
    r=zeros(1,1);
else
    %P
    r=real(sqrtm(P)); %r'*r = P
end

for j=1:k+1
    q=2*pi*(j-1)/k;                      % q is the step theta, roll it around
    cv(:,j)=s*3*r*[cos(q)]+[x];          %here: it is equal to confert a circle radius=1 to a ellipse. r represents the rotation matrix (sim(2)). 
                                         %      since the diag is sigma1 and sigma2, it represent 1 sigma. In 1D case, it just
                                         %      use cos(q), as a line in x-axis. in 2D: [cos(q); sin(q)] represents a ball
end

% Original introduction for 2D case:

% P is 2x2 covariance matrix, s is scale factor, x,y are estimated robot or landmark location

% A typical call is 

% CV=GetCov(sys.P(1:2,1:2,n),sys.X(1,n),sys.X(2,n));

% sys.P(1:2,1:2,n) is robot covariance, sys.X(1,n) sys.X(2,n) is robot location

% To plot it

% set(fig.car.CV,'xdata',CV(1,:),'ydata',CV(2,:));

% fig.car.CV is a graph handle
