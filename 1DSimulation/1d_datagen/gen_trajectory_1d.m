% Vehicle.position [pose0X pose0Y;pose1X pose1Y;pose2X pose2Y;...]

function [ Vehicle ] = gen_trajectory_1d( t )
if nargin < 1
    t = 0:99;
end

%dim = 1; % dimension

%     radius = 10;   
% %     Vehicle.position = radius* ([5*cos(0.3*t)]' - [5*cos(0.3*0)]'); %WHY? put pose0 to [0 0]
% %     Vehicle.position = t';
% %     q = 2*pi/50;  
% %     Vehicle.position = radius*cos(q*t-pi/2);
% %     Vehicle.position = Vehicle.position';
% 
% t1 = t(t<25);
% t2 = t(t>= 25&t< 50);
% t3 = t(t>= 50&t< 75);
% t4 = t(t>= 75&t< 100);
% 
% q = 2*radius/25;
% position1 =  q*t1 - 10;
% position2 = -q*t2 + 10+25*q;
% position3 =  q*t3 - 10-50*q;
% position4 = -q*t4 + 10+75*q;
% 
% 
% Vehicle.position = [position1 position2 position3 position4]'; 

Vehicle.position = [-10 -5 0 5 10]';
    
%     2*radius/5;
%     Vehicle.position = q*abs(t'-50);
%% debug
%     plot (t',Vehicle.position)
%     plot(Vehicle.position(:,1),0,'r.','MarkerSize',10)
end

