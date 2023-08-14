function [ result ] = K_r_x1x2( x1,x2 )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Function:  Calculate K_r(x1,x2). For J_r. Details see document
%   Method:   Details see document
%   Input:    x1:   3��1 vector
%             x2:   3��1 vector
%   Returns:  result:     
%   Author:   Jingwei Song.   04/06/2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Two part of integration. Details see document
integration1 = 0;
integration2 = 0;

integration1 = x2 / 2 + (norm(x1) - sin(norm(x1))) * skew(x1) * x2...
               /norm(x1)^3 + (norm(x1)^2 + 2 * cos(norm(x1)) - 2) * ...
               skew(x1)^2 * x2 / norm(x1)^4;
           
integration2 = 2 * (norm(x1) - norm(x1) * sin(norm(x1))) * eye(3,3) / norm(x1)^3 ...
               - (norm(x1)^2 + 2 * norm(x1) * sin(norm(x1)) + 2 * cos(norm(x1)) ...
               - 2) * skew(x1) / 2 * norm(x1)^4 + ...
               (2 * norm(x1) - 3 * sin(norm(x1)) + norm(x1) * cos(norm(x1))) * ... 
               skew(x1)^2  / norm(x1)^5;
           
result       = skew(integration1) + integration2 * skew(x2) * skew(x1);

end

