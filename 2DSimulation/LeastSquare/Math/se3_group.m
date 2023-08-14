%% This is a second version of Teng's code se3_group 
%{
This function allows we calculate the coordinate transfer in lee-group
se(3). The dimension of the result is according to that of the input: 
    - 2*3 represents 2-D graph
    - 3*4 represents 3-D graph
%}

function [ y ] = se3_group( X1, X2 )
dimension = size(X1, 1);
switch dimension
    case 2
        sprintf('se3_group: This a 2-D graph')
        T1 = [X1; 0 0 1]; %complete the bottom line for calculating the multiple
        T2 = [X2; 0 0 1];
        T  = T1*T2;
        y  = T(1:2,1:3);
    case 3
        sprintf('se3_group: This a 3-D graph')
        T1 = [X1; 0 0 0 1]; %complete the bottom line for calculating the multiple
        T2 = [X2; 0 0 0 1];
        T  = T1*T2;
        y  = T(1:3,1:4);
    otherwise
        sprintf('Waring: Dimension error in se3_group.m!')
        y = 1;
end

end