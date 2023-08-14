function [] = gen_MonteCarlo_data_1d(t)

if nargin < 1
    t = 1;     % a hundred times of monte carlo
end

for i=1:t
    disp(['Processing Monte Carlo ', int2str(i)]);
    data = gen_data_1d;
    filename = ['./data_1d/data_1d_', num2str(i),'.mat'];
    save (filename, 'data');
end



end

