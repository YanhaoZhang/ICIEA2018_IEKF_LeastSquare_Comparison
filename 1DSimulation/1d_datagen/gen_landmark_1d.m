function [landmarks] = gen_landmark_1d

config_1d;
num_land    = N_LANDMARKS;
radius = 10;
range = MAX_RANGE;

rangeLand = 2*(radius+range-1);
q = rangeLand/(num_land-1);
t = 0:num_land-1;
landmarks = (q*t)'-(radius+range-1);

%% debug
%  plot(t',landmarks,'go');

end