
x0 = 0;
z10= data.state(1,1);
z20= data.state(2,1);
u0 = data.state(3,1);
z11= data.state(4,1);
z21= data.state(5,1);

sigma_u = data.odom_sigma;
sigma_z = data.obsv_sigma;


[x0 + sigma_z^2/(sigma_u^2+sigma_z^2)*u0 + 0.5*sigma_u^2/(sigma_u^2+sigma_z^2)*(z10-z11) + 0.5*sigma_u^2/(sigma_u^2+sigma_z^2)*(z20-z21);...
 x0 + 0.25*(3*sigma_u^2+2*sigma_z^2)/(sigma_u^2+sigma_z^2)*z10 + 0.5*sigma_z^2/(sigma_u^2+sigma_z^2)*(z11+u0) + 0.25*sigma_u^2/(sigma_u^2+sigma_z^2)*(z11+z20-z21);...
 x0 + 0.25*(3*sigma_u^2+2*sigma_z^2)/(sigma_u^2+sigma_z^2)*z20 + 0.5*sigma_z^2/(sigma_u^2+sigma_z^2)*(z21+u0) + 0.25*sigma_u^2/(sigma_u^2+sigma_z^2)*(z21+z10-z11)]



0.25/(sigma_u^2+sigma_z^2)*[-2*sigma_u^2,          -2*sigma_u^2;...
                            sigma_u^2+2*sigma_z^2, -sigma_u^2;...
                            -sigma_u^2,            sigma_u^2+2*sigma_z^2]