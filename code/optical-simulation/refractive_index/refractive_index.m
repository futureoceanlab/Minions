n_a = 1;
n_w = 1.34;
n_s = 1.75;%1.45; %75;
d_a = 8;
d_s = 5;
fov = deg2rad(8);
a = 45;
w = sqrt((2592*0.02)^2 + (1944*0.02/cosd(a))^2);
% fov = atan((2592*0.0022/2)/25);
% w = 37.7846*2; %51.84;

h_a = d_a*tan(fov);

theta_s = asin(n_a/n_s*sin(fov));
h_s = d_s*tan(theta_s);

theta_w = asin(n_s/n_w*sin(theta_s));
h_w = w/2-(h_a+h_s);
d_w = h_w/tan(theta_w);
d_w*cosd(a)
rad2deg(atan(w/2/(d_w+d_s+d_a)))*2