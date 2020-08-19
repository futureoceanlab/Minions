% REFRACTIVE INDEX
% Compute the effective focal length and working distance 
% based on the refractive indices of the media involved 
% (i.e., seawater, sapphire and air)

% Refractive index
n_a = 1; % air
n_w = 1.34; % water
n_s = 1.75; % sapphire

% Distance travelled by light (mm)
d_a = 8;
d_s = 5;

fl = 25; % focal length = 25mm
% START LED configuration
% fov = deg2rad(8)
% in the case of LED, the light is 45deg slanted
% a = 45;
% w = sqrt((2592*0.02)^2 + (1944*0.02/cosd(a))^2);
% END LED

% Lens imaging optical configuration
% width of the target water volume 
w = 51.84; % 37.7846*2
fov = atan((2592*0.0022/2)/fl); % field of view in air

h_a = d_a*tan(fov); % height covered in air

theta_s = asin(n_a/n_s*sin(fov));
h_s = d_s*tan(theta_s); % height covered in sapphire
 
theta_w = asin(n_s/n_w*sin(theta_s));
h_w = w/2-(h_a+h_s); % height covered in water
d_w = h_w/tan(theta_w);  %distance in water
d_w % Effective light working distance
rad2deg(atan(w/2/(d_w+d_s+d_a)))*2 % effective field of view