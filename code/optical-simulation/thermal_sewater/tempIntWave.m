% Re = characteristic_length * speed  / kinematic_viscosity;

R = 0.075; %15;
L = 0.6 ;
characteristic_len = R*L/(R+L)/2; % (m)
v = 1.5e-6; % kinematic viscosity (m2/s)
Aw = 100;   % amplitude (m)
T = 3600*10; % period (s)
speed = 2*Aw/T;
Re = characteristic_len*speed/v

w = 2*pi/T;

th = 4*sqrt(v/w)
