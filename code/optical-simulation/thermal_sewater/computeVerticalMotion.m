

alpha_f = 1.04e-4;
alpha_w = 2.5e-4;
rho = 1025;
g = 9.81;
gamma_w = 4.4212e-10; % 1/Pa
delta = 1;
Omega = 2*pi*((rho*g^2*gamma_w/(1-alpha_f/alpha_w))^(1/2));
s = 0.9;
N = 0.0001; %Omega*0.9; %0.001; %:0.001:0.01;
r = (1-s)./(1-s+((N./Omega).^2));
% w0 = sqrt(((1-(alpha_f/alpha_w)).*N.^2+Omega^2*(1-s))./(1+delta)) %((1-alpha_f/alpha_w)*(N^2)/(1+delta)/(1-r))^(1/2)
w0 = sqrt((1-alpha_f/alpha_w)*N^2/(1-r)/(1+delta))
a= 100;
wi = 2*pi*0.04/3600; %w0/50; % 0.001;
v = 1.5e-6;
R = 0.075;
L = 0.6;
bw = (4.8*(a^2*wi*(w0^2)*v^2*r^2)/(R^4*L^2))^(1/5);
Q = (0.21*R^4*L^2*(w0^3)/(wi*a^2*v^2*r^2))^(1/5);
Ew = (3/2)*a^2*wi/(w0^2);
br = (Ew*bw*((Q*r)^2))^(1/2)
V = pi*(R^2)*L;
A = 2*(pi*R^2)+(2*pi*R*L);
l = V*(1+delta)/A;
Re = l*br*w0/v;

beta = 6.51*((v/w0)^(1/2));
N0 = sqrt((1-s)*(alpha_f/alpha_w))*Omega