function [deltaomegas,ys] = simulator()

% constants (needs to be known beforehand)
kt = 0.02;
kq = 0.0006;
l = 0.3;

% estimation parameters
I = 0.08;
Iz = 0.2;
m = 2.6;
tau = 0.12;

dt = 1/32;

kz = kq/Iz;
a = l/I;
b = 1/m;

n = 160;
timestamp = linspace(dt,dt*n,n)';

deltaomegas = zeros(4,n);
for i = 1:160
    j = i;
    deltaomegas(1,i) = sin(2*pi*j/40)*sin(0.5*pi*j/40)*2;
    deltaomegas(2,i) = -sin(1*pi*j/40)*cos(0.5*pi*j/40)*2*cos(0.6*pi/60);
    deltaomegas(3,i) = -sin(1*pi*j/40)*sin(0.5*pi*j/40)*2;
    deltaomegas(4,i) = sin(2*pi*j/40)*cos(0.5*pi*j/40)*2;
end

xs = zeros(9,n);
xs(5,1) = a;
xs(6,1) = b;
xs(7,1) = kz;
xs(8,1) = kt;
xs(9,1) = tau;

for i = 2:n
    x = xs(:,i-1);
    deltaomega = deltaomegas(:,i);
    pdotcmd = (1/sqrt(2)*a*kt).*[1 -1 -1 1]*deltaomega;
    pdot = x(1);
    qdotcmd = (1/sqrt(2)*a*kt).*[1 -1 1 -1]*deltaomega;
    qdot = x(2);
    rdotcmd = kz.*[-1 -1 1 1]*deltaomega;
    rdot = x(3);
    wdotcmd = (b*kt).*[-1 -1 -1 -1]*deltaomega;
    wdot = x(4);
    
    dxdt = [(pdotcmd-pdot)/tau; (qdotcmd-qdot)/tau; (rdotcmd-rdot)/tau; ...
        (wdotcmd-wdot)/tau; 0;0;0;0;0];
    x = x + dt.*dxdt;
    xs(:,i) = x;
end

ys = xs(1:4,:);
csvwrite('../output/simulator.csv',[timestamp deltaomegas' ys']);
end