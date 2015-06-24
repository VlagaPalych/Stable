a = -4.596e-7;
b = 0.002354;
c = -2.154;

x = 1200:0.01:1800;

y = a*x.^2 + b*x + c;

Kmin = 2e-8;
Ymin = Kmin*x.^2;

Kmax = 1.9e-7;
Ymax = Kmax*x.^2;

% plot(x, y, x, Ymin, x, Ymax);

T = 2.205e9;
c = c + T;

D = b^2 - 4*a*c;

x1 = (-b-sqrt(D))/(2*a)
x2 = (-b+sqrt(D))/(2*a)