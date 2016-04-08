f = 0:0.01:2*pi;
R = 1;

x = R*cos(f);
y = R*sin(f);

alpha = -60 * pi / 180;

x2 = x + y*tan(alpha);
y2 = y/cos(alpha);

plot(x, y, x2, y2);
axis equal;
    