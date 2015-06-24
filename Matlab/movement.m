s = 0.001;

Xorig = -1:s:1;
Yorig = zeros(1, 2 / s + 1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a2 = 1;
a0 = 2;

C1 = (pi / 60) * (1 + sqrt(a0/a2));
C2 = (pi / 60) * (1 - sqrt(a0/a2));
b = sqrt(a0 / a2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot(Xorig, Yorig);
axis([-1.5 1.5 -1.5 1.5])
pause(0.01);

for t = 0:0.01:10
    a = C1 * exp(- b * t) + C2 * exp(b * t);

    M = [cos(a) -sin(a);
        sin(a) cos(a)];

    rotated = M * [Xorig; Yorig];
    x = rotated(1, ':');
    y = rotated(2, ':');

    plot(x, y);
    axis([-1.5 1.5 -1.5 1.5])
    pause(0.01);
end

