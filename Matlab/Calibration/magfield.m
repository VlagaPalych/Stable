R = 1;
f = 0:0.01:2*pi;
x = R*cos(f);
y = R*sin(f);


% постоянное однородное магнитное поле
mx = 0;
my = 10;

mx_res = mx.*x/R - my.*y/R;
my_res = mx.*y/R + my.*x/R;

plot(mx_res, my_res, 'o')
axis equal;


% ближе к нулю линии гуще
k = 5;
b = 10;

mx = 0;
my = -k*abs(x) + b;

mx_res = mx.*x/R - my.*y/R;
my_res = mx.*y/R + my.*x/R;

plot(mx_res, my_res, 'o')
axis equal;


