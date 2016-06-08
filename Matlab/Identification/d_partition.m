% лодка без груза
% a0 = 25.5916;
% a1 = 1.06;
% лодка с грузом
a0 = -513.6215;
a1 = 2.9314;
T = 0.0028;
b = 95.9438;

k_lin = T / (T*a1+1);
b_lin = - (T^2*a0*a1+T*a1^2+a1)/(T*a1+1)/b;

k_p_min = (-a0/b);
k_d_min = (-T*a0-a1)/b;

y1 = -1:0.01:1;
x1 = k_p_min*ones(length(y1));

x2 = k_p_min:0.01:10;
y2 = k_lin*x2 + b_lin;

x3 = k_p_min:0.1:10;
y3 = k_d_min*ones(length(x3));


plot(x1, y1, 'blue', x2, y2, 'blue');
title('D-разбиение лодки с грузом');
xlabel('Kp');
ylabel('Kd');

k_p = 2;
k_d = 2;
roots([T (T*a1+1) (T*a0+a1+b*k_d) a0+b*k_p])