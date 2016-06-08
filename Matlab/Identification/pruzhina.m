% уравнение пружинного маятника с затуханием
% d2x/dt2 + (c/m)dx/dt + (k/m)x = f(x)

m = 1;
c = 1.06;
k = 34.5;
x0 = 0.89;
r0 = 0;

[t,y] = ode45(@(t,y)[y(2); -(k/m)*y(1) - (c/m)*y(2)], ...
        [0, 10], [x0, r0]);

% аналитическое решение    
ogib = x0*exp(-c*t/2/m);   
koren = sqrt(4*k*m-c^2);
koren/2/m
koleb = cos(t*koren/2/m) + (c/2/koren)*sin(t*koren/2/m);

hold off;
set(0,'DefaultAxesFontSize',14,...
    'DefaultAxesFontName','Times New Roman');
set(0,'DefaultTextFontSize',14,...
    'DefaultTextFontName','Times New Roman'); 
title('Затухающие колебания пружинного маятника');
hold on;
plot(t,y(:,1), 'Color', 'blue');
plot(t, ogib, 'Color', 'red');
plot(t, -ogib, 'Color', 'red');
ylabel('Координата, м');
xlabel('Время, с');
legend('Колебания', 'Огибающие');
hold off;

% plot(t,y(:,1));

% подбор замены в виде внешнего прямоугольного импульса
% [t,y] = ode45(@(t,y)[y(2); -(k/m)*y(1) - (c/m)*y(2) + k*x0/2], ...
%         [0, 10], [0, 0]);
%     
% plot(t,y(:,1));