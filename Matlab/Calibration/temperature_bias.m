% 885 - нагрев
% 889 ошибка баеса
% 890 ось z
% 891 ось x
% 892 ось y

logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log906.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f\n', [4 Inf]);

start = 1; 
finish = 140000;

gx = A(1,:) * 180 / pi;
gy = A(2,:) * 180 / pi;
gz = A(3,:) * 180 / pi;

temp = A(4,:);
temp = -temp;

N = length(temp);
t = 1:N;

plot(temp, gx, 'b');

gx_k = polyfit(temp, gx, 1);
gy_k = polyfit(temp, gy, 1);
gz_k = polyfit(temp, gz, 1);

hold on;

x = -16:1:25;
y = gx_k(1)*x+gx_k(2);
plot(x,y,'r');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log911.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f\n', [4 Inf]);

gx = A(1,:) * 180 / pi;
gy = A(2,:) * 180 / pi;
gz = A(3,:) * 180 / pi;

temp = A(4,:);
temp = -temp;

N = length(temp);
t = 1:N;

plot(t, gx, 'b');

gx_mean = mean(gx);
gy_mean = mean(gy);
gz_mean = mean(gz);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

logNumber = [891, 892, 890];

k = [gx_k(1), gy_k(1), gz_k(1)];
b = [gx_k(2), gy_k(2), gz_k(2)];
e = [gx_mean, gy_mean, gz_mean];

dt = 0.01;
phi = 90;
for i = 1:3
    logName = sprintf('D:\\Vlad\\Projects\\Stable\\Logger\\BoardConsole\\BoardConsole\\log%d.txt', logNumber(i));
    logFile = fopen(logName, 'r');
    A = fscanf(logFile, '%f %f %f %f\n', [4 Inf]);
    
    w = A(i,:) * 180 / pi;

    temp = A(4,:);
    N = length(temp);
    
    value = 0;
    bias = 0;
    bias_error = 0;
    
    for j = 1:N
        value = value + w(j)*dt;
        bias = bias + (k(i)*(-temp(j)) + b(i))*dt;
        bias_error = bias_error + e(i)*dt;
    end
    alpha(i) = phi / (value - bias - bias_error);
end




