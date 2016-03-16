logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log865.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f %f %f %f %f %f\n', [9 Inf]);

angleRate = A(1:3,:);
euler = A(4:6,:);
stm_eulerRate = A(7:9,:);

N = length(angleRate);

T = [1  sin(euler(1))*tan(euler(2))     cos(euler(1))*tan(euler(2))
     0  cos(euler(1))                   -sin(euler(1))
     0  sin(euler(1))/cos(euler(2))     cos(euler(1))/cos(euler(2))];
     
eulerRate = T * angleRate;
eulerRate = eulerRate * 180 / pi;

t = 1:N;
plot(t, angleRate(1,:)*180/pi, t, eulerRate(1,:), t, stm_eulerRate(1,:));