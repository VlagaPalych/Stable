logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log608.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f\n', [2 Inf]);

arx    = A(1,:);
ary    = A(2,:);

plot(arx)