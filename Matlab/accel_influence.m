logName1 = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log499.txt';

logFile1 = fopen(logName1, 'r');
A1 = fscanf(logFile1, '%f %f %f %f %f %f %f %f %f %f\n', [10 Inf]);

arx1    = A1(1,:);
ary1    = A1(2,:);
temp1   = A1(3,:);
arx2    = A1(4,:);
ary2    = A1(5,:);
temp2   = A1(6,:);
arz     = A1(7,:);
acx     = A1(8,:);
acy     = A1(9,:);
acz     = A1(10,:);

t = 1:1:length(arx1);

plot(t, arx1 ./ acx);
%plot(t, acx, t, acy, t, acz);