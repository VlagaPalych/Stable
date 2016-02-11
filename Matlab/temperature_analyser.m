% Температурные зависимости дусов

logName1 = 'D:\Vlad\Projects\Stable\Logs\temperature5.txt';

logFile1 = fopen(logName1, 'r');
A1 = fscanf(logFile1, '%f %f %f %f %f %f %f\n', [7 Inf]);

log1_arx1    = A1(1,:);
log1_ary1    = A1(2,:);
log1_temp1   = A1(3,:);
log1_arx2    = A1(4,:);
log1_ary2    = A1(5,:);
log1_temp2   = A1(6,:);

plot(log1_ary1);
t = log1_temp1(1:25000);
a = log1_arx1(1:25000);

logName2 = 'D:\Vlad\Projects\Stable\Logs\temperature2.txt';

logFile2 = fopen(logName2, 'r');
A2 = fscanf(logFile2, '%f %f %f %f %f %f\n', [6 Inf]);

log2_arx1    = A2(1,:);
log2_ary1    = A2(2,:);
log2_temp1   = A2(3,:);
log2_arx2    = A2(4,:);
log2_ary2    = A2(5,:);
log2_temp2   = A2(6,:);

logName3 = 'D:\Vlad\Projects\Stable\Logs\temperature3.txt';

logFile3 = fopen(logName3, 'r');
A3 = fscanf(logFile3, '%f %f %f %f %f %f\n', [6 Inf]);

log3_arx1    = A3(1,:);
log3_ary1    = A3(2,:);
log3_temp1   = A3(3,:);
log3_arx2    = A3(4,:);
log3_ary2    = A3(5,:);
log3_temp2   = A3(6,:);

len = length(log1_ary1);
t = 1:1:len;

% plot(log1_temp1);


% log1_ary1 = Hlp.filter(log1_ary1(20:len-20));
% log2_ary1 = Hlp.filter(log2_ary1(20:len-20));
% log3_ary1 = Hlp.filter(log3_ary1(20:len-20));
% log1_temp1 = Hlp.filter(log1_temp1(20:len-20));
% log2_temp1 = Hlp.filter(log2_temp1(20:len-20));
% log3_temp1 = Hlp.filter(log3_temp1(20:len-20));


plot(log1_temp1, log1_ary1, log1_temp1, log2_ary1, log3_temp1, log3_ary1);