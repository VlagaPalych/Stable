logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log837.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f %f %f %f %f %f\n', [19 Inf]);

w1 = A(1:3,:);
w2 = A(4:6,:);
accel = A(7:9,:);
magField = A(10:12,:);
angleRate = A(13:15,:);
stm_q = A(16:19,:);

N = length(w1);

sigma_angleRate = var(angleRate')

sigma_q = var(stm_q')
