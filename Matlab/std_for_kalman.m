logName = 'D:\Vlad\Projects\Stable\Logger\Logs\Ordinary\log1097.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f %f %f %f\n', [7 Inf]);

gx    = A(1,:);
gy    = A(2,:);
gz   = A(3,:);
q1    = A(7,:);
q2    = A(5,:);
q3   = A(6,:);
q4     = A(4,:);

std_gx = std(gx)
std_gy = std(gy)
std_gz = std(gz)
std_q1 = std(q1)
std_q2 = std(q2)
std_q3 = std(q3)
std_q4 = std(q4)