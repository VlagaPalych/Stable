logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log10.txt';

log = fopen(logName, 'r');
%A = fscanf(log, '%.2f %.2f %.2f %hd %hd %hd %d %d %.2f %.2f\n', [10 Inf]);
A = fscanf(log, '%f %f %f %f %f %f %f %f\n', [8 Inf]);
angle   = A(1,:);
angVel  = A(2,:);
F       = A(3,:);
ax      = A(4,:);
ay      = A(5,:);
az      = A(6,:);
pwm1    = A(7,:);
pwm2    = A(8,:);
%k1      = A(9,:);
%k2      = A(10,:);
fclose(log);

plot(angle);
%plot(F, pwm1, '*', F, pwm2, '*');