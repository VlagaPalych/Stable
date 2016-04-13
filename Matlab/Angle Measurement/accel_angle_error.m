logName = 'D:\Vlad\Projects\Stable\Logger\Logs\Ordinary\log1244.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f %f %f %f\n', [7 Inf]);

ax = A(1,:);
ay = A(2,:);
az = A(3,:);

roll = A(4,:);
pitch = A(5,:);

roll_error = A(6,:);
pitch_error = A(7,:);

re = acos(sqrt((1-ax.^2)./(ay.^2+az.^2)));

t = 1:length(ax);

plot(t, roll, t, re);