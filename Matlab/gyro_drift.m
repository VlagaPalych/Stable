logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log638.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f\n', [4 Inf]);

arx    = A(1,:);
ary    = A(2,:);
phi_x  = A(3,:);
phi_y  = A(4,:);

phi_x = phi_x(3000:length(phi_x));
phi_y = phi_y(3000:length(phi_y));

mins = length(phi_x)/6000;
mins = floor(mins);

phi_x = phi_x(1:mins*6000);
phi_y = phi_y(1:mins*6000);

t_step = mins / length(phi_x);

t = 0:t_step:mins-t_step;

k_x = polyfit(t, phi_x, 1);
k_y = polyfit(t, phi_y, 1);

k_x(1), k_y(1)
