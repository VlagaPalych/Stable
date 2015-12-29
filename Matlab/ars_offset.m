logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log538.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f %f %f %f %f %f %f %f %f\n', [12 Inf]);

arx1    = A(1,:);
ary1    = A(2,:);
temp1   = A(3,:);

arx2    = A(4,:);
ary2    = A(5,:);
temp2   = A(6,:);

arx5    = A(7,:);
ary5    = A(8,:);
temp5   = A(9,:);

ax      = A(10,:);
ay      = A(11,:);
az      = A(12,:);

len = length(arx1);

arx1 = arx1 - 0.0037;
arx2 = arx2 - 0.1196;
ary1 = ary1 - 1.7464;
ary2 = ary2 - 3.7092;

ax_av = sum(ax) / len / 275
ay_av = sum(ay) / len / 275
az_av = sum(az) / len / 275

roll = atan(ay_av / az_av) * 180.0 / pi
pitch = atan(- ax_av / sqrt(ay_av^2 + az_av^2)) * 180.0 / pi

arx1_offset = sum(arx1) / len
ary1_offset = sum(ary1) / len

arx2_offset = sum(arx2) / len
ary2_offset = sum(ary2) / len

arx5_offset = sum(arx5) / len
ary5_offset = sum(ary5) / len

t = 1:1:len;

arx1 = arx1 - 0.0037;
arx2 = arx2 - 0.1196;
ary1 = ary1 - 1.7464;
ary2 = ary2 - 3.7092;

%plot(t, ary1, t, ary2);
plot((ary1-ary2)/2);




