% Анализ данных от дуса
% Температурные зависимости

logName1 = 'D:\Vlad\Projects\Stable\Logs\uhod2.txt';

logFile1 = fopen(logName1, 'r');
A1 = fscanf(logFile1, '%f %f %f %f %f %f %f\n', [7 Inf]);

arx1    = A1(1,:);
ary1    = A1(2,:);
temp1   = A1(3,:);
arx2    = A1(4,:);
ary2    = A1(5,:);
temp2   = A1(6,:);
arz     = A1(7,:);

t = 1:1:length(arx1);
plot(t, ary1, t, ary2);

t = 1:1:length(arx1);
plot(t, temp1, t, temp2);

%temperature compensation
termo_arx1 = arx1 + 0.04946 * temp1;
termo_ary1 = ary1 + 1.761 * temp1;
termo_arx2 = arx2 - 0.2236 * temp2;
termo_ary2 = ary2 + 1.309 * temp2;

% t = 1:1:length(arx1);
% plot(t, ary1, t, ary2);

% filtration
arx1 = Hlp.filter(arx1);
ary1 = Hlp.filter(ary1);
arx2 = Hlp.filter(arx2);
ary2 = Hlp.filter(ary2);

termo_arx1 = Hlp.filter(termo_arx1);
termo_ary1 = Hlp.filter(termo_ary1);
termo_arx2 = Hlp.filter(termo_arx2);
termo_ary2 = Hlp.filter(termo_ary2);

t = 1:1:length(arx1);
plot(t, termo_ary1, t, termo_ary2);

% calibration
calibrNumber = length(termo_arx1);

arx1_offset = sum(arx1(1:calibrNumber)) / calibrNumber;
arx2_offset = sum(arx2(1:calibrNumber)) / calibrNumber;
ary1_offset = sum(ary1(1:calibrNumber)) / calibrNumber;
ary2_offset = sum(ary2(1:calibrNumber)) / calibrNumber;

arx1 = arx1(calibrNumber + 1 : length(arx1)) - arx1_offset;
arx2 = arx2(calibrNumber + 1 : length(arx2)) - arx2_offset;
ary1 = ary1(calibrNumber + 1 : length(ary1)) - ary1_offset;
ary2 = ary2(calibrNumber + 1 : length(ary2)) - ary2_offset;

termo_arx1_offset = sum(termo_arx1) / calibrNumber;
termo_arx2_offset = sum(termo_arx2) / calibrNumber;
termo_ary1_offset = sum(termo_ary1) / calibrNumber;
termo_ary2_offset = sum(termo_ary2) / calibrNumber;

av_t1 = sum(temp1) / calibrNumber;
av_t2 = sum(temp2) / calibrNumber;

termo_arx1 = termo_arx1(1 : length(termo_arx1)) - termo_arx1_offset;
termo_arx2 = termo_arx2(1 : length(termo_arx2)) - termo_arx2_offset;
termo_ary1 = termo_ary1(1 : length(termo_ary1)) - termo_ary1_offset;
termo_ary2 = termo_ary2(1 : length(termo_ary2)) - termo_ary2_offset;

t = 1:1:length(termo_ary1);
plot(t, termo_ary1);

% to graduses
arx1 = arx1 / 200.0;
arx2 = arx2 / 200.0;
ary1 = ary1 / 200.0;
ary2 = ary2 / 200.0;

termo_arx1 = termo_arx1 / 200.0;
termo_arx2 = termo_arx2 / 200.0;
termo_ary1 = termo_ary1 / 200.0;
termo_ary2 = termo_ary2 / 200.0;

% angle integration
dt = 0.01;
ax1 = zeros(1, length(arx1));
ax2 = zeros(1, length(arx1));
ay1 = zeros(1, length(arx1));
ay2 = zeros(1, length(arx1));

for i = 1:1:length(arx1)-1
    ax1(i+1) = ax1(i) + termo_arx1(i) * dt;
    ax2(i+1) = ax2(i) + termo_arx2(i) * dt;
    ay1(i+1) = ay1(i) + termo_ary1(i) * dt;
    ay2(i+1) = ay2(i) + termo_ary2(i) * dt;
end


t = 1:1:length(arx1);
plot(t, 0.5*(ay1-ay2));
%plot(t, ay1);