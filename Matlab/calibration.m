% Калибровка акселерометра, основанная на сохранении модуля g
% Задача оптимизации решается методом Нелдера-Мида

logNumbers = [561,562,563,564,565,566,567,568,570,571,572,573,574,575,576,577,578,579];

for i = 1:1:length(logNumbers)
    logName = sprintf('D:\\Vlad\\Projects\\Stable\\Logger\\BoardConsole\\BoardConsole\\log%d.txt', logNumbers(i));

    logFile = fopen(logName, 'r');
    A = fscanf(logFile, '%f %f %f\n', [3 Inf]);

    ax = A(1,:);
    ay = A(2,:);
    az = A(3,:);
    
    G(i,1) = mean(ax(5:15));
    G(i,2) = mean(ay(5:15));
    G(i,3) = mean(az(5:15));
end

G = G / 256;

teta0 = [1 0 0 0 1 0 0 0 1 0 0 0];

fminsearch(@cost, teta0)

