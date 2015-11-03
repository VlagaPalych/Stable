logName1 = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log503.txt';

logFile1 = fopen(logName1, 'r');
A1 = fscanf(logFile1, '%f %f\n', [2 Inf]);

roll    = A1(1,:);
pitch   = A1(2,:);

mu_roll = sum(roll) / length(roll)
mu_roll2 = sum(roll.^2) / length(roll);
sigma_roll = mu_roll2 - mu_roll^2

mu_pitch = sum(pitch) / length(pitch)
mu_pitch2 = sum(pitch.^2) / length(pitch);
sigma_pitch = mu_pitch2 - mu_pitch^2