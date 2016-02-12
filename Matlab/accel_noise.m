% Измеряем уровень и частоты шума аксселерометра
% для того, чтобы подобрать фильтра нижних частот
% 659 - без вибрации
% 662 - с вибрацией
logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log662.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f\n', [3 Inf]);

ax = A(1,:);
ay = A(2,:);
az = A(3,:);

y = ay;

Fs = 1600;          % sampling frequency (Hz)
T = 1/Fs;           % sample time
L = length(y);      % length of signal
t = (0:L-1)*T;      % time vector


y_filtered = Hlp2.filter(y);
plot(Fs*t, y);
plot(y_filtered);

NFFT = 2^nextpow2(L); % next power of 2 from length of signal
y = y .* transpose(blackman(L));
Y = fft(y, NFFT) / L;
magnitude_in_dbs = 10*log10(abs(Y(1:NFFT/2 + 1)));
max_magnitude = max(magnitude_in_dbs);
magnitude_in_dbs = magnitude_in_dbs - max_magnitude;
f = Fs/2*linspace(0,1,NFFT/2 + 1);
plot(f , magnitude_in_dbs );


L_filtered = length(y_filtered);
NFFT_filtered = 2^nextpow2(L_filtered);
y_filtered = y_filtered .* transpose(blackman(L_filtered));
Y_filtered = fft(y_filtered, NFFT_filtered) / L_filtered;
magnitude_in_dbs_filtered  = 10*log10(abs(Y_filtered (1:NFFT_filtered/2 + 1)));
max_magnitude_filtered  = max(magnitude_in_dbs_filtered );
magnitude_in_dbs_filtered  = magnitude_in_dbs_filtered  - max_magnitude_filtered ;
f_filtered = Fs/16/2*linspace(0,1,NFFT_filtered/2 + 1);
plot(f_filtered , magnitude_in_dbs_filtered );

y_decimated = y_filtered(1:16:L_filtered);
L_decimated = length(y_decimated);
NFFT_decimated = 2^nextpow2(L_decimated);
y_decimated = y_decimated .* transpose(blackman(L_decimated));
Y_decimated = fft(y_decimated, NFFT_decimated) / L_decimated;
magnitide_in_dbs_decimated = 10*log10(abs(Y_decimated(1:NFFT_decimated/2+1)));
max_magnitude_decimated = max(magnitide_in_dbs_decimated);
magnitide_in_dbs_decimated = magnitide_in_dbs_decimated - max_magnitude_decimated;
f_decimated = Fs/16/2*linspace(0,1,NFFT_decimated/2 + 1);
plot(f_decimated , magnitide_in_dbs_decimated );