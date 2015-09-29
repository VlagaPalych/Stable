my_logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log105.txt';
logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log107.txt';

log1 = fopen(my_logName, 'r');
A = fscanf(log1, '%f\n', [14 Inf]);

x1 = A(1,:);


log2 = fopen(logName, 'r');
A = fscanf(log2, '%f\n', [14 Inf]);

x2 = A(1, :);


% x = filter(Hlp, x);

% mu = mean(x1)
% sigma = var(x1)
% y = A(2,:);
% z = A(3,:);
fclose(log1);

x1 = x1(1:4:length(x1));
L1 = length(x1);
L2 = length(x2);
L2 = L2 / 4;
L = min(L1, L2);
x1 = x1(1:L);
x2 = x2(1:L*4);

% Fd = 100;
% t_step = 1.0 / Fd;
% t = 0:t_step:(L-1) * t_step;
% plot(x1);

NFFT = 2^nextpow2(L);

window1 = transpose(blackman(L));
x1 = x1 .* window1;

X1 = fft(x1, NFFT) / L;
f1 = 25.0/ 2 * linspace(0,1,NFFT/2 + 1);

db1 = 10*log10(abs(X1(1:NFFT/2+1)));
maxX1 = max(db1);
db1 = db1 - maxX1;


NFFT2 =  2^nextpow2(L*4);
window2 = transpose(blackman(L*4));
x2 = x2 .* window2;

X2 = fft(x2, NFFT2) / (L*4);
f2 = 100.0/ 2 * linspace(0,1,NFFT2/2 + 1);

db2 = 10*log10(abs(X2(1:NFFT2/2+1)));
maxX2 = max(db2);
db2 = db2 - maxX2;
plot(f1, db1, f2, db2);
ylabel('amplitude, dB');
xlabel('frequency, Hz');