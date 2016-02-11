% Cтроит график allan-variance, показывающий время за которое происходит
% перемена bias'ов в гироскопе

logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log615.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f\n', [2 Inf]);

arx    = A(1,:);
ary    = A(2,:);

arx = arx(1000:361000);
ary = ary(1000:361000);

dt = 0.01;

t = 1:1:400;
s_arx = zeros(1, length(t));
s_ary = zeros(1, length(t));

for i = 1:1:length(t)
    K = 3600 / t(i);
    numSamples = t(i) / dt;
    for k = 1:1:K-1
        arx_t_k = mean(arx(k*numSamples+1:(k+1)*numSamples));
        arx_t_k_1 = mean(arx((k-1)*numSamples+1:k*numSamples));
        
        s_arx(i) = s_arx(i) + (arx_t_k - arx_t_k_1)^2;
        
        ary_t_k = mean(ary(k*numSamples+1:(k+1)*numSamples));
        ary_t_k_1 = mean(ary((k-1)*numSamples+1:k*numSamples));
        
        s_ary(i) = s_arx(i) + (ary_t_k - ary_t_k_1)^2;
    end
    s_arx(i) = s_arx(i) / 2;
    s_ary(i) = s_ary(i) / 2;
end

plot(t, s_arx, t, s_ary);