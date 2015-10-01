logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log273_koleb2.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %d %d %d %d %d %d %d %f %f %f %d\n', [14 Inf]);

angle   = A(1,:);
angVel  = A(2,:);
angAcc  = A(3,:);
ax      = A(4,:);
ay      = A(5,:);
az      = A(6,:);
pwm1    = A(7,:);
pwm2    = A(8,:);
count1  = A(9,:);
count2  = A(10,:);
% Kp      = A(12,:);
% Ki      = A(13,:);
% Kd      = A(14,:);
fclose(logFile);

count1 = 14e6./count1;
count2 = 14e6./count2;

TAU = 1.0;
prevAngVel = 0;
angAccels = zeros(1, length(angle));
desAccels = zeros(1, length(angle));

for i = 1:1:length(angle)
    if count1(i) > 10000
        count1(i) = 5000;
    end
    if count2(i) > 10000
        count2(i) = 5000;
    end
    angAccels(i) = (angVel(i) - prevAngVel) / 0.01;
    prevAngVel = angVel(i);
    
    if -angVel(i) * TAU / angle(i) / 2.0 < 1.0
        desAccels(i) = - angVel(i) / TAU - angle(i) / TAU / TAU;
    else
        t1 = - 2*angle(i)/angVel(i);
        desAccels(i) = - angVel(i) / t1;
    end
end

angle = angle * 180 / 3.14159;
angVel = angVel * 180 / 3.14159;

angAccels = angAccels * 180 / 3.14159;
desAccels = desAccels * 180 / 3.14159;

t = 1:1:223;
plot(t, angle(68:290), t, angVel(68:290));
%plot(t, angle, t, angVel, t, desAccels, t, angAccels);
%plot(t, angAccels, t, desAccels, t, pwm1 , t, pwm2);
%plot(t, pwm2, t, count2, t, angle);
% count1 = 24e6./count1;
% count2 = 24e6./count2;
% 
% 
% f1 = 0.0115 * pwm1 - 12.69;
% f2 = 0.0115 * pwm2 - 12.69;
% 
% fThreshold = 4;
% activeTime = 0;
% totalTime = 0;
% for i = 1:1:length(pwm1)
%     if (pwm1(i) ~= 0) && (pwm2(i) ~= 0)
%         totalTime = totalTime + 1;
%         if (f1(i) > fThreshold) || (f2(i) > fThreshold)
%             activeTime = activeTime + 1;
%         end
%     end
% end
% 
% activeTime / totalTime
% 
% testVel = angVel(4000:14000);
% sum = 0;
% for i = 2:1:length(testVel)
%     sum = sum + abs(testVel(i) - testVel(i-1)) / 0.01;
% end
% 
% sum = sum / (length(testVel) - 1)
% 
% t = 1:1:length(angle);
% 
% plot(t, angVel);
% % sp = fft(angle1);
% % plot(linspace(0, 50, length(sp) / 2 + 1), 10*log10(abs(sp(1:(length(sp)/2 + 1)))));
% 
% % pointNumber = 1000;
% % DT = 0.01;
% % 
% % y = zeros(1, 3);
% % u = zeros(1, 3);
% % 
% % Afull = zeros(pointNumber, 3);
% % Bfull = zeros(pointNumber, 1);
% % 
% % solution = zeros(length(angle) - pointNumber - 2, 3);
% % 
% % identificationReady = 0;
% % anglesAccumulated = 1;
% % for i = 1:1:length(angle)
% %     if anglesAccumulated < 3
% %         y(anglesAccumulated) = angle(i);
% %         u(anglesAccumulated) = F(i);
% %     else
% %         y(3) = angle(i);
% %         u(3) = F(i);
% %         
% %         row = anglesAccumulated - 2;
% %         Afull(row,1) = (y(3) - y(2)) / DT;
% %         Afull(row,2) = y(3);
% %         Afull(row,3) = -u(3);
% %         
% %         Bfull(row) = (2*y(2) - y(3) - y(1)) / DT / DT;
% %         
% %         y(1) = y(2);
% %         y(2) = y(3);
% %         u(1) = y(2);
% %         u(2) = y(3);
% %         
% %         if anglesAccumulated == (pointNumber + 2)
% %             identificationReady = 1;
% %             anglesAccumulated = 2;
% %         end
% %         
% %         if identificationReady == 1
% %             At = transpose(Afull);
% %             A = At * Afull;
% %             B = At * Bfull;
% %             
% %             solution(i - pointNumber - 1, :) = linsolve(A, B);
% %         end
% %     end
% %     anglesAccumulated = anglesAccumulated + 1;
% % end
% % 
% % plot(solution(:,3));