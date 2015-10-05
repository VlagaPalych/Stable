logName = '../Logs/imp1.txt';

logFile = fopen(logName, 'r');
M = fscanf(logFile, '%f %f %f %d %d %d %d %d %d %d %f %f %f %d %f \n', [15 Inf]);
angle = M(1,:);
vel = M(2,:);
time = 0:0.01:(length(angle)-1) * 0.01;

angle = - angle;
vel = - vel;

dz = 0.2;
w0 = 7.4;
b = 0.01;

A = [0 1;
    -w0^2 -2*dz*w0];

sys = ss(A, [0; b * w0^2], [1 0], 0);
bode(sys);

controller = pid(20, 0, 0.1, 0);

bode(tf(controller) * tf(sys));
% subplot(2, 1, 1); 
% impulse(sys);
% 
% subplot(2, 1, 2); 
% plot(time, angle);
% ylabel('Angle, rads');
% xlabel('Time, s');