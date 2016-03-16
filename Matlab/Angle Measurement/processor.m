logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log836.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f %f %f %f %f %f\n', [19 Inf]);

w1 = A(1:3,:);
w2 = A(4:6,:);
accel = A(7:9,:);
magField = A(10:12,:);
angleRate = A(13:15,:);
stm_q = A(16:19,:);

N = length(w1);
d = 0.01;

x_apriori = [0 0 0 1 0 0 0]';
P_apriori = zeros(7, 7);

F = eye(7);
F(5,1) = 0.5*d;
F(6,2) = 0.5*d;
F(7,3) = 0.5*d;

Q = diag([0.1 0.1 0.1 0 0 0 0]);

H = eye(7);

R = diag([0.01 0.01 0.01 1e-4 1e-4 1e-4 1e-4]);

quest_roll = zeros(1,N);
quest_pitch = zeros(1,N);
quest_yaw = zeros(1,N);

kalman_roll = zeros(1,N);
kalman_pitch = zeros(1,N);
kalman_yaw = zeros(1,N);

gyro_roll = zeros(1,N);
gyro_pitch = zeros(1,N);
gyro_yaw = zeros(1,N);

for i = 2:N
    q = quest(w1(:,i), w2(:,i), accel(:,i), magField(:,i));
    
    [quest_yaw(i), quest_pitch(i), quest_roll(i)] = quat2angle(q);
    
    zk = [angleRate(:,i)' q]';
    [x_aposteriori, P_aposteriori, x_apriori, P_apriori] = my_kalman(F, Q, H, R, x_apriori, P_apriori, zk, @state_equation, d);
    
    q = x_aposteriori(4:7)';
    orient(:,i) = q;
    
    [kalman_yaw(i), kalman_pitch(i), kalman_roll(i)] = quat2angle(q);
    
    gyro_roll(i) = gyro_roll(i-1) + angleRate(1,i)*d;
    gyro_pitch(i) = gyro_pitch(i-1) + angleRate(2,i)*d;
    gyro_yaw(i) = gyro_yaw(i-1) + angleRate(3,i)*d;
end

[stm_orient_yaw, stm_orient_pitch, stm_orient_roll] = quat2angle(stm_q');

t = 1:N;
plot(t, stm_q(4,:), t, orient(4,:));

% plot(t, quest_roll, t, gyro_roll, t, kalman_roll, t, stm_roll, t, stm_orient_roll);
% plot(t, quest_pitch, t, gyro_pitch, t, kalman_pitch, t, stm_pitch);
% plot(t, quest_yaw, t, gyro_yaw, t, kalman_yaw, t, stm_yaw);

