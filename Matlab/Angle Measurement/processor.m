logName = 'D:\Vlad\Projects\Stable\Logger\Logs\Ordinary\log1088.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f %f %f %f %f %f %f %f %f %f\n', [13 Inf]);

w1 = [-0.00142424612 -0.00955718104 0.99995333]';
w2 = [0.687127531 -0.475319982 -0.549478471]';
accel = A(1:3,:);
angleRate = A(4:6,:);
magField = A(7:9,:);
stm_orient = A(10:13,:);

accel = accel / 16384;
angleRate = angleRate / 16.4;
magField = magField * 0.15;

angleRate(1,:) = - angleRate(1,:);
angleRate(2,:) = - angleRate(2,:);
angleRate(3,:) = - angleRate(3,:);
angleRate = angleRate * pi / 180.0;

magField(1,:) = magField(1,:) * 1.17578125;
magField(2,:) = magField(2,:) * 1.1796875;
magField(3,:) = magField(3,:) * 1.1328125;

magField(3,:) = -magField(3,:);
tmp = magField(1,:);
magField(1,:) = magField(2,:);
magField(2,:) = tmp;

N = length(accel);
d = 0.005;

x_apriori = [0 0 0 1 0 0 0]';
P_apriori = zeros(7, 7);

F = eye(7);
F(5,1) = 0.5*d;
F(6,2) = 0.5*d;
F(7,3) = 0.5*d;

Q = diag([0.1 0.1 0.1 0 0 0 0]);

H = eye(7);

R = diag([0.9125e-5 0.7459e-5 0.3602e-5 0.0017e-5 0.4351e-5 0.3828e-5 0.0006e-5]);

quest_roll = zeros(1,N);
quest_pitch = zeros(1,N);
quest_yaw = zeros(1,N);

kalman_roll = zeros(1,N);
kalman_pitch = zeros(1,N);
kalman_yaw = zeros(1,N);

gyro_roll = zeros(1,N);
gyro_pitch = zeros(1,N);
gyro_yaw = zeros(1,N);

for i = 1:N
    q = quest(w1, w2, accel(:,i) / norm(accel(:,i)), magField(:,i) / norm(magField(:,i)));
%          [quest_yaw(i), quest_pitch(i), quest_roll(i)] = quat2angle(q);
    
    zk = [angleRate(:,i)' q]';
    [x_aposteriori, P_aposteriori, x_apriori, P_apriori] = my_kalman(F, Q, H, R, x_apriori, P_apriori, zk, @state_equation, d);
    
    q = x_aposteriori(4:7)';
    orient(:,i) = q;
%     
%     [kalman_yaw(i), kalman_pitch(i), kalman_roll(i)] = quat2angle(q);
%     
%     gyro_roll(i) = gyro_roll(i-1) + angleRate(1,i)*d;
%     gyro_pitch(i) = gyro_pitch(i-1) + angleRate(2,i)*d;
%     gyro_yaw(i) = gyro_yaw(i-1) + angleRate(3,i)*d;
end

% [stm_orient_yaw, stm_orient_pitch, stm_orient_roll] = quat2angle(stm_q');
quest_yaw = quest_yaw * 180.0 / pi;
quest_pitch = quest_pitch * 180.0 / pi;
quest_roll = quest_roll * 180.0 / pi;

kalman_yaw = kalman_yaw * 180.0 / pi;
kalman_pitch = kalman_pitch * 180.0 / pi;
kalman_roll = kalman_roll * 180.0 / pi;

t = 1:N;
plot(t, stm_orient(1,:), t, orient(2,:));
%plot(magField(3,:))

% plot(t, quest_roll, t, gyro_roll, t, kalman_roll, t, stm_roll, t, stm_orient_roll);
% plot(t, quest_pitch, t, gyro_pitch, t, kalman_pitch, t, stm_pitch);
% plot(t, quest_yaw, t, gyro_yaw, t, kalman_yaw, t, stm_yaw);

