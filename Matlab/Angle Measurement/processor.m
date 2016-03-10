logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log824.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f %f %f %f %f %f\n', [16 Inf]);

ax = A(1,:);
ay = A(2,:);
az = A(3,:);

mx = A(4,:);
my = A(5,:);
mz = A(6,:);

gx = A(7,:);
gy = A(8,:);
gz = A(9,:);

stm_q = A(10:13,:);
stm_roll = A(14,:) * pi / 180.0;
stm_pitch = A(15,:) * pi / 180.0;
stm_yaw = A(16,:) * pi / 180.0;

w1 = [ax(1) ay(1) az(1)]';
w2 = [mx(1) my(1) mz(1)]';

N = length(ax);
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
    v1 = [ax(i) ay(i), az(i)]';
    v2 = [mx(i) my(i), mz(i)]';
    
    q = quest(w1, w2, v1, v2);
    [quest_yaw(i), quest_pitch(i), quest_roll(i)] = quat2angle(q);
    
    zk = [gx(i) gy(i) gz(i) q]';
    [x_aposteriori, P_aposteriori, x_apriori, P_apriori] = my_kalman(F, Q, H, R, x_apriori, P_apriori, zk, @state_equation, d);
    
    q = x_aposteriori(4:7)';
    orient(:,i) = q;
    [kalman_yaw(i), kalman_pitch(i), kalman_roll(i)] = quat2angle(q);
    
    gyro_roll(i) = gyro_roll(i-1) + gx(i)*d;
    gyro_pitch(i) = gyro_pitch(i-1) + gy(i)*d;
    gyro_yaw(i) = gyro_yaw(i-1) + gz(i)*d;
end

[stm_orient_yaw, stm_orient_pitch, stm_orient_roll] = quat2angle(stm_q');

t = 1:N;
plot(t, stm_q(4,:), t, orient(4,:));

plot(t, quest_roll, t, gyro_roll, t, kalman_roll, t, stm_roll, t, stm_orient_roll);
plot(t, quest_pitch, t, gyro_pitch, t, kalman_pitch, t, stm_pitch);
plot(t, quest_yaw, t, gyro_yaw, t, kalman_yaw, t, stm_yaw);

