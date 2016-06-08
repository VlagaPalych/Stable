params;

T_motor = 0.03; % sec

motor = tf(1, [T_motor 1]);

boat = tf(b, [a2 0 a0]);

mysys = motor * boat;

t = 0:0.001:0.5;

u = [ones(1,30) zeros(1,length(t)-30)]';

y = lsim(mysys,u,t);

%tf_est = tfest(iddata(y,u,0.001), 3, 0)

opt = ssestOptions('Advanced', ...
    struct('ErrorThreshold', 0, 'MaxSize', 250000, ...
    'StabilityThreshold', ...
    struct('s', 1e300, 'z', 1), ...
    'AutoInitThreshold', 1.05, ...
    'DDC', 'on'));
ss_est = ssest(iddata(y,u,0.001), 3, opt);

%y_tfest = lsim(tf_est,u,t);

y_ssest = lsim(ss_est,u,t);

plot(t,y,t,y_ssest)