dz = 0.5;
w0 = 7.39;
b = w0^2;

A = [0 1;
    -w0^2 -2*dz*w0];

sys = ss(A, [0; b], [1 0], 0);
impulse(sys)
% bode(sys);
% grid

t = 0:0.01:3;
u = zeros(1, length(t));

x0 = [0.35, 0.35];

x = lsim(sys, u, t, x0);


%plot(t, x);