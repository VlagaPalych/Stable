dz = 0.5;
w0 = 5.5;

A = [0 1;
    -w0^2 -2*dz*w0];

sys = ss(A, [0; 0], [1 0], 0);

t = 0:0.01:3;
u = zeros(1, length(t));

x0 = [0.35, 0.35];

x = lsim(sys, u, t, x0);

plot(t, x);