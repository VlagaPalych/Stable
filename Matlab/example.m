clear;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Chapter 1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
J = 1;
b = 4;
kR = 40;

A = [0 1; -kR/J -b/J];
B = [0; 1/J];
C = [1 0];
D = [0];

JbkR = ss(A, B, C, D);

JbkRtf = tf(JbkR);

JbkRzpk = zpk(JbkR);

[num, den] = tfdata(JbkR, 'v');

[z, p, k] = zpkdata(JbkR, 'v');

JbkRss = ss(JbkRtf);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Chapter 2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t = [0:0.01:4];

U = [zeros(size(t))];

x0 = [0.4; 0.2];

CharPoly = poly(A);

Poles = roots(CharPoly);

Eigs0 = eig(A);

damp(A);

[Yo, t, Xo] = lsim(JbkR, U, t, x0);

Xo(101, :);

X1 = expm(A*1)*x0;

figure;
subplot(211), plot(t, Xo(:,1)); grid;
axis([0 4 -0.2 0.5]);
set(gca, 'FontSize', 18);
ylabel('{\itx}_1 (\itrad)');
subplot(212), plot(t, Xo(:, 2)); grid;
axis([0 4 -2 1]);
set(gca, 'FontSize', 18);
xlabel('\ittime (sec)');
ylabel('{\itx}_2 (\itrad/s)');


[Tdcf, E] = eig(A)

Adcf = inv(Tdcf)*A*Tdcf
Bdcf = inv(Tdcf)*B
Cdcf = C*Tdcf
Ddcf = D

[JbkRm, Tm] = canon(JbkR, 'modal');

Am = JbkRm.a
Bm = JbkRm.b
Cm = JbkRm.c
Dm = JbkRm.d
























