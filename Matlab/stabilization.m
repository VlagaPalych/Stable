clear;
clc;

% Jy'' - mghy = bu

J = 100;
m = 80;
g = 9.8;
h = 1;

a0 = m*g*h
a2 = J

b = 1

% x1 = y
% x2 = y'
%
% x1' = x2
% x2' = (mgh/J)x1 + bu

% State-space matrices
A = [0 1; (m*g*h/J) 0];
B = [0; b];
C = [1 0];
D = 0;

Object = ss(A, B, C, D);

H = tf(Object)

PO = 20;
Ts = 0.5;

tmp = log(PO/100);

ksi = sqrt(tmp^2 / (pi^2 + tmp^2));
Wn = 8 / ksi;

l1 = 3.1627;%- ksi*Wn + Wn*sqrt(ksi^2 - 1);
l2 = -2.1627;%- ksi*Wn - Wn*sqrt(ksi^2 - 1);

K = place(A, B, [l1, l2])

k1 = (-l1-l2)/b
k2 = (1/b)*((a0/a2) + l2*l1)

Acl = A - B*K;
G = -inv(C*inv(Acl)*B)
Bcl = B * G;
Ccl = C;
Dcl = D;

ClosedLoop = ss(Acl, Bcl, Ccl, Dcl);

t = 0:0.001:1;
r = zeros(1, length(t));

[y, t, x] = lsim(ClosedLoop, r, t, [pi/9, pi/9]);

s = 0.01;
Xorig = -1:s:1;
Yorig = zeros(1, 2 / s + 1);

% for i = 1:1:length(t)
%     a = y(i);
%     M = [cos(a) -sin(a);
%         sin(a) cos(a)];
% 
%     rotated = M * [Xorig; Yorig];
%     Xrot = rotated(1, ':');
%     Yrot = rotated(2, ':');
% 
%     plot(Xrot, Yrot);
%     axis([-1.5 1.5 -1.5 1.5])
%     pause(0.001);
% end

%plot(t, y);
% 
%step(ClosedLoop);
Hcl = tf(ClosedLoop);
% 
X = minreal(Hcl / (H*(1 - Hcl)));
OpenLoop = minreal(H*X)
nyquist(OpenLoop)
%step(OpenLoop)

% 
% % step(OpenLoop)
% Delay = tf(1,1,'InputDelay',0.1);
% 
% WithDelay = ss(tf(X*Delay*H));
% %[Ade, Bde, Cde, Dde] = ssdata(WithDelay)
% 
% % step(ClosedLoop)


