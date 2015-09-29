a1 = 0.0005;
a0 = 3;
b = 0.2;

A = [0 1;
    -a0 -a1];

B = [0;
    b];

C = [1 0];

OpenLoop = ss(A, B, C, 0);

k1 = 25;
k2 = 5;

K = [k1 k2];

Acl = A - B*K;
G = -inv(C*inv(Acl)*B);
Bcl = B * G;
Ccl = C;

ClosedLoop = ss(Acl, Bcl, Ccl, 0);

dt = 0.01;
t = 0:dt:5;
r = zeros(1, length(t));

[y, t, x] = lsim(ClosedLoop, r, t, [pi/9, pi/9]);

plot(t, y);

n = length(t);
Aid = zeros(10, 3);
Bid = zeros(10, 1);
for j = 1:1:10
    Aid(j, 1) = (y(j+2) - y(j+1)) / dt;
    Aid(j, 2) = y(j+2);
    Xj = x(j+2,:);
    Aid(j, 3) = -dot(K,Xj);
    Bid(j) = (2*y(j+1) - y(j+2) - y(j)) / dt / dt;
end

for i = 13:1:n
    Awork = transpose(Aid) * Aid;
    Bwork = transpose(Aid) * Bid;
    w = linsolve(Awork, Bwork)
    
    row = rem(i-3, 10) + 1;
    Aid(row, 1) = (y(i) - y(i-1)) / dt;
    Aid(row, 2) = y(i);
    Xi = x(i,:);
    Aid(row, 3) = -dot(K,Xi);
    Bid(row) = (2*y(i-1) - y(i) - y(i-2)) / dt / dt;
end