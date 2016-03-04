% 740 - z+ circle
% 755 - z- circle
% 741 - x+ circle
% 742 - x- circle
% 745 - y+ circle
% 751 - y+ circle
% mag_calibr - all points
logName = 'D:\Vlad\Projects\Stable\Logger\BoardConsole\BoardConsole\log760.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f %f %f\n', [6 Inf]);

ax = A(1,:);
ay = A(2,:);
az = A(3,:);
mx = A(4,:)';
my = A(5,:)';
mz = A(6,:)';

plot3(mx, my, mz, 'bo');

% zp = [-0.023, -0.059, -0.286];
% zm = [0.018, -0.077, 0.298];
% xp = [-0.271, -0.061, 0];
% xm = [0.311, -0.064, -0.033];
% yp = [0.007, -0.33, 0.006];
% ym = [0.005, 0.107, 0.033];
% 
% x = xm - xp;
% y = ym - yp;
% z = zm - zp;
% 
% x = x / norm(x);
% y = y / norm(y);
% z = z / norm(z);
% 
% A1 = [x' y' z'];
% 
% A2 = inv(A1);
% 
% e_data = A2*[mx my mz]';
% ex = e_data(1,:);
% ey = e_data(2,:);
% ez = e_data(3,:);
e_data = [mx my mz]';

[center, radii, evecs] = ellipsoid_fit(e_data');

a = radii(1);
b = radii(2);
c = radii(3);

V = evecs;

% ex_shifted = ex - center(1);
% ey_shifted = ey - center(2);
% ez_shifted = ez - center(3);

L = diag([1/a 1/b 1/c]);

S = V*L*inv(V);

for i = 1:1:3
    e_data(i) = e_data(i) - center(i);
end

plot3(e_data(1), e_data(2), e_data(3), 'o');
s_data = S * e_data ;

sx = s_data(1,:);
sy = s_data(2,:);
sz = s_data(3,:);

plot3(sx, sy, sz, 'o');

% S*A2
% 
% S * center

