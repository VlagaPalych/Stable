logName = 'D:\Vlad\Projects\Stable\Logger\Logs\Ordinary\log1208.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f %f %f %f %f %f %f\n', [9 Inf]);

m = A(1:3,:);

% [center, radii, evecs] = ellipsoid_fit(m')

plot3(m(1,:), m(2,:), m(3,:), 'o')
axis equal;

N = length(m(1,:));
m_light = m;
m_light_num = length(m(1,:));
% m_light = zeros(size(m));
% m_light(:,1) = m(:,1);
% m_light_num = 1;
% for i = 2:N
%     %m_light
%     %m(:,i)
%     no_match = 1;
%     for j = 1:m_light_num
%         len = norm(m_light(:,j));
%         dist = norm(m(:,i) - m_light(:,j));
%         
%         if dist < 0.12*len
%             no_match = 0;
%             break
%         end
%     end
%     if no_match == 1
%         m_light_num = m_light_num+1;
%         m_light(:,m_light_num+1) = m(:,i);
%     end
% end
% 
% plot3(m_light(1,1:m_light_num), m_light(2,1:m_light_num), m_light(3,1:m_light_num), 'o');

C = zeros(m_light_num, 9);
d = zeros(m_light_num, 1);
for i = 1:m_light_num
    x = m_light(1,i);
    y = m_light(2,i);
    z = m_light(3,i);
    C(i,:) = [x, y, z, -y^2, -z^2, -x*y, -x*z, -y*z, 1];
    d(i) = x^2;
end

A = diag([0 0 0 -1 -1 -1 -1 -1 0]);
b = [0 0 0 -1e-12 -1e-12 -1e-12 -1e-12 -1e-12 0];
X = linsolve(C, d);
for i = 4:8
    if X(i) < 0
        X(i) = 1e-12;
    end
end
%-1.54412 8.49701 -9.45264 0.873806 0.835283 1e-12 0.0811884 0.0180809 172.254
A = [2      X(6) X(7)
     X(6) 2*X(4) X(8)
     X(7) X(8) 2*X(5)];
b = [X(1) X(2) X(3)]';
m0 = linsolve(A,b);
x0 = m0(1);
y0 = m0(1);
z0 = m0(1);

R = 1;
a2 = X(9) + x0^2 + X(4)*y0^2 + X(5)*z0^2 + X(6)*x0*y0 + X(7)*x0*z0 + X(8)*y0*z0;
b2 = a2 / X(4);
c2 = a2 / X(5);
d2 = a2 / X(6);
e2 = a2 / X(7);
f2 = a2 / X(8);

a = sqrt(a2);
b = sqrt(b2);
c = sqrt(c2);
d = sqrt(d2);
e = sqrt(e2);
f = sqrt(f2);

SF = diag([1/a 1/b 1/c]);

cos_xy = 2*a*b/d2;
cos_xz = 2*a*c/e2;
cos_yz = 2*b*c/f2;

sin_xy = sqrt(1 - cos_xy^2);
sin_xz = sqrt(1 - cos_xz^2);
sin_yz = sqrt(1 - cos_yz^2);

A = [1          cos_xy          cos_xz
     0          sin_xy          cos_yz*sin_xy
     0          0               sqrt(1 - cos_xz^2 - cos_yz^2*sin_xy^2)];
 
% B = [1          0                   0
%      0          sin_xy              cos_yz*(sin_xy-1)
%      0          0                   sqrt(1 - cos_xz^2 - cos_yz^2*sin_xy^2)]
%  
% SI = linsolve(A, B)

m_cal = zeros(size(m));
for i = 1:N
    m_cal(:,i) = m(:,i) - m0;
    m_cal(:,i) = SF * m_cal(:,i);
    m_cal(:,i) = A * m_cal(:,i);
end

plot3(m_cal(1,:), m_cal(2,:), m_cal(3,:), 'o')
axis equal;

m0
F = A * SF

% mag_max = [-4800 -4800 -4800];
% mag_min = [4800 4800 4800];
% mag_bias = [0 0 0];
% mag_scale = [0 0 0];
% 
% 
% for i = 1:3
%     mag_max(i) = max(m(i,:));
%     mag_min(i) = min(m(i,:));
%     mag_bias(i) = (mag_max(i) + mag_min(i)) / 2;
%     mag_scale(i) = (mag_max(i) - mag_min(i)) / 2;
% end
% 
% avg_rad = mean(mag_scale);
% for i = 1:3
%     mag_scale(i) = avg_rad / mag_scale(i);
% end
% 
% for i = 1:3
%     m(i,:) = m(i,:) - mag_bias(i);
%     m(i,:) = m(i,:) * mag_scale(i);
% end
% 
% plot3(m(1,:), m(2,:), m(3,:), 'o')
% axis equal;
% 
% mag_bias
% mag_scale

