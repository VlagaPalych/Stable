logName = 'D:\Vlad\Projects\Stable\Logger\Logs\Ordinary\log1136.txt';

logFile = fopen(logName, 'r');
A = fscanf(logFile, '%f %f %f\n', [3 Inf]);

m = A(1:3,:);

% [center, radii, evecs] = ellipsoid_fit(m')

plot3(m(1,:), m(2,:), m(3,:), 'o')
axis equal;

mag_max = [-4800 -4800 -4800];
mag_min = [4800 4800 4800];
mag_bias = [0 0 0];
mag_scale = [0 0 0];


for i = 1:3
    mag_max(i) = max(m(i,:));
    mag_min(i) = min(m(i,:));
    mag_bias(i) = (mag_max(i) + mag_min(i)) / 2;
    mag_scale(i) = (mag_max(i) - mag_min(i)) / 2;
end

avg_rad = mean(mag_scale);
for i = 1:3
    mag_scale(i) = avg_rad / mag_scale(i);
end

for i = 1:3
    m(i,:) = m(i,:) - mag_bias(i);
    m(i,:) = m(i,:) * mag_scale(i);
end

plot3(m(1,:), m(2,:), m(3,:), 'o')
axis equal;

mag_bias
mag_scale

