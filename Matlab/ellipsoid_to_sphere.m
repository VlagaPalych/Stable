[center, radii, evecs] = ellipsoid_fit([mx2 my2 mz2]);
V = evecs;
L = diag([1/radii(1) 1/radii(2) 1/radii(3)]);
S = V*L*inv(V);

mx2_shifted = mx2 - center(1);
my2_shifted = my2 - center(2);
mz2_shifted = mz2 - center(3);

e_data = [mx2_shifted my2_shifted mz2_shifted]';

s_data = S * e_data;

plot3(s_data(1,:), s_data(2,:), s_data(3,:), 'o')