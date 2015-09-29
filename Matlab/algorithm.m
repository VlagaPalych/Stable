a0 = pi/18; 
w0 = pi/18;
TAU = 1;

w1 = 1.5 * pi / TAU;
phi = pi / 4;

len = 2*TAU / 0.01;
angles = zeros(1, len);
angvels = zeros(1, len);

angles(1) = a0;
angvels(1) = w0;

for i = 2:1:len;
    k = (-a0 - w0*TAU/2 + w0/w1) / (TAU^2/2 - (TAU/2/w1)*(cos(w1*TAU/2 + phi) - cos(phi)));
    e = 0;

    if (a0 > 0) && (w0 > 0)
        e = k;
    end

    if (a0 > 0) && (w0 < 0)
        sin_int = (w0/w1) * (cos(phi) - cos(w1*TAU/2 + phi));

        if -sin_int < a0
            e = k;
        else 
            da = - sin_int - a0;
            par = 8 * da / TAU / TAU;
%             a = 1 - cos(w1*TAU/2);
%             b = sin(w1*TAU/2);
%             c = (w1/w0) * a;
%             phi = 2 * atan((sqrt(a^2 + b^2 - c^2) + b)/(a + c));
            e = w0*w1*cos(phi) - par;
        end
    end

    w = w0 + e * 0.01;
    a0 = a0 + w0 * 0.01 + e * 5e-5;
    w0 = w;
    
    angles(i) = a0;
    angvels(i) = w0;
end

t = 1:1:length(angles);
plot(t, angles, t, angvels);

