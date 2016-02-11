% Алгоритм Антона двухуровневой стабилизации
% Основан на компенсации кинематических характеристик движения

a0 = pi/36; 
w0 = pi/6;
TAU = 1;
noise = 0.5;

len = 10*TAU / 0.01;
angles = zeros(1, len);
angvels = zeros(1, len);
angaccels = zeros(1, len);
pwms = zeros(1, len)

angles(1) = a0;
angvels(1) = w0;
angaccels(1) = 0;
pwms(1) = 0;

pwm = 1;

delay = 20;

for i = 2:1:len;    
%     if i < delay
%         e_real = 0;
%     else 
%         e_real = angaccels(i - delay+1);
%     end
    if -w0 * TAU/2/a0 < 1
        e = - w0/TAU - a0/TAU/TAU;
    else
        t1 = - 2*a0/w0;
        e = -w0 / t1;
    end
    
%     e_real = e_real * sign(e);
%     spwm=sign(pwm);
%     if spwm==0
%         spwm=1;
%     end;
%     if (abs(e) > e_real) 
%         pwm = pwm + sign(pwm)*30;
%     else 
%         pwm = pwm - sign(pwm)*30;
%     end


%     w = w0 + e_real * 0.01;
%     a0 = a0 + w0 * 0.01 + e_real * 5e-5;
    w = w0 + e * 0.01;
    a0 = a0 + w0 * 0.01 + e * 5e-5;
    w0 = w;
    
    angles(i) = a0;
    angvels(i) = w0;
    angaccels(i) = e;
    pwms(i) = pwm;
end

t = 1:1:length(angles);
plot(t, angles, t, angvels, t, angaccels, t, pwms/10000);

