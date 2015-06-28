%% Аппроксимация зависимости тяги от шима

pwm = [1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1500, 1550];
force1 = [0, 0.13, 0.54, 1, 1.56, 2.15, 2.64, 3.05, 3.5, 3.9];

force2 = force1 / 3.87;

poly = polyfit(pwm, force2, 2)

poly_force = zeros(1, length(pwm));
for i = 1:1:length(pwm)
    poly_force(i) = polyval(poly, pwm(i));
end

plot(pwm, force2, pwm, poly_force);