alpha_pend_no_cargo_array = [0.71 0.53 0.52];
T_pend_no_cargo_array = [1.0741 1.10384 1.1553];

% параметры маятника лодки без груза
alpha_pend_no_cargo = mean(alpha_pend_no_cargo_array);  % затухание
T_pend_no_cargo = mean(T_pend_no_cargo_array);    % средний период колебаний, с

% параметры автоколебаний лодки без груза
T_self_no_cargo = 0.311;     % средний период колебаний, с
k_self_no_cargo = 4;


w_pend_no_cargo = 2*pi/T_pend_no_cargo;

a1_no_cargo = 2*alpha_pend_no_cargo;
a0_no_cargo = w_pend_no_cargo^2 + alpha_pend_no_cargo^2;

w_self_no_cargo = 2*pi/T_self_no_cargo;

T = a1_no_cargo / (w_self_no_cargo^2 - a0_no_cargo);
b = (w_self_no_cargo^2*(T*a1_no_cargo + 1) - a0_no_cargo) / ...
    k_self_no_cargo;


T_self_light_cargo = 0.30465;
w_self_light_cargo = 2*pi/T_self_light_cargo;
k_self_light_cargo = 5.5;

A = [T 1;
     1 -w_self_light_cargo^2*T];
 
B = [w_self_light_cargo^2*T
     w_self_light_cargo^2*T - b*k_self_light_cargo];
x = linsolve(A,B);

a0_light_cargo = x(1);
a1_light_cargo = x(2);

motor = tf(1, [T, 1]);
nc_boat = tf(b, [1 a1_no_cargo, a0_no_cargo]);
lc_boat = tf(b, [1 a1_light_cargo, a0_light_cargo]);

obj = motor * lc_boat;

con = tf([1 5.5], 1);

ol = con*obj;

cl = ol / (1 + ol);

margin(cl)

% k_d_min = (-T*a0_no_cargo - a1_no_cargo) / b
% k_p_min = -a0_no_cargo / b

