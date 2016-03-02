g = 980;
rho = 1;

R = 8;
d = 1.9;
L = 43;

w = 4;
h = 8;

boat_m = 1000;
cargo_m = 2000;

cargo_I = cargo_m*(w^2 + h^2)/12 + cargo_m*(d + h/2)^2;
boat_I = boat_m*R^2 + boat_m*d^2 - 8*boat_m*d*R/3/pi; 
I = cargo_I + boat_I;

a0 = 0.01;
w0 = 0;

a_cur = a0;
a_next = 0;
w_cur = w0;
w_next = 0;
dt = 0.01;


% cargo
cargo_hor_x = -w:0.01:w;
cargo_hor_y1 = 0 * ones(1, length(cargo_hor_x));
cargo_hor_y2 = h * ones(1, length(cargo_hor_x));

cargo_ver_y = 0:0.01:h;
cargo_ver_x1 = -w * ones(1, length(cargo_ver_y));
cargo_ver_x2 = w * ones(1, length(cargo_ver_y));

% debug purposes
% x1 = (cargo_m*h/2 + boat_m*4*R/3/pi + (cargo_m+boat_m)*d)*g
% x2 = rho*g*L*d*(R^2*asin(sqrt(R^2-d^2*cos(a0)^2)/R) - d*cos(a0)*sqrt(R^2-d^2*cos(a0)^2))


for i = 1:1:100
    M_mg = (cargo_m*h/2 + boat_m*4*R/3/pi + (cargo_m+boat_m)*d)*g*sin(a_cur);
    M_a = rho*g*L*d*sin(a)*(R^2*asin(sqrt(R^2-d^2*cos(a)^2)/R) - d*cos(a)*sqrt(R^2-d^2*cos(a)^2));

    w_next = w_cur + (M_mg - M_a)*dt/I;
    a_next = a_cur + w_cur*dt;

    % boat
    boat_x = -R:0.01:R;
    boat_hs_y = -sqrt(R^2 - boat_x.^2);
    boat_line_y = 0 * ones(1, length(boat_x));
    
    a_cur = a_next;
    w_cur = w_next;
    
    if a_cur > 0.23
        break;
    end
    % tilt
    a = a_cur;

    tilted_boat_hs_x = boat_x * cos(a) - boat_hs_y * sin(a);
    tilted_boat_hs_y = boat_x * sin(a) + boat_hs_y * cos(a);

    tilted_boat_line_x = boat_x * cos(a) - boat_line_y * sin(a);
    tilted_boat_line_y = boat_x * sin(a) + boat_line_y * cos(a);

    tilted_cargo_hor_x1 = cargo_hor_x * cos(a) - cargo_hor_y1 * sin(a);
    tilted_cargo_hor_y1 = cargo_hor_x * sin(a) + cargo_hor_y1 * cos(a);

    tilted_cargo_hor_x2 = cargo_hor_x * cos(a) - cargo_hor_y2 * sin(a);
    tilted_cargo_hor_y2 = cargo_hor_x * sin(a) + cargo_hor_y2 * cos(a);

    tilted_cargo_ver_x1 = cargo_ver_x1 * cos(a) - cargo_ver_y * sin(a);
    tilted_cargo_ver_y1 = cargo_ver_x1 * sin(a) + cargo_ver_y * cos(a);

    tilted_cargo_ver_x2 = cargo_ver_x2 * cos(a) - cargo_ver_y * sin(a);
    tilted_cargo_ver_y2 = cargo_ver_x2 * sin(a) + cargo_ver_y * cos(a);

    plot(tilted_boat_hs_x, tilted_boat_hs_y, 'blue', ...
        tilted_boat_line_x, tilted_boat_line_y, 'blue', ...
        tilted_cargo_hor_x1, tilted_cargo_hor_y1, 'blue', ...
        tilted_cargo_hor_x2, tilted_cargo_hor_y2, 'blue', ...
        tilted_cargo_ver_x1, tilted_cargo_ver_y1, 'blue', ...
        tilted_cargo_ver_x2, tilted_cargo_ver_y2, 'blue')


    axis([-2*h, 2*h, -2*h, 2*h])
    axis equal
    pause(0.04);
end
