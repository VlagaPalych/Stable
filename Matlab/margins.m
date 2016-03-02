params;

boat = tf(1, [a2 0 a0]);

Kp_min = -a0 + 0.1;
Kd_min = 0.1;

u_max = 1.5; % kg
angle_max = 0.23; % rad (13 grad)
arate_max = 0.23;

Kp_max = u_max / angle_max;
Kd_max = u_max / arate_max;

controller = pid(Kp_min, 0, Kd_min);
open_loop = controller * boat;
rlocus(open_loop)

% % Линии уровня функции
% [X,Y] = meshgrid(Kp_min:0.1:Kp_max, Kd_min:0.1:Kd_max);
% for i = 1:1:length(X)
%     for j = 1:1:length(X(1,:))
%         controller = pid(X(i,j), 0, Y(i,j));
%         open_loop = controller * boat;
% 
%         [Gm Pm, Wgm, Wpm] = margin(open_loop);
%         Z(i,j) = 20*log10(Gm);
%     end
% end

% [C,h] = contour(X,Y,Z);
% set(h,'ShowText','on','TextStep',get(h,'LevelStep')*2)
% colormap cool