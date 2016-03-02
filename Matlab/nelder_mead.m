function [ x_min ] = nelder_mead( f, x_init, l, eps )

% Линии уровня функции
[X,Y] = meshgrid(x_init(1)-l(1):0.1:x_init(1)+l(1),x_init(2)-l(2):0.1:x_init(2)+l(2));
for i = 1:1:length(X)
    for j = 1:1:length(X(1,:))
        Z(i,j) = f([X(i,j) Y(i,j)]);
    end
end

[C,h] = contour(X,Y,Z);
set(h,'ShowText','on','TextStep',get(h,'LevelStep')*2)
colormap cool
hold on

% Параметры изменений симплекса
alpha = 1;
gamma = 2;
rho = 0.5;
sigma = 0.5;

% Генерируем начальный симплекс
n = length(x_init);
x = zeros(n+1, n);

x(1,:) = x_init;
for i = 2:1:n+1
    for j = 1:1:n
        if j == i-1
            x(i,j) = x_init(j) + l(j);
        else
            x(i,j) = x_init(j);
        end
    end
end

plot_x = [x(:,1)', x(1,1)];
plot_y = [x(:,2)', x(1,2)];
line_descriptor = line('XData', plot_x, 'YData', plot_y)
hold off

f_vals = zeros(1, n+1);
for i = 1:1:n+1
    f_vals(i) = f(x(i,:));
end

% Основной цикл
while 1
    f_vals(n+1) = f(x(n+1,:));
    
    % exit condition
    average = mean(f_vals);
    criteria = 0;
    for i = 1:1:n+1
        criteria = criteria + (f_vals(i) - average)^2;
    end
    if sqrt(criteria / (n+1)) <= eps
        break;
    end   
    
    % sorting
    [f_vals, indices] = sort(f_vals);     
    x = x(indices,:);

    x_o = (sum(x) - x(n+1,:)) / n;
    

    % reflection
    x_r = (1+alpha)*x_o - alpha*x(n+1,:);
    if f(x_r) < f_vals(1)
        x_e = (1 + gamma)*x_r - gamma*x_o;
        if f(x_e) < f_vals(1)
            x(n+1,:) = x_e; % expansion
        else
            x(n+1,:) = x_r; % reflection
        end
    else if f(x_r) > f_vals(n)
        if f(x_r) <= f(x(n+1,:))
            x(n+1,:) = x_r; % reflection
        end
            
        x_c = rho*x(n+1,:) + (1-rho)*x_o;
        if f(x_c) > f(x(n+1,:))
            for i = 2:1:n+1 % multiple contraction
                x(i,:) = (x(i,:) + x(1,:)) / 2;
            end
        else
            x(n+1,:) = x_c; % contraction
        end
    else
        x(n+1,:) = x_r; % reflection
        end
    
    plot_x = [x(:,1)', x(1,1)];
    plot_y = [x(:,2)', x(1,2)];
    set(line_descriptor, 'XData', plot_x);
    set(line_descriptor, 'YData', plot_y);
end

x_min = x(1,:);
end
