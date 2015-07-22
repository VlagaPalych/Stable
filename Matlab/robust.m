%% Robust stability research

clear;
clc;

%% Полноразмерная доска
% m = [40 120];               % масса груза
% h = [1 2];                  % высота груза
% m_constr = [0 20];          % масса конструкции
% w = [0.6 2];                % ширина доски
% w_mot = [1 2];              % расстояние между моторами
% d = [1.5 3];                % длина доски
% eta = [0.0007978 0.001002]; % вязкость воды
% k_f = [-0.5 -0.04];         % коэффициент тяги от частоты
% g = 9.8;

%% Макет
m = [0.16 0.19];               % масса груза
h = [0.4 0.45];                  % высота груза
m_constr = [0 1.3];          % масса конструкции
w = [0.15 0.16];                % ширина доски
w_mot = [0.39 0.41];              % расстояние между моторами
d = [0.43 0.47];                % длина доски
eta = [0.0007978 0.001002]; % вязкость воды
k_f = [-1.9e-7 -2e-8];         % коэффициент тяги от частоты
g = 9.8;

m_full = m + m_constr;
J = (m_full/12).*(4*h.^2 + w.^2);
%b = w_mot .* k_f / 2;
b = w_mot / 2;
c = 3 * sqrt(pi) * eta .* w .* sqrt(w.*d);
c(1) = c(1) / 5;
c(2) = c(2) * 5;

a2 = J;
a1 = c;
a0(1) = -m(2)*g*h(2)/2;
a0(2) = -m(1)*g*h(1)/2;

if b(1) < 0
    low = a2(1);
    up = a2(2);
    if (a2(1) < 0)
        a2(1) = up / b(1);
        a2(2) = low / b(2);
    elseif a2(1) > 0
        a2(1) = up / b(2);
        a2(2) = low / b(1);
    end
    
    low = a1(1);
    up = a1(2);
    if (a1(1) < 0)
        a1(1) = up / b(1);
        a1(2) = loe / b(2);
    elseif a1(1) > 0
        a1(1) = up / b(2);
        a1(2) = low / b(1);
    end
    
    low = a0(1);
    up = a0(2);
    if (a0(1) < 0)
    	a0(1) = up / b(1);
        a0(2) = low / b(2);
    elseif a0(1) > 0
        a0(1) = up / b(2);
        a0(2) = low / b(1);
    end
elseif b(1) > 0
    low = a2(1);
    up = a2(2);
    if (a2(1) < 0)
        a2(1) = low / b(1);
        a2(2) = up / b(2);
    elseif a2(1) > 0
        a2(1) = low / b(2);
        a2(2) = up / b(1);
    end
    
    low = a1(1);
    up = a1(2);
    if (a1(1) < 0)
    	a1(1) = low / b(1);
        a1(2) = up / b(2);
    elseif a1(1) > 0
        a1(1) = low / b(2);
        a1(2) = up / b(1);
    end
    
    low = a0(1);
    up = a2(2);
    if (a0(1) < 0)
    	a0(1) = low / b(1);
        a0(2) = up / b(2);
    elseif a0(1) > 0
        a0(1) = low / b(2);
        a0(2) = up / b(1);
    end
end

a0
a1
a2

P = [a2(2) a1(1) a0(1);
     a2(1) a1(1) a0(2);
     a2(1) a1(2) a0(2);
     a2(2) a1(2) a0(1)];
 
% Xleft = min(-P(:,3));
% Xright = max(-P(:,3));
% 
% Xdist = Xright - Xleft;
% Xleft = Xleft - Xdist*0.1;
% Xright = Xright + Xdist*0.1;
% 
% x2 = Xleft:0.1:Xright;
% 
% Ylow = min(-P(:,2));
% Yhigh = max(-P(:,2));
% 
% Ydist = Yhigh - Ylow;
% Ylow = Ylow - Ydist*0.1;
% Yhigh = Yhigh + Ydist*0.1;
% 
% y1 = Ylow:0.1:Yhigh;
% 
% x11 = -P(1, 3)*ones(1, length(y1));
% y12 = -P(1, 2)*ones(1, length(x2));
% 
% x21 = -P(2, 3)*ones(1, length(y1));
% y22 = -P(2, 2)*ones(1, length(x2));
% 
% x31 = -P(3, 3)*ones(1, length(y1));
% y32 = -P(3, 2)*ones(1, length(x2));
% 
% x41 = -P(4, 3)*ones(1, length(y1));
% y42 = -P(4, 2)*ones(1, length(x2));
% 
% plot(x11, y1, 'b', x2, y12, 'b', x21, y1, 'g', x2, y22, 'g', x31, y1, 'r', x2, y32, 'r', x41, y1, 'y', x2, y42, 'y');

% 4 полинома
% Для каждого полинома проверяем корни в левой полуплоскости для трех точек
% 1) правее/левее -a0, выше -а1
% 2) правее/левее -а0, ниже -а1
% 3) левее/правее -а0, ниже -а1
% Правее/левее зависит от того а2>0/a2<0 

points = zeros(size(4,6));
pointsType = zeros(size(4,6));
polys = zeros(size(12,3));

for i = 1:1:4 
    if P(i,1) > 0
        points(i,1) = -P(i,3) + 1;
    else
        points(i,1) = -P(i,3) - 1;
    end
    points(i,2) = -P(i,2) + 1;
    polys(3*(i-1)+1,1) = P(i,1);
    polys(3*(i-1)+1,2) = P(i,2)+points(i,2);
    polys(3*(i-1)+1,3) = P(i,3)+points(i,1);
    
    if P(i,1) > 0
        points(i,3) = -P(i,3) + 1;
    else
        points(i,3) = -P(i,3) - 1;
    end
    points(i,4) = -P(i,2) - 1;
    polys(3*(i-1)+2,1) = P(i,1);
    polys(3*(i-1)+2,2) = P(i,2)+points(i,4);
    polys(3*(i-1)+2,3) = P(i,3)+points(i,3);
    
    if P(i,1) > 0
        points(i,5) = -P(i,3) - 1;
    else
        points(i,5) = -P(i,3) + 1;
    end
    points(i,6) = -P(i,2) - 1;
    polys(3*(i-1)+3,1) = P(i,1);
    polys(3*(i-1)+3,2) = P(i,2)+points(i,6);
    polys(3*(i-1)+3,3) = P(i,3)+points(i,5);
end

% Проверяем точки на устойчивость
% Определяем тип устойчивой точки
leftLowerCorner = zeros(1,2);
leftUpperCorner = zeros(1,2);
rightLowerCorner = zeros(1,2);
rightUpperCorner = zeros(1,2);

for i = 1:1:4
    for j = 1:1:3
        poly = polys(3*(i-1)+j,:);
        eigvals = roots(poly);
        testCorner = zeros(1,2);
        if (real(eigvals(1)) < 0) && (real(eigvals(2)) < 0)
            if poly(1) > 0 
                if j == 1 
                    testCorner(1) = points(i,2*j-1) - 1;
                    testCorner(2) = points(i,2*j) - 1;
                    if (leftLowerCorner(1) == 0) && (leftLowerCorner(2) == 0)
                        leftLowerCorner = testCorner;
                    else
                        if testCorner(1) > leftLowerCorner(1)
                            leftLowerCorner(1) = testCorner(1);
                        end
                        if testCorner(2) > leftLowerCorner(2)
                            leftLowerCorner(2) = testCorner(2);
                        end
                    end
                elseif j == 2
                    testCorner(1) = points(i,2*j-1) - 1;
                    testCorner(2) = points(i,2*j) + 1;
                    if (leftUpperCorner(1) == 0) && (leftUpperCorner(2) == 0)
                        leftUpperCorner = testCorner;
                    else
                        if testCorner(1) > leftUpperCorner(1)
                            leftUpperCorner(1) = testCorner(1);
                        end
                        if testCorner(2) < leftUpperCorner(2)
                            leftUpperCorner(2) = testCorner(2);
                        end
                    end
                elseif j == 3
                    disp('Not a corner');
                end
            else
                if j == 1 
                    testCorner(1) = points(i,2*j-1) + 1;
                    testCorner(2) = points(i,2*j) - 1;
                    if (rightLowerCorner(1) == 0) && (rightLowerCorner(2) == 0)
                        rightLowerCorner = testCorner;
                    else
                        if testCorner(1) < rightLowerCorner(1)
                            rightLowerCorner(1) = testCorner(1);
                        end
                        if testCorner(2) > rightLowerCorner(2)
                            rightLowerCorner(2) = testCorner(2);
                        end
                    end
                elseif j == 2
                    testCorner(1) = points(i,2*j-1) + 1;
                    testCorner(2) = points(i,2*j) + 1;
                    if (rightUpperCorner(1) == 0) && (rightUpperCorner(2) == 0)
                        rightUpperCorner = testCorner;
                    else
                        if testCorner(1) < rightUpperCorner(1)
                            rightUpperCorner(1) = testCorner(1);
                        end
                        if testCorner(2) < rightUpperCorner(2)
                            rightUpperCorner(2) = testCorner(2);
                        end
                    end
                elseif j == 3
                    disp('Not a corner');
                end
            end
        end
    end
end

leftLowerCorner
leftUpperCorner
rightLowerCorner
rightUpperCorner

% k1 = 1;
% k2 = 1;
% 
% poly = [P4a2 (P4a1+k2) (P4a0+k1)]
% roots(poly)

