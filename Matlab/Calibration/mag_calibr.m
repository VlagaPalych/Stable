clear;
clc;

x0 = [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0];

options.MaxFunEvals = 1000000;
options.MaxIter = 100000;
options.TolFun = 1e-10;
options.TolX = 1e-10;
%options.Display = 'iter';

lb = [-0.2 -0.2 -0.2 -0.2 -0.2 -0.2, 0.8, 0.8, -0.5, -0.5, -0.5];
ub = [0.2 0.2 0.2 0.2 0.2 0.2, 1.2, 1.2, 0.5, 0.5, 0.5];

%options.Algorithm = 'levenberg-marquardt';
[x,resnorm,residual,exitflag,output] = lsqnonlin(@myfun2,x0);
%x = fminsearch(@myfun2, x0, options);
G = [x(1) x(2) x(3)
     x(4) x(5) x(6)
     x(7) x(8) x(9)];

F = inv(G)
%fminunc(@myfun, x0, options) 