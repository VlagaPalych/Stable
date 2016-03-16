clear;
clc;

x0 = [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0];

options.MaxFunEvals = 1000000;
options.MaxIter = 100000;
options.TolFun = 1e-10;
options.TolX = 1e-10;
%options.Display = 'iter';

lb = [-0.2 -0.2 -0.2 -0.2 -0.2 -0.2, 0.8, 0.8, -0.5, -0.5, -0.5];
ub = [0.2 0.2 0.2 0.2 0.2 0.2, 1.2, 1.2, 0.5, 0.5, 0.5];

%options.Algorithm = 'levenberg-marquardt';
[x,resnorm,residual,exitflag,output] = lsqnonlin(@myfun,x0, lb, ub, options)
%fminsearch(@myfun, x0, options)