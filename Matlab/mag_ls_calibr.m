clear;
clc;

x0 = [0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0];

options.MaxFunEvals = 1000000;
options.MaxIter = 100000;
options.TolFun = 1e-20;
options.TolX = 1e-20;
%options.Display = 'iter';

%options.Algorithm = 'levenberg-marquardt';
[x,resnorm,residual,exitflag,output] = lsqnonlin(@myfun,x0, [], [], options)
%fminsearch(@myfun, x0, options)