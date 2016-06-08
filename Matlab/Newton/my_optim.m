function [ x_ans ] = my_optim( f, g, H, x0, x_tol, f_tol )
    x_ans = my_multidim_newton(g, H, x0, x_tol, f_tol);
end


