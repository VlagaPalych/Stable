function [ x_ans ] = my_multidim_newton( f, J, x0, x_tol, f_tol )
    
    x_cur = x0;
    x_next = x0;
    dx = x_tol + 1;
    df = f_tol + 1;
    
    while (dx > x_tol) || (df > f_tol)
        x_cur = x_next;
        x_next = x_cur - inv(J(x_cur))*f(x_cur);

        dx = norm(x_cur - x_next);
        df = norm(f(x_next));
    end
    
    x_ans = x_next;
end


