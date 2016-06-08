syms s T a1 a0 b k p;

P = T*s^3 + (T*a1+1)*s^2 + (T*a0+a1)*s + (a0+b*k);

pretty(solve(T*s^3 + (T*a1+1)*s^2 + (T*a0+a1)*s + (a0+b*k) == 0, s))