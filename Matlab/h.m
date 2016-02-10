function [ a_p ] = transf( a_s, teta )
E = [teta(1) teta(2) teta(3)
    teta(4) teta(5) teta(6)
    teta(7) teta(8) teta(9)];

b = [teta(10) teta(11) teta(12)];

a_p = E*(a_s - b);

end

