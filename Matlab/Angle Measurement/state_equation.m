function out = state_equation( in )
    out(1,1) = 0;
    out(2,1) = 0;
    out(3,1) = 0;
    out(4,1) = -0.5*(in(1)*in(5) + in(2)*in(6) + in(3)*in(7));
    out(5,1) = 0.5*(in(1)*in(4) + in(3)*in(6) - in(2)*in(7));
    out(6,1) = 0.5*(in(2)*in(4) + in(1)*in(7) - in(3)*in(5));
    out(7,1) = 0.5*(in(3)*in(4) + in(2)*in(5) - in(1)*in(6));
end

