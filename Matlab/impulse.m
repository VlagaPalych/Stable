function [ h ] = impulse( params, t )

p = params(1);
q = params(2);

h = (q / (2 * sqrt(p^2/4 - q))) * (exp(t * (-p/2 + sqrt(p^2/4 - q))) - exp(t * (-p/2 - sqrt(p^2/4 - q))));

end

