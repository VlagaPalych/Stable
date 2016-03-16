function Q = quest(w1, w2, v1, v2)
    a1 = 0.5;
    a2 = 0.5;
    
    cos_teta_cf = dot(v1,v2)*dot(w1,w2) + norm(cross(v1,v2))*norm(cross(w1,w2));
    lambda_max = sqrt(a1^2 + 2*a1*a2*cos_teta_cf + a2^2);
    
    S = a1*(w1*v1' + v1*w1') + a2*(w2*v2' + v2*w2');
    Z = a1*cross(w1,v1) + a2*cross(w2,v2);
    
    sigma = 0.5*trace(S);
    delta = det(S);
    kappa = trace(delta*inv(S));
    
    alpha = lambda_max^2 - sigma^2 + kappa;
    beta = lambda_max - sigma;
    gamma = (lambda_max + sigma)*alpha - delta;
    
    X = (alpha*eye(3) + beta*S + S*S)*Z;
    
    Q = [gamma; X]';
    Q = Q / norm(Q);
end