function [x_aposteriori, P_aposteriori, x_apriori_next, P_apriori_next] = my_kalman(Fk, Qk, Hk, Rk, x_apriori, P_apriori, zk, f, DELTA)
    Kk = P_apriori*Hk'*inv(Hk*P_apriori*Hk' + Rk);

    x_aposteriori = x_apriori + Kk*(zk - x_apriori);
    P_aposteriori = (eye(length(zk)) - Kk*hk)*P_apriori;
    
    x_apriori_next = x_aposteriori + f(x_aposteriori)*DELTA;
    P_apriori_next = Fk*P_aposteriori*Fk' + Qk;
end