function foft = fFun(t,P)

foft = 0;
for k = 1:P.k
    foft = foft - k*(P.pi_k(k)*sin(P.pi_k(k)*t)+P.beta*cos(P.pi_k(k)*t))/(P.pi_k(k)*(P.pi_k(k)^2+P.beta^2));
end
