function foft = fFun(t,P)

foft = 0;
for k = 1:k
    foft = foft - k*(P.pi_k(k)*sin(P.pi_k(k)*t)+beta*cos(P.pi_k(k)*t))/(pi_k(k)*(pi_k(k)^2+P.beta^2));
end
