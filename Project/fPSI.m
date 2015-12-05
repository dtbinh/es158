function psi = fPSI(tf,tc,P)

psi = P.m * P.g * P.r / (P.epsilon * P.beta) * (exp(-P.beta * tf) - 2 * exp(-P.beta * (tf - tc)) - 1) - (P.m * P.g * P.r / P.epsilon) * tf;
