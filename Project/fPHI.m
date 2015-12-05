function PHI = fPHI(tc,tf,P)

PHI = P.r*P.gamma*(fFun(0,P)-fFun(tf,P)+(1/P.beta)*fFun(0,P)*(1+exp(P.beta*tf)) -(2/P.beta)*fFun(tc,P)*(1+exp(-P.beta*(tf-tc))));
