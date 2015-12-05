function PHI = fPHI(tf,tc,P)

PHI = P.r*P.gamma*(fFun(0)-fFun(tf)+(1/P.beta)*fFun(0)*(1+exp(P.beta*tf)) -(2/P.beta)*fFun(tc)*(1+exp(-P.beta*(tf-tc))));
