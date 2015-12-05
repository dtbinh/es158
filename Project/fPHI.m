function PHI = fPHI(tf,tc,P)

PHI = P.r*P.gamma*(fFun(0)-fFun(tf)+(1/beta)*fFun(0)*(1+exp(beta*tf)) -(2/beta)*fFun(tc)*(1+exp(-beta*(tf-tc))));
