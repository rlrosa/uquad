%Minimizar bolas
x0=[0 0 0 1];
[X,RESNORM,RESIDUAL,EXITFLAG]=lsqnonlin(@cost_bola,x0,[],[],optimset('MaxFunEvals',10000,'MaxIter',1000));

mean(RESIDUAL)
std(RESIDUAL)