%Cargo las medidas obtenidas
x=load('datos_crudos');

%Columna uno corresponde a los gramos medidos tengo que pasar a Newton
F=9.81/1000*x(:,1);

%Columna dos corresponde a la frecuencia medida, es decir, el doble de la
%velocidad angular del motor

w=pi*x(:,2);

plot(w,F,'x')
hold on


%Asumiendo modelo cuadrático
B=[w.^2 w ones(size(w),1)];

%param=((B'*B)^(-1))*B'*F;
param=(B'*B)\(B'*F);

wtest=[0:350];
Ftest=param(1)*(wtest.^2)+param(2)*wtest+param(3)*ones(size(wtest),1);
plot(wtest,Ftest)
hold on

msecuad=mse(F,param(1)*w.^2+param(2)*w+param(3)*ones(size(wtest),1))


%Asumiendo modelo cúbico
B=[w.^3 w.^2 w ones(size(w),1)];

%param=((B'*B)^(-1))*B'*F;
param=(B'*B)\(B'*F);

wtest=[0:350];
Ftest=param(1)*(wtest.^3)+param(2)*(wtest.^2)+param(3)*wtest+param(4)*ones(size(wtest),1);
plot(wtest,Ftest,'r')

msecub=mse(F,param(1)*w.^3+param(2)*w.^2+param(3)*w+param(4)*ones(size(wtest),1))