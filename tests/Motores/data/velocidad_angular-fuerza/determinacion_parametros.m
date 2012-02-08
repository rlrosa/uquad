%Cargo las medidas obtenidas
x=load('datos_crudos');
%y=load('log2.txt');

%x=[x;y];
%Distancia del eje del motor al centro del quad
%D=0.285/2;
%Columna uno corresponde a los gramos medidos tengo que pasar a Newton m
Fm=9.81/1000*x(:,1);

%Columna dos corresponde a la frecuencia medida, es decir, el doble de la
%velocidad angular del motor

w=pi*x(:,2);

plot(w,Fm,'x')
title('Drag-Velocidad angular')
xlabel('Velocidad angular (rad/s)')
ylabel('Drag (Nm)')
hold on


%Asumiendo modelo cuadrático
B=[w.^2 w ones(size(w),1)];

%param=((B'*B)^(-1))*B'*F;
param=(B'*B)\(B'*Fm)

wtest=[0:350];
Ftest=param(1)*(wtest.^2)+param(2)*wtest+param(3)*ones(size(wtest),1);
plot(wtest,Ftest)
legend('Medias experimentales','Curva modelo cuadrático','Curva modelo cúbico')



hold on


 error=Fm-(param(1)*w.^2+param(2)*w+param(3)*ones(size(w),1));

 error_promedio=mean(error)
 sigma=std(error)



%Asumiendo modelo cúbico
B=[w.^3 w.^2 w ones(size(w),1)];

%param=((B'*B)^(-1))*B'*F;
param=(B'*B)\(B'*Fm);

wtest=[0:350];
Ftest=param(1)*(wtest.^3)+param(2)*(wtest.^2)+param(3)*wtest+param(4)*ones(size(wtest),1);
plot(wtest,Ftest,'r')
legend('Medias experimentales','Curva modelo cuadrático','Curva modelo cúbico')

error=Fm-(param(1)*(w.^3)+param(2)*(w.^2)+param(3)*w+param(4)*ones(size(wtest),1));

 error_promedio=mean(error)
 sigma=std(error)