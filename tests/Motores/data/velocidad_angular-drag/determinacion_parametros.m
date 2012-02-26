%Cargo las medidas obtenidas
%x=load('logs_02_21/log_02_21_n01.txt');
x=load('logs_02_21/log_02_21_n02.txt');
%x=load('logs_02_21/log_02_21_n03.txt');

%Promedio las tres series

%x=(y+z)/2

%Distancia del eje del motor al centro del quad
D=0.293;
%Columna uno corresponde a los gramos medidos tengo que pasar a Newton m
Fm=D*9.81/1000*x(:,2);

%Columna dos corresponde a la frecuencia medida, es decir, el doble de la
%velocidad angular del motor

w=pi*x(:,3);

plot(w,Fm,'x')
%title('Drag-Velocidad angular')
xlabel('Velocidad angular (rad/s)')
ylabel('Drag (Nm)')
hold on


%Asumiendo modelo cuadrático
B=[w.^2 w];

%param=((B'*B)^(-1))*B'*F;
param=(B'*B)\(B'*Fm)

wtest=[0:350];
Ftest=param(1)*(wtest.^2)+param(2)*wtest;
plot(wtest,Ftest)


legend('\fontsize{13}Medias experimentales' ,'\fontsize{13}Curva modelo cuadrático');
xlabel ('\fontsize{13}Velocidad angular (rad/s)')
ylabel('\fontsize{13}Torque (Nm)');

hold on
hold on


error=Fm-(param(1)*w.^2+param(2)*w);
error_promedio=mean(error)
sigma=std(error)


% %Asumiendo modelo cuadrático sin término lineal
% 
% B=[w.^2];
% 
% %param=((B'*B)^(-1))*B'*F;
% param=(B'*B)\(B'*Fm)
% 
% wtest=[0:350];
% Ftest=param(1)*(wtest.^2);
% plot(wtest,Ftest)
% legend('Medias experimentales','Curva modelo cuadrático','Curva modelo cúbico')
% 
% 
% hold on
% 
% 
% error=Fm-(param(1)*w.^2);
% error_promedio=mean(error)
% sigma=std(error)
% 
% %Asumiendo modelo cúbico
% B=[w.^3 w.^2 w ];
% 
% %param=((B'*B)^(-1))*B'*F;
% param=(B'*B)\(B'*Fm);
% 
% wtest=[0:350];
% Ftest=param(1)*(wtest.^3)+param(2)*(wtest.^2)+param(3)*wtest;
% plot(wtest,Ftest,'r')
% 
% error=Fm-(param(1)*w.^3+param(2)*w.^2+param(3)*w);
% error_promedio=mean(error)
% sigma=std(error)