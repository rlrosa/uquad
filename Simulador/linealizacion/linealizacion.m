%%
%Definición del vector de estados y Linealización para cualquier
%trayectoria. 
syms dX x y z vq1 vq2 vq3 psi phi theta wq1 wq2 wq3 T1 T2 T3 T4 w1 w2 w3 w4 dw1 dw2 dw3 dw4 g M L Ixx Iyy Izz Izzm



%Valores de las constantes del sistema
L=1;
M=1;
Ixx=1;
Iyy=1;
Izz=1;
Izzm=1;
g=9.81;
   
%Fuerza de los motores
T1=3.7646e-5*w1^2-9.0535e-04*w1+0.0170;
T2=3.7646e-5*w2^2-9.0535e-04*w2+0.0170;
T3=3.7646e-5*w3^2-9.0535e-04*w3+0.0170;
T4=3.7646e-5*w4^2-9.0535e-04*w4+0.0170;

Q1=5.47e-7*w1^2+7.49e-5*w1-5.73e-4;
Q2=5.47e-7*w2^2+7.49e-5*w2-5.73e-4;
Q3=5.47e-7*w3^2+7.49e-5*w3-5.73e-4;
Q4=5.47e-7*w4^2+7.49e-5*w4-5.73e-4;

   
dX = [vq1*cos(phi)*cos(theta)+vq2*(cos(theta)*sin(phi)*sin(psi)-cos(phi)*sin(theta))+vq3*(sin(psi)*sin(theta)+cos(psi)*cos(theta)*sin(phi));
      vq1*cos(phi)*sin(theta)+vq2*(cos(psi)*cos(theta)+sin(theta)*sin(phi)*sin(psi))+vq3*(cos(psi)*sin(theta)*sin(phi)-cos(theta)*sin(psi));
      -vq1*sin(phi)+vq2*cos(phi)*sin(psi)+vq3*cos(phi)*cos(psi);
      wq1+wq3*tan(phi)*cos(psi)+wq2*tan(phi)*sin(psi);
      wq2*cos(psi)-wq3*sin(psi);
      wq3*cos(psi)/cos(phi)+wq2*sin(psi)/cos(phi);
      vq2*wq3-vq3*wq2+g*sin(phi);
      vq3*wq1-vq1*wq3-g*cos(phi)*sin(psi);
      vq1*wq2-vq2*wq1-g*cos(phi)*cos(psi)+(T1+T2+T3+T4)/M;
      (wq2*wq3*(Iyy-Izz)+L*(T2-T4)+wq2*Izzm*(w1-w2+w3-w4))/Ixx;
      (wq1*wq3*(Izz-Ixx)+L*(T3-T1)+wq1*Izz*(w1-w2+w3-w4))/Iyy;
	  (wq1*wq2*(Ixx-Iyy)-Izzm*(Q1-Q2+Q3-Q4))/Izz];
   
   

   
dX=eval(dX);
%Linealizado para cualquier trayectoria
A=[diff(dX,x) diff(dX,y) diff(dX,z)  diff(dX,psi) diff(dX,phi) diff(dX,theta) diff(dX,vq1) diff(dX,vq2) diff(dX,vq3)  diff(dX,wq1) diff(dX,wq2) diff(dX,wq3)];
B=[diff(dX,w1) diff(dX,w2) diff(dX,w3) diff(dX,w4)];
     
 



%Quiero que sea SLIT
%dw1=0;
%dw2=0;
%dw3=0;
%dw4=0;
   

   
%%
%Condiciones de Hovering
wq1=0;
wq2=0;
wq3=0;
   
vq1=0;
vq2=0;
vq3=0;

psi=0;
phi=0;   
   

 
sym x;
fuerza_hov=9.81*M/4;
val=solve(3.7646e-5*x^2-9.0535e-4*x+0.0170-fuerza_hov);
val_hov=eval(val);
     
if (val_hov(1)>=0)
         w1=val_hov(1);
else w1=val_hov(2);
end
   
w2=w1;
w3=w1;
w4=w1;
  
theta=0;
Ah=eval(A);
Bh=eval(B);
%%


%Con estos parámetros scás la K con la regla de bryson

Qb=diag([1/4 1/4 1/4 1/(0.0175)^2 1/(0.0175)^2 1/(0.0175)^2 1 1 1 1/(0.0175)^2 1/(0.0175)^2 1/(0.0175)^2]);
Rb=diag([1 1 1 1]);
[Kb,S,E]=lqr(Ah,Bh,Qb,Rb);

%Con estos parámetros funca bien la posición. Los ángulos quedan medios
%changados. De acá salio la K_hov_theta0.mat. Regla de Pater 
Qp=diag([1 1 1 100 100 100 1 1 1 100 100 100]);
Rp=diag([1 1 1 1]);
[Kp,S,E]=lqr(Ah,Bh,Qp,Rp);

%Otra prueba
Qp2=diag([100 100 100 1 1 1  100 100 100 1 1 0]);
Rp2=diag([1 1 1 1]);
[Kp2,S,E]=lqr(Ah,Bh,Qp2,Rp2);
%%
%Condiciones de vuelo en linea recta

wq1=0;
wq2=0;
wq3=0;

psi=0;
phi=0;

w2=w1;
w3=w1;
w4=w1;

Ar=eval(A);
Br=eval(B);
   
  
 
   
 