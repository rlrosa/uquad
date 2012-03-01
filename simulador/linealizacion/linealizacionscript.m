function K=linealizacion(modo,setpoint)
%--------------------------------------------------------------------------
%Esta función recibe el modo de vuelo y el setpoint deseado y devuelve la
%matriz de realimentación lqr
%modo:  - hov (Hovering)
%       - rec (Recta)
%--------------------------------------------------------------------------



%% Definición del vector de estados y Linealización para cualquier trayectoria. 
syms dX x y z vq1 vq2 vq3 psi phi theta wq1 wq2 wq3 T1 T2 T3 T4 w1 w2 w3 w4 dw1 dw2 dw3 dw4 g M L Ixx Iyy Izz Izzm



%Valores de las constantes del sistema
L=0.29;
M=1.541;
Ixx=2.32e-05;
Iyy=2.32e-05;
Izz=4.37e-02;
Izzm=1.54e-05;
g=9.81;
   
%Fuerzas y torques de los motores
T1=3.5296e-5*w1^2-4.9293e-04*w1;
T2=3.5296e-5*w2^2-4.9293e-04*w2;
T3=3.5296e-5*w3^2-4.9293e-04*w3;
T4=3.5296e-5*w4^2-4.9293e-04*w4;

Q1=3.4734e-6*w1^2-1.3205e-4*w1;
Q2=3.4734e-6*w2^2-1.3205e-4*w2;
Q3=3.4734e-6*w3^2-1.3205e-4*w3;
Q4=3.4734e-6*w4^2-1.3205e-4*w4;

 
%Modelo del sistema
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
	  (wq1*wq2*(Ixx-Iyy)+Izzm*(Q1-Q2+Q3-Q4))/Izz];
   
   

   
dX=eval(dX);
%Linealizado para cualquier trayectoria
A=[diff(dX,x) diff(dX,y) diff(dX,z)  diff(dX,psi) diff(dX,phi) diff(dX,theta) diff(dX,vq1) diff(dX,vq2) diff(dX,vq3)  diff(dX,wq1) diff(dX,wq2) diff(dX,wq3)];
B=[diff(dX,w1) diff(dX,w2) diff(dX,w3) diff(dX,w4)];
     
   

%% Linealización en Hovering

% Condiciones
wq1=0;
wq2=0;
wq3=0;
   
vq1=0;
vq2=0;
vq3=0;

psi=0;
phi=0;   
   

%Fuerza
sym x;
fuerza_hov=9.81*M/4;
val=solve(3.5296e-5*x^2-4.9293e-04*x-fuerza_hov);
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
%% Construcción de la matriz K método LQR para hovering

%Con estos parámetros funca bien la posición. Los ángulos quedan medios
%changados. De acá salio la K_hov_theta0.mat. Regla de Pater 
Qp=diag([1 1 1 100 100 100 1 1 1 100 100 100]);
Rp=diag([1 1 1 1]);
[Kp,S,E]=lqr(Ah,Bh,Qp,Rp);

%Otra prueba. Esta por ahora es la que mejor anda. Igual hay algunos casos
%que no se porta bien
Qp2=diag([100 100 100 1 1 1 100 100 100 1 1 1]);
Rp2=diag([1 1 1 1]);
[Kp2,S,E]=lqr(Ah,Bh,Qp2,Rp2);

%% Linealización vuelo en linea recta

%Condiciones de vuelo en linea recta
% Por ejemplo me quiero mover según i a velocidad 1m/s
vq1=1;
vq2=1;
vq3=1;

wq1=0;
wq2=0;
wq3=0;

psi=0;
phi=0;
theta=pi/4;


%Fuerza
sym x;
fuerza_hov=9.81*M/4;
val=solve(3.5296e-5*x^2-4.9293e-04*x-fuerza_hov);
val_hov=eval(val);
     
if (val_hov(1)>=0)
         w1=val_hov(1);
else w1=val_hov(2);
end
w2=w1;
w3=w1;
w4=w1;

Ar=eval(A);
Ar=Ar(4:12,4:12);
Br=eval(B);
Br=Br(4:12,:);
   
%% Construcción de la matriz K método LQR para linea recta

Qp=diag([10 10 10 1 1 10 10 10 10]);
Rp=diag([0.1 0.1 0.1 0.1]);
[Kp,S,E]=lqr(Ar,Br,Qp,Rp);  

Qp2=diag([1 1 1 100 100 100 1 1 1]);
Rp2=diag([1 1 1 1]);
[Kp2,S,E]=lqr(Ar,Br,Qp2,Rp2);
   
 