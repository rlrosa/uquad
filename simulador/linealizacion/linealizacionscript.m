function K=linealizacion(modo,setpoint)
%--------------------------------------------------------------------------
%Esta función recibe el modo de vuelo y el setpoint deseado y devuelve la
%matriz de realimentación lqr
%modo:  - hov (Hovering)
%       - rec (Recta)
%--------------------------------------------------------------------------



%% Definición del vector de estados y Linealización para cualquier trayectoria. 
syms dX x y z vq1 vq2 vq3 psis phi theta wq1 wq2 wq3 T1 T2 T3 T4 w1 w2 w3 w4 dw1 dw2 dw3 dw4 g M L Ixx Iyy Izz Izzm



%Valores de las constantes del sistema
L=0.29;
M=1.541;
Ixx=2.32e-02;
Iyy=2.32e-02;
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
dX = [vq1*cos(phi)*cos(theta)+vq2*(cos(theta)*sin(phi)*sin(psis)-cos(phi)*sin(theta))+vq3*(sin(psis)*sin(theta)+cos(psis)*cos(theta)*sin(phi));
      vq1*cos(phi)*sin(theta)+vq2*(cos(psis)*cos(theta)+sin(theta)*sin(phi)*sin(psis))+vq3*(cos(psis)*sin(theta)*sin(phi)-cos(theta)*sin(psis));
      -vq1*sin(phi)+vq2*cos(phi)*sin(psis)+vq3*cos(phi)*cos(psis);
      wq1+wq3*tan(phi)*cos(psis)+wq2*tan(phi)*sin(psis);
      wq2*cos(psis)-wq3*sin(psis);
      wq3*cos(psis)/cos(phi)+wq2*sin(psis)/cos(phi);
      vq2*wq3-vq3*wq2+g*sin(phi);
      vq3*wq1-vq1*wq3-g*cos(phi)*sin(psis);
      vq1*wq2-vq2*wq1-g*cos(phi)*cos(psis)+(T1+T2+T3+T4)/M;
      (wq2*wq3*(Iyy-Izz)+L*(T2-T4)+wq2*Izzm*(w1-w2+w3-w4))/Ixx;
      (wq1*wq3*(Izz-Ixx)+L*(T3-T1)+wq1*Izz*(w1-w2+w3-w4))/Iyy;
	  (wq1*wq2*(Ixx-Iyy)+Izzm*(Q1-Q2+Q3-Q4))/Izz];
   
   

   
dX=eval(dX);
%Linealizado para cualquier trayectoria
A=[diff(dX,x) diff(dX,y) diff(dX,z)  diff(dX,psis) diff(dX,phi) diff(dX,theta) diff(dX,vq1) diff(dX,vq2) diff(dX,vq3)  diff(dX,wq1) diff(dX,wq2) diff(dX,wq3)];
B=[diff(dX,w1) diff(dX,w2) diff(dX,w3) diff(dX,w4)];
     
save('./../lazo_cerrado/linealizacion','A','B');   


 