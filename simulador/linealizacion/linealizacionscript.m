%--------------------------------------------------------------------------
%Linealiza el sistema en las distintas trayectorias posibles
% - Hovering
% - Recta
% - Círculos
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
T1=4.60160135072435e-05*w1^2-0.00103822726273726*w1;
T2=4.60160135072435e-05*w2^2-0.00103822726273726*w2;
T3=4.60160135072435e-05*w3^2-0.00103822726273726*w3;
T4=4.60160135072435e-05*w4^2-0.00103822726273726*w4;

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
	  -(Q1-Q2+Q3-Q4)/Izz];
   
dX=eval(dX);

%Linealizado para cualquier trayectoria
A=[diff(dX,x) diff(dX,y) diff(dX,z)  diff(dX,psis) diff(dX,phi) diff(dX,theta) diff(dX,vq1) diff(dX,vq2) diff(dX,vq3)  diff(dX,wq1) diff(dX,wq2) diff(dX,wq3)];
B=[diff(dX,w1) diff(dX,w2) diff(dX,w3) diff(dX,w4)]; 
  

%Para los círculos hay que cambiar las tres primeras ecuaciones

dXcirc(1:3,1)= [vq1+wq2*z-wq3*y;
           vq2+wq3*x-wq1*z;
           vq3+wq1*y-wq2*x];


A3primeras=[diff(dXcirc,x) diff(dXcirc,y) diff(dXcirc,z)  diff(dXcirc,psis) diff(dXcirc,phi) diff(dXcirc,theta) diff(dXcirc,vq1) diff(dXcirc,vq2) diff(dXcirc,vq3)  diff(dXcirc,wq1) diff(dXcirc,wq2) diff(dXcirc,wq3)];
Acirc=[A3primeras;A(4:12,1:12)];

save('./simulador/lazo_cerrado/linealizacion','A','B','Acirc'); 