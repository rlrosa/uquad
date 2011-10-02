function dX = dinamica_quad(t,X)
global T1 T2 T3 T4 w1 w2 w3 w4

%Constantes
g=9.81;
M=1;
L=1;
Ixx=1;
Iyy=1;
Izz=1;
Izzm=1;

%dw1=diff(w1,t);
%dw2=diff(w2,t);
%dw3=diff(w3,t);
%dw4=diff(w4,t);


%Defino el vector de estados
psis=X(1); 
phi=X(2);
theta=X(3);
x=X(4);
y=X(5);
z=X(6);
vq1=X(7);
vq2=X(8);
vq3=X(9);
wq1=X(10);
wq2=X(11);
wq3=X(12);


%Ecuaciones de cada derivada
dpsis = wq1 + (wq3*cos(psis)+wq2*sin(psis))*tan(phi);
dphi  = wq2*cos(psis)-wq3*sin(psis);
dtheta= (wq3*cos(psis)+wq2*sin(psis))/cos(phi);
dx    = vq1*cos(phi)*cos(theta)+vq2*(cos(psis)*cos(theta)+sin(theta)*sin(phi)*sin(psis))+vq3*(sin(psis)*sin(theta)+cos(psis)*cos(theta)*sin(phi));
dy    = vq1*cos(phi)*sin(theta)+vq2*(cos(psis)*cos(theta)+sin(theta)*sin(phi)*sin(psis))+vq3*(cos(psis)*sin(theta)*sin(phi)-cos(theta)*sin(psis));
dz    = -vq1*sin(phi)+vq2*cos(phi)*sin(psis)+vq3*cos(phi)*cos(psis);
dvq1  = vq2*wq3-vq3*wq2+g*sin(phi);
dvq2  = vq3*wq1-vq1*wq3 - g*cos(phi)*sin(psis);
dvq3  = vq1*wq2-vq2*wq1 - g*cos(phi)*cos(psis)+(T1+T2+T3+T4)/M;
dwq1  = (wq2*wq3*(Iyy-Izz)+L*(T2-T4) +wq2*Izzm*(w1-w2+w3-w4))/Ixx;
dwq2  = (wq1*wq3*(-Ixx+Izz)+L*(T3-T1)+wq1*Izzm*(w1-w2+w3-w4))/Iyy;
dwq3  = (wq1*wq2*(Ixx-Izz) );%-Izzm*(dw1-dw2+dw3-dw4))/Izz;



dX = [dpsis; dphi; dtheta; dx; dy; dz; dvq1; dvq2; dvq3; dwq1; dwq2; dwq3];

