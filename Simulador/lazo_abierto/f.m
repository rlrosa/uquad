function dX = f(t,X)
g=evalin('base','g'); %Constante gravitacional
L=evalin('base','L'); %Largo de los brazos del quadcopter en metros
M=evalin('base','M'); %Masa del quadcopter en kilogramos
%Elementos relevantes del tensor de inercia en kilogramos metros cuadrados
Ixx=evalin('base','Ixx');
Iyy=evalin('base','Iyy');
Izz=evalin('base','Izz');
Izzm=evalin('base','Izzm');
x=X(1);
y=X(2);
z=X(3);
psi=X(4);
phi=X(5);
theta=X(6);
vq1=X(7);
vq2=X(8);
vq3=X(9);
wq1=X(10);
wq2=X(11);
wq3=X(12);

w1=evalin('base','w1');
w2=evalin('base','w2');
w3=evalin('base','w3');
w4=evalin('base','w4');
dw1=evalin('base','dw1');
dw2=evalin('base','dw2');
dw3=evalin('base','dw3');
dw4=evalin('base','dw4');

 T1=3.7646e-4*w1.^2-9.0535e-04*w1+0.0170*ones(size(w1),1);
   T2=3.7646e-4*w2.^2-9.0535e-04*w2+0.0170*ones(size(w1),1);
   T3=3.7646e-4*w3.^2-9.0535e-04*w3+0.0170*ones(size(w1),1);
   T4=3.7646e-4*w4.^2-9.0535e-04*w4+0.0170*ones(size(w1),1);
   
dX=[vq1*cos(phi)*cos(theta)+vq2*(cos(theta)*sin(phi)*sin(psi)-cos(phi)*sin(theta))+vq3*(sin(psi)*sin(theta)+cos(psi)*cos(theta)*sin(phi));
    vq1*cos(phi)*sin(theta)+vq2*(cos(psi)*cos(theta)+sin(theta)*sin(phi)*sin(psi)+vq3*(cos(psi)*sin(theta)*sin(phi)-cos(theta)*sin(psi)));
    -vq1*sin(phi)+vq2*cos(phi)*sin(psi)+vq3*(cos(phi)*cos(psi));
    wq1+wq3*tan(phi)*cos(psi)+wq2*tan(phi)*sin(psi);
    wq2*cos(psi)-wq3*sin(psi);
    wq3*cos(psi)/cos(phi)+wq2*sin(psi)/cos(phi);
    vq2*wq3-vq3*wq2+g*sin(phi);
    vq3*wq1-vq1*wq3-g*cos(phi)*sin(phi);
    vq1*wq2-vq2*wq1-g*cos(phi)*cos(psi)+(T1+T2+T3+T4)/M;
    (wq2*wq3*(Iyy-Izz)+L*(T2-T4)+wq2*Izzm*(w1-w2+w3-w4))/Ixx;
    (wq1*wq3*(Izz-Ixx)+L*(T3-T1)+wq1*Izzm*(w1-w2+w3-w4))/Iyy;
    wq1*wq2*(Ixx-Iyy)-Izzm*(dw1-dw2+dw3-dw4)];
   