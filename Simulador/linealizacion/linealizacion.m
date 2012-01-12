syms dX x y z vq1 vq2 vq3 psi phi theta wq1 wq2 wq3 T1 T2 T3 T4 w1 w2 w3 w4 dw1 dw2 dw3 dw4 g M L Ixx Iyy Izz Izzm


dX = [vq1*cos(phi)*cos(theta)+vq2*(cos(theta)*sin(phi)*sin(psi)-cos(phi)*sin(theta))+vq3*(sin(psi)*sin(theta)+cos(psi)*cos(theta)*sin(phi));
      vq1*cos(phi)*sin(theta)+vq2*(cos(psi)*cos(theta)+sin(theta)*sin(phi)*sin(psi))+vq3*(cos(psi)*sin(theta)*sin(phi)-cos(theta)*sin(psi));
      -vq1*sin(phi)+vq2*cos(phi)*sin(psi)+vq3*cos(phi)*cos(psi);
       vq2*wq3-vq3*wq2+g*sin(phi);
       vq3*wq1-vq1*wq3-g*cos(phi)*sin(psi);
       vq1*wq2-vq2*wq1-g*cos(phi)*cos(psi)+(T1+T2+T3+T4)/M;
       wq1+wq3*tan(phi)*cos(psi)+wq2*tan(phi)*sin(psi);
       wq2*cos(psi)-wq3*sin(psi);
       wq3*cos(psi)/cos(phi)+wq2*sin(psi)/cos(phi);
       (wq2*wq3*(Iyy-Izz)+L*(T2-T4)+wq2*Izzm*(w1-w2+w3-w4))/Ixx;
       (wq1*wq3*(Izz-Ixx)+L*(T3-T1)+wq1*Izz*(w1-w2+w3-w4))/Iyy;
	   (wq1*wq2*(Ixx-Iyy)-Izzm*(dw1-dw2+dw3-dw4))/Izz];
   
   
   %Linealizado para cualquier trayectoria
   A=[diff(dX,x) diff(dX,y) diff(dX,z) diff(dX,vq1) diff(dX,vq2) diff(dX,vq3) diff(dX,psi) diff(dX,phi) diff(dX,theta) diff(dX,wq1) diff(dX,wq2) diff(dX,wq3)];
   B=[diff(dX,T1) diff(dX,T2) diff(dX,T3) diff(dX,T4) diff(dX,w1) diff(dX,w2) diff(dX,w3) diff(dX,w4) diff(dX,dw1) diff(dX,dw2) diff(dX,dw3) diff(dX,dw4)];
     
   
   %Steady state aircraft flight
   dw1=0;
   dw2=0;
   dw3=0;
   dw4=0;
   
   dXss=eval(dX);
   
   dvq1=dX(4);
   dvq2=dX(5);
   dvq3=dX(6);
   dwq1=dX(10);
   dwq2=dX(11);
   dwq3=dX(12);
   
   %S1=solve(dvq1,dvq2,dvq3,dwq1,dwq2,dwq3); Esta es la otra condición pero
   %no la puedo imponer todavía porque tengo 16 incógnitas. Se que los wqi
   %y los vqi tienen que ser constantes
   
   %Condiciones de Hovering
   psi=0;
   phi=0;
   %theta=0; %O cualquier otro valor constante
   
   w2=w1;
   w3=w1;
   w4=w1;
   T2=T1;
   T3=T1;
   T4=T1;
   
   
   %Trayectoria hovering
   wq1=0;
   wq2=0;
   wq3=0;
   vq1=0;
   vq2=0;
   vq3=0;
   
    %%Valores de las cosas
   L=1;
   M=1;
   Ixx=1;
   Iyy=1;
   Izz=1;
   Izzm=1;
   g=9.81;
   Ah=eval(A);
   Ah=double('Ah');
   Bh=eval(B);
   Bh=double('Bh');
   
   
   
   Qdiag=[1 1 1 1 1 1 100 100 100 100 100 100 100];
   Q=diag(Qdiag);
   
   
   Rdiag=[1 1 1 1 10 10 10 10 10 10 10 10];
   R=diag(Rdiag);
   
   [K,S,E]=LQR(Ah,Bh,Q,R,1);