function F = sistema(x)

L=0.29;
M=1.541;
Ixx=2.32e-02;
Iyy=2.32e-02;
Izz=4.37e-02;
Izzm=1.54e-05;
g=9.81;
   
%Fuerzas y torques de los motores
T1 = 3.5296e-5*x(9)^2-4.9293e-04*x(9);
T2 = 3.5296e-5*x(10)^2-4.9293e-04*x(10);
T3 = 3.5296e-5*x(11)^2-4.9293e-04*x(11);
T4 = 3.5296e-5*x(12)^2-4.9293e-04*x(12);

Q1 = 3.4734e-6*x(9)^2-1.3205e-4*x(9);
Q2 = 3.4734e-6*x(10)^2-1.3205e-4*x(10);
Q3 = 3.4734e-6*x(11)^2-1.3205e-4*x(11);
Q4 = 3.4734e-6*x(12)^2-1.3205e-4*x(12);

Vhor=evalin('base','Vhor');
thetap=evalin('base','thetap');
theta0=evalin('base','theta0');


F =[x(3)*cos(x(2))*cos(theta0)+x(4)*(cos(theta0)*sin(x(2))*sin(x(1))-cos(x(2))*sin(theta0))+x(5)*(sin(x(1))*sin(theta0)+cos(x(1))*cos(theta0)*sin(x(2)))+Vhor*sin(theta0);
      x(3)*cos(x(2))*sin(theta0)+x(4)*(cos(x(1))*cos(theta0)+sin(theta0)*sin(x(2))*sin(x(1)))+x(5)*(cos(x(1))*sin(theta0)*sin(x(2))-cos(theta0)*sin(x(1)))-Vhor*cos(theta0);
      -x(3)*sin(x(2))+x(4)*cos(x(2))*sin(x(1))+x(5)*cos(x(2))*cos(x(1));
    
    x(6)+x(8)*tan(x(2))*cos(x(1))+x(7)*tan(x(2))*sin(x(1));
    x(7)*cos(x(1))-x(8)*sin(x(1));
    x(8)*cos(x(1))/cos(x(2))+x(7)*sin(x(1))/cos(x(2))-thetap
    
    x(4)*x(8)-x(5)*x(7)+g*sin(x(2));
    x(5)*x(6)-x(3)*x(8)-g*cos(x(2))*sin(x(1));
    x(3)*x(7)-x(4)*x(6)-g*cos(x(2))*cos(x(1))+(T1+T2+T3+T4)/M;
     
    (x(7)*x(8)*(Iyy-Izz)+L*(T2-T4)+x(7)*Izzm*(x(9)-x(10)+x(11)-x(12)))/Ixx;
    (x(6)*x(8)*(Izz-Ixx)+L*(T3-T1)+x(6)*Izz*(x(9)-x(10)+x(11)-x(12)))/Iyy;
	Izzm*(Q1-Q2+Q3-Q4)/Izz];