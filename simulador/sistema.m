function F = sistema(x)

L=evalin('base','L');
M=evalin('base','M');
Ixx=evalin('base','Ixx');
Iyy=evalin('base','Iyy');
Izz=evalin('base','Izz');
Izzm=evalin('base','Izzm');
g=evalin('base','g');
   
%Fuerzas y torques de los motores
T1=4.60160135072435e-05*x(4)^2-0.00103822726273726*x(4);
T2=4.60160135072435e-05*x(5)^2-0.00103822726273726*x(5);
T3=4.60160135072435e-05*x(6)^2-0.00103822726273726*x(6);
T4=4.60160135072435e-05*x(7)^2-0.00103822726273726*x(7);

Q1=3.4734e-6*x(4)^2-1.3205e-4*x(4);
Q2=3.4734e-6*x(5)^2-1.3205e-4*x(5);
Q3=3.4734e-6*x(6)^2-1.3205e-4*x(6);
Q4=3.4734e-6*x(7)^2-1.3205e-4*x(7);

Vhor=evalin('base','Vhor');
thetap=evalin('base','thetap');

F =[  
    x(2)*cos(x(1))-x(3)*sin(x(1));
    x(3)*cos(x(1))+x(2)*sin(x(1))-thetap
    
    -Vhor*x(3)-g*sin(x(1));
    Vhor*x(2)-g*cos(x(1))+(T1+T2+T3+T4)/M;
     
    (x(2)*x(3)*(Iyy-Izz)+L*(T2-T4)+x(2)*Izzm*(x(4)-x(5)+x(6)-x(7)))/Ixx;
    T3-T1
	-Izzm*(Q1-Q2+Q3-Q4)/Izz];