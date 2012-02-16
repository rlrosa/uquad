function sim_lazo_abierto 
global ti tf t x0 y0 z0 psi0 phi0 theta0 vq10 vq20 vq30 wq10 wq20 wq30 X0 w1 w2 w3 w4 

%%%%%%%%Constantes del sistema%%%%%%%%%%%%%%%%%%%%%%
assignin('base','g',9.81); %Constante gravitacional
assignin('base','L',1); %Largo de los brazos del quadcopter en metros
assignin('base','M',1); %Masa del quadcopter en kilogramos
%Elementos relevantes del tensor de inercia en kilogramos metros cuadrados
assignin('base','Ixx',1);
assignin('base','Iyy',1);
assignin('base','Izz',1);
assignin('base','Izzm',1);

x0=evalin('base','x0');
y0=evalin('base','y0');
z0=evalin('base','z0');
psi0=evalin('base','psi0');
phi0=evalin('base','phi0');
theta0=evalin('base','theta0');
vq10=evalin('base','vq10');
vq20=evalin('base','vq20');
vq30=evalin('base','vq30');
wq10=evalin('base','wq10');
wq20=evalin('base','wq20');
wq30=evalin('base','wq30');
ti=evalin('base','ti');
tf=evalin('base','tf');
t=evalin('base','t');
w1=evalin('base','w1');
w2=evalin('base','w2');
w3=evalin('base','w3');
w4=evalin('base','w4');

X0=[x0,y0,z0,psi0,phi0,theta0,vq10,vq20,vq30,wq10,wq20,wq30];
assignin('base','X0',X0);


%%Simulación del modelo Simulink
[t,X]=sim('dinamica_quad',[ti tf], [],[t',w1',w2',w3',w4']);

%Simulación por Ode45
%[t,X]=ode45(@f,[ti,tf],X0,[w1,w2,w3,w4]);


assignin('base','Variables',X);

%Ploteo la trayectoria del sistema
plot3(X(:,1),X(:,2),X(:,3));grid on
hold on;
plot3(X(1:5:end,1),X(1:5:end,2),X(1:5:end,3),'x');grid on
hold off;