function sim_lazo_abierto 
global ti tf x0 y0 z0 psi0 phi0 theta0 vq10 vq20 vq30 wq10 wq20 wq30 X0 indice

%%%%%%%%Constantes del sistema%%%%%%%%%%%%%%%%%%%%%%
assignin('base','g',-9.81); %Constante gravitacional
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

X0=[x0,y0,z0,psi0,phi0,theta0,vq10,vq20,vq30,wq10,wq20,wq30];
assignin('base','X0',X0);
%%Defino el tiempo de simulación y la cantidad de puntos.

ti=evalin('base','ti');
tf=evalin('base','tf');
%t=(ti:0.2:tf);
t=linspace(ti,tf,(tf-ti)*5);


%%%%Entradas%%%%%
T1=zeros(size(t));
T2=zeros(size(t));
T3=zeros(size(t));
T4=zeros(size(t));

ind=evalin('base','indice');
if ind==1
elseif (ind==2)||(ind==3);

m=evalin('base','M');
psio=evalin('base','psi0');
phio=evalin('base','phi0');
T1(t>(ti-1))=9.81*m/(4*cos(phio)*cos(psio));
T2(t>-1)=9.81*m/(4*cos(phio)*cos(psio));
T3(t>-1)=9.81*m/(4*cos(phio)*cos(psio));
T4(t>-1)=9.81*m/(4*cos(phio)*cos(psio));

end
w1=zeros(size(t));
w2=zeros(size(t));
w3=zeros(size(t));
w4=zeros(size(t));

%w1(t>5)=1;
%w3(t>5)=1;


%%Simulación del modelo Simulink
[t,X]=sim('dinamica_quad',[0 10], [],[t',T1',T2',T3',T4',w1',w2',w3',w4']);

%Simulación por Ode45
%[t,X]=ode45(@f,[0,10],X0,[T1',T2',T3',T4',w1',w2',w3',w4']);


assignin('base','Variables',X);

%Ploteo la trayectoria del sistema
plot3(X(:,4),X(:,5),X(:,6));grid on