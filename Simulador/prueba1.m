%%%%%%%%Constantes del sistema%%%%%%%%%%%%%%%%%%%%%%
g=-9.81; %Constante gravitacional
L=1; %Largo de los brazos del quadcopter en metros
M=1; %Masa del quadcopter en kilogramos
%Elementos relevantes del tensor de inercia en kilogramos metros cuadrados
Ixx=1;
Iyy=1;
Izz=1;
Izzm=1;

%%Defino el tiempo de simulación y la cantidad de puntos.
t=linspace(0,10,50);

%%%%%%%
theta0=0;
psi0=pi/4;
phi0=0;
x0=0;
y0=0;
z0=0;
vq10=0;
vq20=0;
vq30=0;
wq10=0;
wq20=0;
wq30=0;

%%%%Entradas%%%%%
T1=zeros(size(t));
T2=zeros(size(t));
T3=zeros(size(t));
T4=zeros(size(t));

T1(t>-1)=(9.81/4)/cos(pi/4);
T2(t>-1)=(9.81/4)/cos(pi/4);
T3(t>-1)=(9.81/4)/cos(pi/4);
T4(t>-1)=(9.81/4)/cos(pi/4);


w1=zeros(size(t));
w2=zeros(size(t));
w3=zeros(size(t));
w4=zeros(size(t));


%%Simulación del modelo Simulink
[t,X]=sim('dinamica_quad',[0 10], [],[t',T1',T2',T3',T4',w1',w2',w3',w4']);



%Ploteo la trayectoria del sistema
plot3(X(:,4),X(:,5),X(:,6))