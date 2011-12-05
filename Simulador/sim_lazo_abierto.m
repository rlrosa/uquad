function sim_lazo_abierto %(Cond_iniciales)

%%%%%%%%Constantes del sistema%%%%%%%%%%%%%%%%%%%%%%
assignin('base','g',-9.81); %Constante gravitacional
assignin('base','L',1); %Largo de los brazos del quadcopter en metros
assignin('base','M',1); %Masa del quadcopter en kilogramos
%Elementos relevantes del tensor de inercia en kilogramos metros cuadrados
assignin('base','Ixx',1);
assignin('base','Iyy',1);
assignin('base','Izz',1);
assignin('base','Izzm',1);


%%Defino el tiempo de simulación y la cantidad de puntos.
t=linspace(0,10,50);


%%%%Entradas%%%%%
T1=zeros(size(t));
T2=zeros(size(t));
T3=zeros(size(t));
T4=zeros(size(t));

T1(t>-1)=(9.81/4)/cos(pi/3);
T2(t>-1)=(9.81/4)/cos(pi/3);
T3(t>-1)=(9.81/4)/cos(pi/3);
T4(t>-1)=(9.81/4)/cos(pi/3);


w1=zeros(size(t));
w2=zeros(size(t));
w3=zeros(size(t));
w4=zeros(size(t));


%%Simulación del modelo Simulink
[t,X]=sim('dinamica_quad',[0 10], [],[t',T1',T2',T3',T4',w1',w2',w3',w4']);

assignin('base','Variables',X);

%Ploteo la trayectoria del sistema
plot3(X(:,4),X(:,5),X(:,6));grid on