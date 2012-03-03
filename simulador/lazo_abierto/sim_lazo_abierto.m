%function sim_lazo_abierto 
function sim_lazo_abierto(ti,tf,w1,w2,w3,w4)


% Constantes del sistema, hay que guardarlas como globales para que
% simulink las entienda
assignin('base','g',9.81); %Constante gravitacional
assignin('base','L',0.29); %Largo de los brazos del quadcopter en metros
assignin('base','M',1.541); %Masa del quadcopter en kilogramos
%Elementos relevantes del tensor de inercia en kilogramos metros cuadrados
assignin('base','Ixx',2.32e-2);
assignin('base','Iyy',2.32e-2);
assignin('base','Izz',4.37e-2);
assignin('base','Izzm',1.54e-5);

%La linea del tiempo no la pude pasar como parámetro así que la defino como
%global en la parte de la interfaz y acá me la traigo.
t=evalin('base','t');

%%Simulación del modelo Simulink
[t,X]=sim('dinamica_quad',[ti tf], [],[t',w1',w2',w3',w4']);

assignin('base','X',X);

%Ploteo la trayectoria del cuadricóptero
plot3(X(:,1),X(:,2),X(:,3));grid on
hold on;
plot3(X(1:5:end,1),X(1:5:end,2),X(1:5:end,3),'x');grid on
hold off;