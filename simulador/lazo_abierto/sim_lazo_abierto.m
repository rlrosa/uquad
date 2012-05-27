%function sim_lazo_abierto 
function [t,X]=sim_lazo_abierto(ti,tf,w1,w2,w3,w4)


% Constantes del sistema, hay que guardarlas como globales para que
% simulink las entienda
assignin('base','g',9.81); %Constante gravitacional
assignin('base','L',0.29); %Largo de los brazos del quadcopter en metros
assignin('base','d',0.07); %Distancia al centro de masa
assignin('base','M',1.741); %Masa del quadcopter en kilogramos
%Elementos relevantes del tensor de inercia en kilogramos metros cuadrados
assignin('base','Ixx',2.32e-2);
assignin('base','Iyy',2.32e-2);
assignin('base','Izz',4.37e-2);
assignin('base','Izzm',1.54e-5);

t=linspace(ti,tf,(tf-ti)*5);

%%Simulación del modelo Simulink
[t,X]=sim('dinamica_quad',[ti tf], [],[t',w1',w2',w3',w4']);

assignin('base','X',X);
assignin('base','t',t);

%Ploteo la trayectoria del cuadricóptero
plot3(X(:,1),X(:,2),X(:,3));grid on
hold on;
plot3(X(1:5:end,1),X(1:5:end,2),X(1:5:end,3),'x');grid on
hold off;