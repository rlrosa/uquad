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
%% Grafico la trayectoria

syms a b c
Id=diag([1 1 1]);
%Rotación según el eje z
R_theta=[cos(a) sin(a) 0; -sin(a) cos(a) 0; 0 0 1];

%Rotación según el eje y
R_phi=[cos(b) 0 -sin(b) ;0 1 0; sin(b) 0 cos(b)];

%Rotación según el eje x
R_psis=[1 0 0; 0 cos(c) sin(c) ;0 -sin(c) cos(c)];
m_T_q =simple((Id/(R_theta))*(Id/(R_phi))*(Id/(R_psis)));


green = [154,205,50]/256;
blue  = [0,0,128]/256;
red   = [184,69,57]/256;

plot3(X(:,1),X(:,2),X(:,3));

i=1;
c=X(20*(i-1)+1,4); % yaw
b=X(20*(i-1)+1,5); % pitch
a=X(20*(i-1)+1,6); % roll
R=eval(m_T_q);
D=[X(20*(i-1)+1,1)*ones(1,6);X(20*(i-1)+1,2)*ones(1,6);X(20*(i-1)+1,3)*ones(1,6)];
T=(R*[1 0 0 0 0 0 ; 0 0 1 0 0 0 ;0 0 0 0 1 0]);
D=D+T;
i=i+1;
plot3(D(1,1:2),D(2,1:2),D(3,1:2),'color',blue);
hold on; grid
plot3(D(1,3:4),D(2,3:4),D(3,3:4),'color',red);
plot3(D(1,5:6),D(2,5:6),D(3,5:6),'color',green);
legend('x_q','y_q','z_q');
while i<=length(X(:,1))/20
    
    c=X(20*(i-1)+1,4); % yaw
    b=X(20*(i-1)+1,5); % pitch
    a=X(20*(i-1)+1,6); % roll    
    
    R=eval(m_T_q);
    
    D=[X(20*(i-1)+1,1)*ones(1,6);X(20*(i-1)+1,2)*ones(1,6);X(20*(i-1)+1,3)*ones(1,6)];
    T=(R*[1 0 0 0 0 0 ; 0 0 1 0 0 0 ;0 0 0 0 1 0]);
    D=D+T;
    i=i+1;
    
    plot3(D(1,1:2),D(2,1:2),D(3,1:2),'color',blue);
    plot3(D(1,3:4),D(2,3:4),D(3,3:4),'color',red);
    plot3(D(1,5:6),D(2,5:6),D(3,5:6),'color',green);

    %pause(20*1e-2)
    %hold on    
end
plot3(X(:,1),X(:,2),X(:,3),'color','black','linewidth',3);
%axis equal
hold off

% 
% plot3(X(:,1),X(:,2),X(:,3));grid on
% hold on;
% plot3(X(1:5:end,1),X(1:5:end,2),X(1:5:end,3),'x');grid on
% hold off;