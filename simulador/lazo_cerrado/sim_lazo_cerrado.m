function [t,X,Y]=sim_lazo_cerrado(ti,tf,set,modo)
%function [t,X,Y]=sim_lazo_cerrado(ti,tf,X0,setpoint,modo)
%--------------------------------------------------------------------------
%Esta es la propia así que la voy a comentar bien.
%    -ti y tf son escalares que corresponden al tiempo
% 	 -modo es un string que corresponde al modo de vuelo:
%        *hov para hovering
%        *rec para recta 
%            
%Setpoint de hovering: [xs;ys;zs;thetas]
%Setpoint de la linea recta: [vqxs;vqys;vqzs;thetass]
%--------------------------------------------------------------------------



%% Inicialización
assignin('base','g',9.81); %Constante gravitacional
assignin('base','L',0.29); %Largo de los brazos del quadcopter en metros
assignin('base','d',0.07); %Distancia al centro de masa 
assignin('base','M',1.341); %Masa del quadcopter en kilogramos
%Elementos relevantes del tensor de inercia en kilogramos metros cuadrados
assignin('base','Ixx',2.32e-2);
assignin('base','Iyy',2.32e-2);
assignin('base','Izz',4.37e-2);
assignin('base','Izzm',1.54e-5);

%Tiempo de simulación.
t=linspace(ti,tf,(tf-ti)*10);

%% Setpoint para hovering
if modo=='hov'
        
    setpoint = [set(1) set(2) set(3) 0 0 set(4) 0 0 0 0 0 0];
    
    w = zeros(4,length(t));
    w(:,:) =calc_omega((evalin('base','M'))*evalin('base','g')/4);

%% Setpoint para linea recta
elseif modo=='rec';
         
    setpoint = [ 0 0 set(4) set(1) set(2) set(3) 0 0 0]; % The positions are set in the simulink model as the integral of the speeds
    
    w = zeros(4,length(t));
     w(:,:) =calc_omega(evalin('base','M')*evalin('base','g')/4);

%% Setpoint para circulos
elseif modo=='cir';
         
    %Returns psi, \dot{theta},angular speeds and angular speeds of the
    %rotors
    G=trim_circ(set(1),set(2))       
    
    w=[G(4);G(5);G(6);G(7)]*ones(1,length(t));
    setpoint = [ 0; -evalin('base','R')*cos(G(1)); evalin('base','R')*sin(G(1)); G(1); 0; set(1); 0; 0; G(2); 0; G(3)]; %theta is set in the simulink model as the integral of the angular speed 
    
end
%% Simulo el sistema

assignin('base','setpoint',setpoint)
ctrl = evalin('base','control');
K=linealizacion_dis(modo,setpoint, w(:,1)',ctrl);
switch evalin('base','control')
    case 2
        K = [K zeros(4,4)];
        
    case 3
        K = [zeros(4,2) K(:,1:4) zeros(4,2) K(:,5:8) K(:,9:12)];
    case 4
        K = [zeros(4,2) K(:,1:4) zeros(4,2) K(:,5:8) zeros(4,4)];
end
assignin('base','K', K);
    
modelo=['lazo_cerrado_' modo];
%modelo='onedof';
[t,X,Y]=sim(modelo,[ti tf],[],[t',w(1,:)',w(2,:)',w(3,:)',w(4,:)']);


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
% axis equal
hold off
