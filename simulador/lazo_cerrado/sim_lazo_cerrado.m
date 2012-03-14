function [t,X,Y]=sim_lazo_cerrado(ti,tf,setpoint,modo)
%function [t,X,Y]=sim_lazo_cerrado(ti,tf,X0,setpoint,modo)
%--------------------------------------------------------------------------
%Esta es la propia así que la voy a comentar bien.
%   -ti y tf son escalares que corresponden al tiempo
%   -X0 es un vector horizontal de 12 variables que corresponde a las
%   condiciones iniciales
%	-modo es un string que corresponde al modo de vuelo:
%        *hov para hovering
%         *rec para recta   
%Setpoint de hovering: [Xs;Ys;Zs;THETAs]
%Setpoint de la linea recta: [Vqxs;Vqys;Vqzs;THETAs]
%--------------------------------------------------------------------------



%% Inicialización
assignin('base','g',9.81); %Constante gravitacional
assignin('base','L',0.29); %Largo de los brazos del quadcopter en metros
assignin('base','M',1.541); %Masa del quadcopter en kilogramos
%Elementos relevantes del tensor de inercia en kilogramos metros cuadrados
assignin('base','Ixx',2.32e-2);
assignin('base','Iyy',2.32e-2);
assignin('base','Izz',4.37e-2);
assignin('base','Izzm',1.54e-5);
assignin('base','Ts',5e-2)
%Tiempo de simulación.
t=linspace(ti,tf,(tf-ti)*10);

% %Condiciones iniciales
% 
% assignin('base','x0',X0(1));assignin('base','y0',X0(2)); 
% assignin('base','z0',X0(3));
% 
% assignin('base','psi0',X0(4));assignin('base','phi0',X0(5));
% assignin('base','theta0',X0(6));
% 
% assignin('base','vq10',X0(7));assignin('base','vq20',X0(8));
% assignin('base','vq30',X0(9));
% 
% assignin('base','wq10',X0(10));assignin('base','wq20',X0(11));
% assignin('base','wq30',X0(12));


%% Setpoint para hovering

if modo=='hov'
    
    %Setpoint x,y,z,theta;
    N= [1 0 0 0; %x 
        0 1 0 0; %y
        0 0 1 0; %z
        0 0 0 0; %psi   
        0 0 0 0; %phi
        0 0 0 1; %theta
        0 0 0 0; %vq1
        0 0 0 0; %vq2
        0 0 0 0; %vq3
        0 0 0 0; %wq1
        0 0 0 0; %wq2
        0 0 0 0];%wq3

    %Defino el setpoint de las velocidade angulares
    w = zeros(4,length(t));
    w(:,:) =334.279741754537;
    
    
    

%% Setpoint para linea recta

elseif modo=='rec';
         
    %setpoint x y z vq1, vq2,vq3,theta
    N =[1 0 0 0 0 0 0;  
        0 1 0 0 0 0 0;
        0 0 1 0 0 0 0; 
        0 0 0 0 0 0 0;
        0 0 0 0 0 0 0;
        0 0 0 0 0 0 1;
        0 0 0 1 0 0 0;
        0 0 0 0 1 0 0;
        0 0 0 0 0 1 0;
        0 0 0 0 0 0 0;
        0 0 0 0 0 0 0;
        0 0 0 0 0 0 0];
 
    %Defino el setpoint de las velocidade angulares
    w = zeros(4,length(t));
    w(:,:) =334.279741754537;

%% Setpoint para circulos

elseif modo=='cir';
         
    G=trim_circ(setpoint(1),setpoint(2))
   
    N =eye(12);
    %Defino el setpoint de las velocidades angulares
    w=[G(9);G(10);G(11);G(12)]*ones(1,length(t));
    setpoint = [ G(1) G(2) G(3) G(4) G(5) G(6) G(7) G(8)];
 
    
    
end
%% Simulo el sistema


assignin('base','N',N)
assignin('base','setpoint',setpoint)

if modo=='cir'
    K=linealizacion(modo,[setpoint w(:,1)'],0);
    assignin('base','K', K);
    
else
    K= linealizacion(modo,setpoint,0);
    assignin('base','K', K);


end
%modelo=['lazo_cerrado_' modo];
modelo=['lazo_cerrado_' modo '_dis'];
[t,X,Y]=sim(modelo,[ti tf],[],[t',w(1,:)',w(2,:)',w(3,:)',w(4,:)']);


%% Grafico la trayectoria
plot3(X(:,1),X(:,2),X(:,3));
grid
hold on


syms a b c
Id=diag([1 1 1]);
%Rotación según el eje z
R_theta=[cos(a) sin(a) 0; -sin(a) cos(a) 0; 0 0 1];

%Rotación según el eje y
R_phi=[cos(b) 0 -sin(b) ;0 1 0; sin(b) 0 cos(b)];

%Rotación según el eje x
R_psis=[1 0 0; 0 cos(c) sin(c) ;0 -sin(c) cos(c)];
m_T_q =simple((Id/(R_theta))*(Id/(R_phi))*(Id/(R_psis)));


i=1;
hold on;
while i<=length(X(:,1))/20
    
    c=X(20*(i-1)+1,4);
    b=X(20*(i-1)+1,5);
    a=X(20*(i-1)+1,6);
    
    
    R=eval(m_T_q);
    
    D=[X(20*(i-1)+1,1)*ones(1,6);X(20*(i-1)+1,2)*ones(1,6);X(20*(i-1)+1,3)*ones(1,6)];
    T=(R*[1 0 0 0 0 0 ; 0 0 1 0 0 0 ;0 0 0 0 1 0]);
    D=D+T;
    i=i+1;
   
    plot3(D(1,:),D(2,:),D(3,:));
  
  end
hold off
%axis equal