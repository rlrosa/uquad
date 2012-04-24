function K=linealizacion_dis(modo,setpoint,w,ctrl)
%--------------------------------------------------------------------------
%Esta función recibe el modo de vuelo y el setpoint deseado y devuelve la
%matriz de realimentación lqr
%
%modo:  - hov (Hovering)
%       - rec (Recta)
%       - cir (Circulo) 
%--------------------------------------------------------------------------


%% Cargo las matrices A y B linealizadas para una trayectoria genérica 

Matrices = load('linealizacion.mat');

A=Matrices.A;
B=Matrices.B;
Ac = Matrices.Acirc;
     
Ts=evalin('base','Ts');

%Auxiliar matrix to add integrators
Id = zeros(4,12);
Id(1,1)=1;
Id(2,2)=1;
Id(3,3)=1;
Id(4,6)=1;

Z1 = zeros(12,4);
Z2 = zeros(4,4);
ZB = zeros(4,4);

w1 = w(1); w2 = w(2); w3 = w(3); w4 = w(4);
%% Linealización en Hovering
if modo=='hov'
    
    psis = setpoint(4); phi = setpoint(5); theta = setpoint(6);
    vq1 = setpoint(7); vq2 = setpoint(8); vq3 = setpoint(9);
    wq1 = setpoint(10); wq2 = setpoint(11); wq3 = setpoint(12);

    Ah=eval(A);
    Bh=eval(B);
    
    Aext = [Ah Z1;Id Z2];
    Bext = [Bh;ZB];
    
    %Construcción de la matriz K método LQR para hovering
      
    Q=diag([1 1 1 1e3 1e3 1e3 1 1 1 1 1 1 1 1 1 1]);
    R = diag(.1*[1 1 1 1]);
 
%% Linealización vuelo en linea recta
elseif modo=='rec'
    
    psis = setpoint(1); phi = setpoint(2); theta = setpoint(3);
    vq1 = setpoint(4); vq2 = setpoint(5); vq3 = setpoint(6);
    wq1 = setpoint(7); wq2 = setpoint(8); wq3 = setpoint(9);

    Ar=eval(A);
    Br=eval(B);
    
    Aext = [Ar Z1;Id Z2];
    Bext = [Br;ZB];
       
    % Construcción de la matriz K método LQR para linea recta
           
    Q=diag([1 1 1 1e3 1e3 1e3 1 1 1 1 1 1 1 1 1 1]);
    R = diag([1e-2 1e-2 1e-2 1e-2]);
              
%% Linealización círculos
elseif modo=='cir'
    
    x = setpoint(1); y = setpoint(2); z = setpoint(3);
    psis = setpoint(4); phi = setpoint(5);
    vq1 = setpoint(6); vq2 = setpoint(7); vq3 = setpoint(8);
    wq1 = setpoint(9); wq2 = setpoint(10); wq3 = setpoint(11);
        
    Ac=eval(Ac);
    Bc=eval(B);        
    
    Aext = [Ac Z1;Id Z2];
    Bext = [Bc;ZB];
    % Construcción de la matriz K método LQR para circulos
    Q = diag([1 1 1 1e3 1e3 1e3 1 1 1 1 1 1 1 1 1e-3 1]);
    R=diag([1e-2 1e-2 1e-2 1e-2]);
         
     
end

 switch ctrl
     case 2
        Aext(13:16,:)= [];
        Aext(:,13:16)= [];
        Bext(13:16,:) = [];
        Q(13:16,:) = [];
        Q(:,13:16) = [];
     case 3
        Aext(13:14,:) = [];
        Aext(:,13:14)= [];
        Aext(7:8,:) = [];
        Aext(:,7:8)= [];
        Aext(1:2,:) = [];
        Aext(:,1:2)= [];
        Bext(13:14,:) = [];
        Bext(7:8,:) = [];
        Bext(1:2,:) = [];
        Q(13:14,:) = [];
        Q(:,13:14) = [];
        Q(7:8,:) = [];
        Q(:,7:8) = [];
        Q(1:2,:) = [];
        Q(:,1:2) = [];
    case 4
        Aext(13:16,:) = [];
        Aext(:,13:16)= [];
        Aext(7:8,:) = [];
        Aext(:,7:8)= [];
        Aext(1:2,:) = [];
        Aext(:,1:2)= [];
        Bext(13:16,:) = [];
        Bext(7:8,:) = [];
        Bext(1:2,:) = [];
        Q(13:16,:) = [];
        Q(:,13:16) = [];
        Q(7:8,:) = [];
        Q(:,7:8) = [];
        Q(1:2,:) = [];
        Q(:,1:2) = [];
 end

K=uquad_dlqr(Aext,Bext,Q,R); 
 