function K=linealizacion_dis(modo,setpoint,lqrm)
%--------------------------------------------------------------------------
%Esta función recibe el modo de vuelo y el setpoint deseado y devuelve la
%matriz de realimentación lqr
%
%modo:  - hov (Hovering)
%       - rec (Recta)
%Por ahora hay dos opciones de matriz de realimentación para cada modo. Si
%le pasas lqrm=0 usas la primera, lqrm=1 usas la segunda.
%
%En la carpeta linealización hay un script con pruebas, pero esta es la
%posta. Para ahorrar cuentas ya arrancamos con la matriz linealizada.
%--------------------------------------------------------------------------


%% Cargo las matrices A y B linealizadas para una trayectoria genérica 

Matrices = load('linealizacion.mat');

A=Matrices.A;
B=Matrices.B;
     
Ts=evalin('base','Ts');
%% Linealización en Hovering
if modo=='hov'
    
    wq1=0; wq2=0; wq3=0;

    vq1=0; vq2=0; vq3=0;

    psis=0; phi=0;   

    w1 = 316.103650939028;
    w2 = 316.103650939028;
    w3 = 316.103650939028;
    w4 = 316.103650939028;

    theta=setpoint(4);
    
    Ah=eval(A);
    
    Bh=eval(B);
    %% Construcción de la matriz K método LQR para hovering
    %C = zeros(4,12);
%     C(1,1)=1;
%     C(2,2)=1;
%     C(3,3)=1;
%     C(4,6)=1;
%     Cz = zeros(4,4);
    C = eye(size(Ah));
    Z = zeros(12,12);
    
    BZ = zeros(12,4);
       
    Q=diag([1 1 1 1e3 1e3 1e3 1 1 1 1 1 1 1e-0 1e-0 1e-0 1e-0 1e-0 1e-0 1e-0 1e-0 1e-0 1e-0 1e-0 1e-0]);
    R = diag([1e-2 1e-2 1e-2 1e-2]);
    
    %[K,S,E]=lqrd([Ah Z; C Cz],[Bh;BZ],Q,R,Ts);
    K = uquad_dlqr([Ah Z; C Z],[Bh;BZ],Q,R);
    
%% Linealización vuelo en linea recta

elseif modo=='rec'
    
    
    vq1=setpoint(1); vq2=setpoint(2); vq3=setpoint(3);

    wq1=0; wq2=0; wq3=0;

    psis=0; phi=0; theta=setpoint(4);


    w1=334.279741754537;
    w2=334.279741754537;
    w3=334.279741754537;
    w4=334.279741754537;
    
    %No voy a realimentar la posición. Me interesa controlar la velocidad
    %solamente
    Ar=eval(A);
    Ar=Ar(4:12,4:12);
    
    Br=eval(B);
    Br=Br(4:12,:);
    
   
    %% Construcción de la matriz K método LQR para linea recta
    if lqrm==0
        %Qp=diag([1 1 1 100 100 100 1 1 1 1 1 1]);
        Qp=diag([100 100 100 1 1 1 1 1 1]); %Sin las posiciones
        Rp=diag([0.1 0.1 0.1 0.1]);
        [K,S,E]=lqrd(Ar,Br,Qp,Rp,Ts);  
    else
        Qp2=diag([1 1 1 100 100 100 1 1 1]);
        Rp2=diag([1 1 1 1]);
        [K,S,E]=lqr(Ar,Br,Qp2,Rp2);
    end
%% Linealización círculos

elseif modo=='cir'
    
    phi=setpoint(1); psis=setpoint(2);theta=0;
    vq1=setpoint(3); vq2=setpoint(4); vq3=setpoint(5);
    
    
    wq1=setpoint(6); wq2=setpoint(7); wq3=setpoint(8);
    
    w1=setpoint(9); w2=setpoint(10); w3=setpoint(11); w4=setpoint(12);
        
    %No voy a realimentar la posición. Me interesa controlar la velocidad
    %solamente. Tampoco me interesa theta
    Ac=A(4:12,4:12);
    Ac=eval(Ac);
    Ac(3,:)=[];
    Ac(:,3)=[];
    
    Bc=B(4:12,:);
    Bc=eval(Bc);
    Bc(3,:)=[];
    %% Construcción de la matriz K método LQR para circulos
    if lqrm==0
        Qp=diag([1 1 1e2 1e2 1e5 1 1 1e2]);
        
        Rp=diag([1 1 1 1]);
        [K,S,E]=lqr(Ac,Bc,Qp,Rp);  
    else
        Qp2=diag([1 1 10 10 10 1 1 1]);
        Rp2=diag([1 1 1 1]);
        [K,S,E]=lqr(Ac,Bc,Qp2,Rp2);
    end
    
end
 