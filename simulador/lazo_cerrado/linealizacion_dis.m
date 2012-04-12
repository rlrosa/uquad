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
Ac = Matrices.Acirc;
     
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


    w1 = 316.103650939028;
    w2 = 316.103650939028;
    w3 = 316.103650939028;
    w4 = 316.103650939028;
    
    Ar=eval(A);
    Br=eval(B);
    
    
   
    %% Construcción de la matriz K método LQR para linea recta
    
        
    C = eye(size(Ar));
    Z = zeros(12,12);
    
    BZ = zeros(12,4);
       
    Q=diag([1 1 1 1e3 1e3 1e3 1 1 1 1 1 1 1 1 1 1e-0 1e-0 1e-0 1e-0 1e-0 1e-0 1e-0 1e-0 1e-0]);
    R = diag([1e-2 1e-2 1e-2 1e-2]);
    
    %[K,S,E]=lqrd([Ah Z; C Cz],[Bh;BZ],Q,R,Ts);
    K = uquad_dlqr([Ar Z; C Z],[Br;BZ],Q,R);
   
%% Linealización círculos

elseif modo=='cir'
    
    phi=setpoint(1); psis=setpoint(2);theta=0;
    vq1=setpoint(3); vq2=setpoint(4); vq3=setpoint(5);
    
    
    wq1=setpoint(6); wq2=setpoint(7); wq3=setpoint(8);
    
    w1=setpoint(9); w2=setpoint(10); w3=setpoint(11); w4=setpoint(12);
    
    x =vq1/wq3*cos(phi);
    y = vq1/wq3*sin(phi)*sin(psis);
    z = vq1/wq3*sin(phi)*cos(psis);
    
    %Ac=A(4:12,4:12);
    Ac=eval(Ac);
    %Ac(3,:)=[];
    %Ac(:,3)=[];
    
    %Bc=B(4:12,:);
    Bc=eval(B);
    %Bc(3,:)=[];
    %% Construcción de la matriz K método LQR para circulos
    if lqrm==0
        %Qp=diag([1 1 1e2 1e2 1e5 1 1 1e2]);
        Qp = diag([1 1 1 1e2 1e2 1e2 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1 1]);
        Rp=diag([1e-2 1e-2 1e-2 1e-2]);
        K=uquad_dlqr([Ac zeros(12);eye(12) zeros(12)],[Bc; zeros(12,4)],Qp,Rp);  
    else
        Qp2=diag([1 1 10 10 10 1 1 1]);
        Rp2=diag([1 1 1 1]);
        [K,S,E]=lqr(Ac,Bc,Qp2,Rp2);
    end
    
end
 