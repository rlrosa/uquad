function K=linealizacion(modo,setpoint,lqrm)
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
Acirc=Matrices.Acirc;     
Ts=evalin('base','Ts');
%% Linealización en Hovering
if modo=='hov'
    
    wq1=0; wq2=0; wq3=0;

    vq1=0; vq2=0; vq3=0;

    psis=0; phi=0;   

    w1=334.279741754537;
    w2=334.279741754537;
    w3=334.279741754537;
    w4=334.279741754537;

    theta=setpoint(4);
    Ah=eval(A);
    Bh=eval(B);
    
    %% Construcción de la matriz K método LQR para hovering

    %Por ahora hay dos opciones para trabajar. No se todavía cual es mejor
    if lqrm==0
        Qp=diag([1 1 1e2 1 1 1 1 1 1e4 1 1 1]); 
        
        
        Rp=diag([1 1 1 1]);
        %[K,S,E]=lqr(Ah,Bh,Qp,Rp);
        K=uquad_dlqr(Ah,Bh,Qp,Rp);
    else
        Qp2=diag([100 100 100 1 1 1 100 100 100 1 1 1]);
        Rp2=diag([1 1 1 1]);
        [K,S,E]=lqr(Ah,Bh,Qp2,Rp2);
    end

%% Linealización vuelo en linea recta

elseif modo=='rec'
    
    
    vq1=setpoint(1); vq2=setpoint(2); vq3=setpoint(3);

    wq1=0; wq2=0; wq3=0;

    psis=0; phi=0; theta=setpoint(4);


    w1=334.279741754537;
    w2=334.279741754537;
    w3=334.279741754537;
    w4=334.279741754537;
    
    Ar=eval(A);
    %Ar=Ar(4:12,4:12);
    
    Br=eval(B);
    %Br=Br(4:12,:);
    
   
    %% Construcción de la matriz K método LQR para linea recta
    if lqrm==0
        %Qp=diag([1 1 1 1 1 1 1e4 1e4 1e4 1 1 1]);%No seas pancho no cambies esto
        Qp=diag([1 1 1 1 1 1 1 1 1 1 1 1]);
        Rp=diag([1 1 1 1]);
        %[K,S,E]=lqrd(Ar,Br,Qp,Rp,Ts);  
        [K,S,E]=lqr(Ar,Br,Qp,Rp);  
    else
        Qp=diag([1e2 1e2 1e2 1 1 1 1e2 1e2 1e2 1 1 1]);
        Rp2=diag([1 1 1 1]);
        [K,S,E]=lqr(Ar,Br,Qp2,Rp2);
    end
%% Linealización círculos

elseif modo=='cir'
    
    phi=setpoint(1); psis=setpoint(2);theta=0;
    vq1=setpoint(3); vq2=setpoint(4); vq3=setpoint(5);
    Velhor=evalin('base','Vhor');
    thetap=evalin('base','thetap');
    x=Velhor/thetap*cos(phi);
    y=Velhor/thetap*sin(phi)*sin(psis);
    z=Velhor/thetap*sin(phi)*cos(psis);
    wq1=setpoint(6); wq2=setpoint(7); wq3=setpoint(8);
    
    w1=setpoint(9); w2=setpoint(10); w3=setpoint(11); w4=setpoint(12);
    
    Ac=eval(Acirc);       
    Bc=eval(B);
    %% Construcción de la matriz K método LQR para circulos
    if lqrm==0
        Qp=diag([10 1 1 1 1e3 1e2 1e4 1e4 1e4 1 1 1e2]);
        %Qp=diag([1 1 1 1e2 1e2 1e2 1 1 1 1 1 1]);
        Rp=diag([1 1 1 1]);
        [K,S,E]=lqr(Ac,Bc,Qp,Rp);  
    else
        Qp2=diag([1 1 10 10 10 1 1 1]);
        Rp2=diag([1 1 1 1]);
        [K,S,E]=lqr(Ac,Bc,Qp2,Rp2);
    end
    
end
 