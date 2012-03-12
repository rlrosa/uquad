%Este programa se encarga de pasar los waypoints. Obtiene la ruta y 
%la sigue.


%Waypoints
waypoints= [0 0 0 0;
            8 0 0 50;
            25 20 30 100;
            8 3 2 150;
            3 4 20 200];
Nway=length(waypoints(:,1)); %Número de waypoints

k=1; %Contador que dice cuantos waypoints recorrí.
     %Por ahora suponemos que mi posición inicial coincide con el waypoint
     %inicial
     
r=0.2;  %Error aceptable
t_p=0.5;  %Tiempo de paso      
dis_min=1000*ones(Nway-1,1);  %Voy a usar esta variable para controlar si me estoy alejando
                            %o acercando a un waypoint 
v=generador_rutas(waypoints);

X0=[0 0 0 0 0 0 0 0 0 0 0 0];
assignin('base','x0',X0(1)); assignin('base','y0',X0(2));assignin('base','z0',X0(3));

assignin('base','psi0',X0(4));assignin('base','phi0',X0(5));assignin('base','theta0',X0(6));

assignin('base','vq10',X0(7));assignin('base','vq20',X0(8));assignin('base','vq30',X0(9));

assignin('base','wq10',X0(10));assignin('base','wq20',X0(11));assignin('base','wq30',X0(12));
ti=0;
t=ti;
X=X0;

k2=1; % Cuenta cuantas simulaciones de paso t_p se hacen en cada waypoint.


%Loop principal que se encarga de llamar a todas las simulaciones
while k<Nway 
    
    posicion_actual=[X0(1) X0(2) X0(3)];
    posicion_deseada=waypoints(k+1,1:3);
        
    %Distancia al waypoint
    distancia=norm(posicion_deseada-posicion_actual);
    
    %Actualizo distancia minima
    dis_min(k)=min(dis_min(k),distancia);
    
    %Me aseguro que sigo en el transitorio
    transitorio=(k2<5);
    
    if distancia > r && (distancia==dis_min(k) || transitorio)
        
       setpoint = [v(k,:) 0];                            
                             
       [taux,Xaux,Yaux]=sim_lazo_cerrado(ti,ti+t_p,setpoint','rec');
        
        %Las condiciones finales de este tramo son las condiciones
        %iniciales del tramo que sigue
        ti=ti+t_p;
        X0=Xaux(end,:);
        assignin('base','x0',X0(1)); assignin('base','y0',X0(2));assignin('base','z0',X0(3));

        assignin('base','psi0',X0(4));assignin('base','phi0',X0(5));assignin('base','theta0',X0(6));

        assignin('base','vq10',X0(7));assignin('base','vq20',X0(8));assignin('base','vq30',X0(9));

        assignin('base','wq10',X0(10));assignin('base','wq20',X0(11));assignin('base','wq30',X0(12));
        
        %Agrego al tiempo y al vector de estados este segmento de
        %simulación
        t=[t;taux(2:end)]; % No agrego la primer muestra para no tener repetidas
        X=[X;Xaux(2:end,1:12)]; %Idem
       
        k2=k2+1;
        
    %Si estoy cerca de la posición deseada me voy a la siguiente 
    %trayectoria
    else
        k=k+1; %Voy al siguiente waypoint            
        k2=0;  %Seteo en cero el contador que verifica el transitorio
    end
  
end

%% Grafico la trayectoria

plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),'r*')
hold on
plot3(X(:,1),X(:,2),X(:,3));
grid



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
while i<=length(X(:,1))/1e3
    
    c=X(40*(i-1)+1,4);
    b=X(40*(i-1)+1,5);
    a=X(40*(i-1)+1,6);
    
    
    R=eval(m_T_q);
    
    D=[X(1e3*(i-1)+1,1)*ones(1,6);X(1e3*(i-1)+1,2)*ones(1,6);X(1e3*(i-1)+1,3)*ones(1,6)];
    T=(R*[1 -1 0 0 0 0  ; 0 0 0 1 -1 0  ;0 0 0 0 0 0 ]);
    D=D+T;
    i=i+1;
   
    plot3(D(1,:),D(2,:),D(3,:));
    plot3(D(1,4),D(2,4),D(3,4),'ro')
    %axis([0 20 0 20 0 20]);
end

