%Este programa se encarga de pasar los waypoints. Obtiene la ruta y 
%la sigue.


%Waypoints
waypoints= [0 0 0 0;
            1 0 0 1;
            15 15 20 20];
Nway=length(waypoints(:,1)); %Número de waypoints

k=1; %Contador que dice cuantos waypoints recorrí.
     %Por ahora suponemos que mi posición inicial coincide con el waypoint
     %inicial
     
r=0.2;  % Distancia para control por hovering. 
        % Esto va a estar mientras no tenga círculos

e=0.05;  % Error que puedo cometer en la posición        
        
v=generador_rutas(waypoints);

X0=[0 0 0 0 0 0 0 0 0 0 0 0];
ti=0;
t=ti;
X=X0;
pase=false;

%Loop principal que se encarga de llamar a todas las simulaciones
while k<Nway && ti<30
    
    posicion_actual=[X0(1) X0(2) X0(3)];
    posicion_deseada=waypoints(k+1,1:3);
    
    
    %Si estoy lejos de la posición deseada sigo con la ruta
    distancia=norm(posicion_actual-posicion_deseada);

         
    if distancia >r && ~ pase
        setpoint = [v(k,:) 0];                            
                             
        [taux,Xaux]=sim_lazo_cerrado(ti,ti+0.1,X0,setpoint,'rec');
        
        %Las condiciones finales de este tramo son las condiciones
        %iniciales del tramo que sigue
        ti=ti+0.1;
        X0=Xaux(end,:);
        
        %Agrego al tiempo y al vector de estados este segmento de
        %simulación
        t=[t;taux];
        X=[X;Xaux];
    
        %Me fijo si pasé o no por el waypoint
        posicion_final=[Xaux(end,1) Xaux(end,2) Xaux(end,3)];
        pase=norm(posicion_final-posicion_deseada) < e;
    
% %     elseif (distancia < r) && ~pase
% %         setpoint = [waypoints(k+1,1:3) atan2(v(k+1,2),v(k+1,1))];
% %         
% %         
% %         [taux,Xaux]=sim_lazo_cerrado(ti,ti+0.1,X0,setpoint,'hov');
% %         
% %         %Las condiciones finales de este tramo son las condiciones
% %         %iniciales del tramo que sigue
% %         ti=ti+0.1;
% %         X0=Xaux(end,:);
% %         
% %         %Agrego al tiempo y al vector de estados este segmento de
% %         %simulación
% %         t=[t;taux];
% %         X=[X;Xaux];
% %             
% %         %Me fijo si pasé o no por el waypoint
% %         posicion_final=[Xaux(end,1) Xaux(end,2) Xaux(end,3)];
% %         pase=norm(posicion_final-posicion_deseada) < e;
        
        
        
        %Si estoy cerca de la posición deseada me voy a la siguiente 
        %trayectoria
    else
        k=k+1;
        
        X0=X(end,:);
        pase=false;
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
while i<=length(X(:,1))/20
    
    c=X(40*(i-1)+1,4);
    b=X(40*(i-1)+1,5);
    a=X(40*(i-1)+1,6);
    
    
    R=eval(m_T_q);
    
    D=[X(40*(i-1)+1,1)*ones(1,6);X(40*(i-1)+1,2)*ones(1,6);X(40*(i-1)+1,3)*ones(1,6)];
    T=(R*[1 0 0 0 0 0 ; 0 0 1 0 0 0 ;0 0 0 0 1 0]);
    D=D+T;
    i=i+1;
   
    plot3(D(1,:),D(2,:),D(3,:));
    axis([0 20 0 20 0 20]);
    hold on
    grid
end

