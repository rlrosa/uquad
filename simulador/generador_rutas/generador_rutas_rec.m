function [v,mod]=generador_rutas_rec(waypoints)


v=zeros(size(waypoints(:,1))-1,5);
mod = zeros(length(waypoints(:,1))-1);
i =1;
while i<length(waypoints(:,1))
       
    %Verifico que el Yaw sea el mismo para dos waypoints consecutivos.
    %Si la posición es distinta voy con una recta
    if (norm(waypoints(i+1,1:3)-waypoints(i,1:3)) > 1)&&(norm(waypoints(i,4)-waypoints(i+1,4))<5*pi/180)
      
        aux = (waypoints(i+1,1:3)-waypoints(i,1:3))./(waypoints(i+1,5)-waypoints(i,5));
        v(i,:) = [aux(1)*cos(waypoints(i,4))-aux(2)*sin(waypoints(i,4))...
            aux(1)*sin(waypoints(i,4))+aux(2)*cos(waypoints(i,4))...
            aux(3)...
            waypoints(i+1,4)...
            waypoints(i+1,5)];
        
        mod(i) = 1;
        i = i+1;
    
    %Misma posición y distinto Yaw, movimiento circular  
    elseif (norm(waypoints(i+1,1:3)-waypoints(i,1:3)) < 1)&&(norm(waypoints(i,4)-waypoints(i+1,4))>5*pi/180) 
        
        v(i,1:3) = 0;
        v(i,4) = (waypoints(i+1,4)-waypoints(i,4))/(waypoints(i+1,5)-waypoints(i,5));
        v(i,5) = waypoints(i+1,5);
        mod(i) = 2;
        i = i+1;
        
    %Distinta posición y distinto Yaw, agrego un waypoint.    
    elseif (norm(waypoints(i+1,1:3)-waypoints(i,1:3)) > 1)&&(norm(waypoints(i,4)-waypoints(i+1,4))>5*pi/180)
        
        waypoints = [waypoints;zeros(1,5)];
        v = [v;zeros(1,5)];
        waypoints(i+2:end,:) = waypoints(i+1:end-1,:);
        %El nuevo waypoint tiene la misma posición que el anterior y el
        %Yaw del siguiente waypoint. Tiene tres segundos para hacer el giro
        waypoints(i+1,:) = [waypoints(i,1:3) waypoints(i+2,4) waypoints(i,5)+3];
        
    %Misma posición y mismo Yaw, hovering    
    else
        mod(i) = 0;
        v(i,:) = waypoints(i+1,:);
        i = i+1;
        
    end
    
end

   
     