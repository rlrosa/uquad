function [v,mod]=generador_rutas_rec(waypoints)


%Velocidades si no hubiera restricciones de aceleración
v=zeros(size(waypoints(:,1))-1,4);
mod = zeros(length(waypoints(:,1))-1);
for i=1:length(waypoints(:,1))-1
    if norm(waypoints(i+1,1:3)-waypoints(i,1:3)) > 1;
        v(i,1:3)=(waypoints(i+1,1:3)-waypoints(i,1:3))./(waypoints(i+1,4)-waypoints(i,4));
        v(i,4)=waypoints(i+1,4);
        mod(i) = 1;
    else
        mod(i) = 0;
        v(i,:) = waypoints(i+1,:);
        
    end
end

   
     