function v=generador_rutas_rec(waypoints)

vel_max=1.2; % m/s
t_min=5; %s

%Velocidades si no hubiera restricciones de aceleración
vini=zeros(waypoints(:,1),4);
for i=1:length(waypoints(:,1))
    vini(i+1,:)=(waypoints(i+1,1:3)-waypoints(i,1:3))./(waypoints(i+1,4)-waypoints(i,4));
    vini(i+1,4)=waypoints(4);
end

 %Me fijo si las velocidades cumplen con las restricciones de aceleración
 for i=1:length(vini(:,1))
    if vini(i,1)-vini(i-1,1) > vel_max && vini(i,1)+vel_max -vini(i+1,1) < 
        
        
    else
     
     
 end