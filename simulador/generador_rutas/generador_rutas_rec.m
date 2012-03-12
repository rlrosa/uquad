function v=generador_rutas_rec(waypoints)
v=zeros(waypoints(:,1)-1,3);
for i=1:length(waypoints(:,1))-1
   v(i,:)=(waypoints(i+1,1:3)-waypoints(i,1:3))./(waypoints(i+1,4)-waypoints(i,4));
end

