function J=cost_ruta(x)
J=0;
waypoints=evalin('base','waypoints');

for i=1:length(waypoints(:,1))-2
   J=J+(x(i,1)+x(i,3)*cos(x(i,5)*x(i,6)+x(i,4))-waypoints(i+1,1))^2+...
   (x(i,2)+x(i,3)*sin(x(i,5)*x(i,6)+x(i,4))-waypoints(i+1,1))^2;
end

