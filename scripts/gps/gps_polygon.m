function r = gps_polygon(xy,d)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function r = gps_polygon(xy,d)
%
% Toma las rectas del poligono que se supone q es:
% 3                2                1
% x ---- ---- ---- x ---- ---- ---- x
% |                                 |
% |                                 |
% 
% |                                 |
% |                                 |
% x ---- ---- ---- x ---- ---- ---- x
% 6                5                4
%
% y las coordenadas xy = [x;y] y devuelve la diferencia para cada
% diagional, la diferencia:
%   || p(i) - p(j) ||^2 = d_{ij}^2
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


r = zeros(length(d),1);
points = 6;
x = xy(1:points);
y = xy(points+1:end);

ind = 1;
for i=1:points-1
  for j=i+1:points
    % || p(i) - p(j) ||^2 = d_{ij}^2
    r(ind) = ...
      (x(i) - x(j))^2 + (y(i) - y(j))^2 ...
      - d(ind)^2;
    ind = ind + 1;
  end
end
  
end