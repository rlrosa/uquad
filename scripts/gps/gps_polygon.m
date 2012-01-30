function r = gps_polygon(xy,d)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function r = gps_polygon(xy,d)
%
% Toma las diagonales del poligono que se supone q es:
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
%   || p(i) - p(j) || = d_{ij}
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


r = zeros(length(d),1);
dec = 0;
points = 6;
x = xy(1:points);
y = xy(points+1:end);

for i=1:length(d)+2
  for j=i+1:points
    if((i==1 && j==3) || (i==4 && j==6))
      dec = dec + 1;
      continue;
    end
    % || p(i) - p(j) || = d_{ij}
    r(i+j-(2+dec)) = ...
      (x(i) - x(j))^2 + (y(i) - y(j))^2 ...
      - d(i+j-(2+dec))^2;
  end
end
  
end