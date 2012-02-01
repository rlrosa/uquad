function [x, y, err_rel, RESNORM,RESIDUAL] = gps_mc()
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% Este script toma las medidas de las diagonales del pol rectangulo del
% experimento del GPS, y obtiene la figura que + se parece al pol mediante
% minimos cuadrados.
% 
% x,y son las coordenadas en cm de los puntos resultantes.
% Se toma el punto 1 como el origen de coordenadas
%
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% diags en cm
diags = load('diags.txt');

% puntos en un caso ideal, en m
x0 = [0 6 12 0 6 12];
y0 = [0 0 0 6 6 6];
seed = [x0';y0'];

% convierto a cm
seed = seed*100;

[xy, RESNORM,RESIDUAL,EXITFLAG] = lsqnonlin(@(xy) ...
  gps_polygon(xy,diags),seed');

if(EXITFLAG <= 0)
  fprintf('lsqnonlin ta re quemado! EXITFLAG:%d\n',EXITFLAG);
  return;
end

points = 6;
x = xy(1:points)';
y = xy(points+1:end)';

% en cm
x_plot = [x(1:3);x(6:-1:4);x(1)];
y_plot = [y(1:3);y(6:-1:4);y(1)];

h = plot(x_plot, y_plot,'.-');
grid on
text(x_plot(1:end-1), y_plot(1:end-1), ...
    char('1','2','3','6','5','4'),'FontSize',16, ...
    'BackgroundColor',[.7 .9 .7],'FontWeight','bold')

err_rel = gps_err_rel(RESIDUAL);
