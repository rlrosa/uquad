% Antes de ejecutar este script:
%   - Cargar los logs en una estructura, usando gpxlogs_to_struct.m
%
%   - Configurar gps_plot3 para que solo imprima la grafica que tiene toda
%   la data junta (plot3d de easting, northing y elevation).

if(~exist('xs','var'))
  error('Cannot find data! xs not defined!\nRefer to comments in 10m_data.m for help')
end

% Asignar un valor > 0 a la siguiente variable para generar un plot con
% todos los datos (puede ser un quilombo)
% Un valor distinto de cero imprime en una grilla
todos_en_el_mismo_plot = 1;

radio = 2.5; % distancia en metros al valor medio

easting_mean = zeros(6,1);
northing_mean = zeros(6,1);
elevation_mean = zeros(6,1);
fuera_cantidad = zeros(6,1);
fuera_porcentage = zeros(6,1);

figure
for i =1:6
  [easting, northing, elevation, utm_zone, sat] = ...
    gpxlogger_xml_handler(xs(i),0);
  if(todos_en_el_mismo_plot)
    easting_mean(i) = mean(easting);
    northing_mean(i) = mean(northing);
    elevation_mean(i) = mean(elevation);
    
    fuera_cantidad(i) = sum(sqrt( ...
      (easting - easting_mean(i)).^2 + ...
      (northing - northing_mean(i)).^2 ...
      ) ...
      > radio);
    fuera_porcentage(i) = fuera_cantidad(i)/length(easting);
    
    gps_plot3(easting, northing, elevation, sat, gcf, i-1);
  else
    subplot(320 + i)
    gps_plot3(easting, northing, elevation, sat, gcf, 0);
  end
end

if(todos_en_el_mismo_plot)
  legend('Pt. 1', 'Pt. 2', 'Pt. 3', 'Pt. 4', 'Pt. 5', 'Pt. 6')
  title('10 minutos en cada punto')
  hold on
  circle([0,0],radio,50,'-.k')
  hold off
end

%% Arma una grilla con ejes en la misma escala
% Definir los siguiente valores, luego definir una variable 'listo_logs' en
% el workspace.
% Fijar los siguiente valores mirando el plot existente. El objetivo es
% hacer entrar todos lo puntos en un cierto rango (a determinar), y que
% todos los plots tengan la misma escala, asi es + facil ver los errores.
if(~todos_en_el_mismo_plot)
  xmin = -3;
  xmax = 6;
  ymin = -5;
  ymax = 6;
  if(~exist('listo_logs','var'))
    error('Fijar max/min de los ejes, luego definir una var listo_logs en el workspace (no importa el valor que se le asigne)')
  end
  for i =1:6
    subplot(320 + i)
    title(sprintf('Pt. #%d',i))
    axis([xmin xmax ymin ymax])
    legend hide
  end
end