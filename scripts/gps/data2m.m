% Antes de ejecutar este script:
%   - Cargar los logs en una estructura, usando gpxlogs_to_struct.m
%
%   - Configurar gps_plot3 para que solo imprima la grafica que tiene toda
%   la data junta (plot3d de easting, northing y elevation).

if(~exist('xs','var'))
  error(sprintf('Cannot find data! xs not defined!\nRefer to comments in 2m_data.m for help'))
else
  fprintf('Using data from variable named xs\n\t-->VERIFY this is what you expect<--\n\nPress any key to continue...\n');
  pause;
end

% Asignar un valor > 0 a la siguiente variable para generar un plot con
% todos los datos (puede ser un quilombo)
% Un valor distinto de cero imprime en una grilla
todos_en_el_mismo_plot = 1;

easting_acum = [];
northing_acum = [];
elevation_acum = [];
sat_acum = [];

px = zeros(6,1);
py = zeros(6,1);
pz = zeros(6,1);

figure
for i =1:6
  [easting, northing, elevation, utm_zone, sat, lat, lon] = ...
    gpxlogger_xml_handler(xs(i),0);
  easting_acum = [easting_acum; easting];
  northing_acum = [northing_acum; northing];
  elevation_acum = [elevation_acum; elevation];
  sat_acum = [sat_acum; sat];

  % save data to log, in tab separated format:
  %   easting northing elevation sat lat lon
  data_to_save = [easting, northing, elevation, sat, lat, lon];
  log_name = sprintf('log0%d_data',i);
  fprintf('Will save current log data to %s...\n',log_name);
  save(sprintf('%s',log_name),'data_to_save')
  fprintf('Saved log data to %s!\n Format: (tab separated)\neasting northing elevation sat lat lon\n\n',log_name);
  
  % saco una cantidad ahi, un valor se que cae cerca del lugar donde se
  % tomaron los datos asi en los ejes la resolucion permite leer algo util.
  easting = easting - 576066.600750781;
  northing = northing - 6135583.22188047;
  elevation = elevation - 21.8;
  
  % armar un promedio de este punto
  px(i) = mean(easting);
  py(i) = mean(northing);
  pz(i) = mean(elevation);
  
  if(todos_en_el_mismo_plot)
    gps_plot3(easting, northing, elevation, sat, gcf, i-1, 0, 0);
  else
    subplot(320 + i)
    gps_plot3(easting, northing, elevation, sat, gcf, 0);
  end
end

if(todos_en_el_mismo_plot)
  %legend('Pt. 1', 'Pt. 2', 'Pt. 3', 'Pt. 4', 'Pt. 5', 'Pt. 6')
  legend hide
  if(length(easting) > 600)
    tmp = 10;
  else
    tmp = 2;
  end
  title(sprintf('%d minutos en cada punto',tmp))
  % armar el pol
  px_plot = [px(1:3);px(6:-1:4);px(1)];
  py_plot = [py(1:3);py(6:-1:4);py(1)];
  pz_plot = [pz(1:3);pz(6:-1:4);pz(1)];
  hold on
  plot3(px_plot, py_plot, pz_plot, 'r.-')
  text(px_plot(1:end-1), py_plot(1:end-1), pz_plot(1:end-1), ...
    char('1','2','3','6','5','4'),'FontSize',16, ...
    'BackgroundColor',[.7 .9 .7],'FontWeight','bold')
  % show values
  avg_data = [px, py, pz];
  fprintf('Will save data to avg_data...\n');
  save 'avg_data' -ascii -double -tabs avg_data
  fid = fopen('avg_data','a');
  fprintf(fid,'%%  easting\t\t\t  northing\t\t\t  elevation\n');
  fclose(fid);
  
  fprintf('Saved average data to avg_data! (easting,northing,elevation)\n');
  grid on
  axis equal
  hold off
end

%% Arma una grilla con ejes en la misma escala
% Definir los siguiente valores, luego definir una variable 'listo_logs' en
% el workspace.
% Fijar los siguiente valores mirando el plot existente. El objetivo es
% hacer entrar todos lo puntos en un cierto rango (a determinar), y que
% todos los plots tengan la misma escala, asi es + facil ver los errores.
if(~todos_en_el_mismo_plot)
  xmin = -2;
  xmax = 2;
  ymin = -2;
  ymax = 5;
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