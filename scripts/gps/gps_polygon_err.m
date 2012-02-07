% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% Analiza los logs correpsondientes a las medidas de la posicion de los
% vertices del poligono.
% Se estima un error en la medida de las rectas que unen los vertices.
%
% NOTA: Se deben cargar en memoria los logs antes de ejecutar este script,
% en variables llamadas 'xs_10' 'xs_2mor1' 'xs_2mor2' 'xs_2' 'xs_3'.
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

varname = {'xs_10' 'xs_2mor1' 'xs_2mor2' 'xs_2' 'xs_3'};
for i=1:length(varname)
  if(~exist(char(varname(i)),'var'))
    error(sprintf('I cannot find %s',char(varname(i))))
  end
end

diags = load('diags.txt'); % medidas de las rectas usando un metro

%% Analizar logs
% aca se van a guardas las diagonales entre los puntos experimentas para
% los distintos logs
diag_err_600 = [];
diag_err_848 = [];
diag_err_1200 = [];
diag_err_1341 = [];

for j = 1:length(varname) % loop en juegos de datos

  easting_acum = [];
  northing_acum = [];
  elevation_acum = [];
  sat_acum = [];

  px = zeros(6,1);
  py = zeros(6,1);
  pz = zeros(6,1);

  easting_resto = 0;
  northing_resto = 0;
  elevation_resto = 0;

  xs = eval(char(varname(j)));
  
  for i =1:6 % loop en vertices dentro del j-esimo juego de datos
    [easting, northing, elevation, utm_zone, sat, lat, lon] = ...
      gpxlogger_xml_handler(xs(i),0);
    easting_acum = [easting_acum; easting];
    northing_acum = [northing_acum; northing];
    elevation_acum = [elevation_acum; elevation];
    sat_acum = [sat_acum; sat];

    repetidos(easting, northing, elevation);

    % saco una cantidad ahi, un valor se que cae cerca del lugar donde se
    % tomaron los datos asi en los ejes la resolucion permite leer algo util.
    if(i==1)
      easting_resto = mean(easting);
      northing_resto = mean(northing);
      elevation_resto = mean(elevation);
    end

    easting = easting - easting_resto;
    northing = northing - northing_resto;
    elevation = elevation - elevation_resto;

    % armar un promedio de este punto
    px(i) = mean(easting);
    py(i) = mean(northing);
    pz(i) = mean(elevation);
  
  end % i

  avg_data = [px, py, pz];
  avg_x = avg_data(:,1)-avg_data(1,1);
  avg_y = avg_data(:,2)-avg_data(1,2);
  fxy = gps_polygon([avg_x;avg_y]*100,diags);
  [diag_err, err_rel] = gps_err_rel(fxy); 

  % Analizo por separado, dependiendo de largo de la recta involucrada
  % Hay 600, 848, 1200 y 1341
  % 600
  tmp = diag_err([1 3 6 8 12 13 15]);
  diag_err_600 = [diag_err_600;tmp];
  % 848
  tmp = diag_err([4 7 9 11]);
  diag_err_848 = [diag_err_848;tmp];
  % 1200
  tmp = diag_err([2 14]);
  diag_err_1200 = [diag_err_1200;tmp];
  % 1341
  tmp = diag_err([5 10]);
  diag_err_1341 = [diag_err_1341;tmp];
  
end % j

%% Mostrar resultados
% 600
mmmm = mean(diag_err_600);
ssss = std(diag_err_600,1);
fprintf('600:\n\tMean:\t%0.2f\tstd:\t%0.2f\n',mmmm,ssss)
tmp = mmmm+2*ssss;
fprintf('\t95%% de las muestras tendran un error menor a %0.2f\t->%0.2f%%\n', ...
  tmp,tmp/600*100);
% 848
mmmm = mean(diag_err_848);
ssss = std(diag_err_848,1);
fprintf('848:\n\tMean:\t%0.2f\tstd:\t%0.2f\n',mmmm,ssss)
tmp = mmmm+2*ssss;
fprintf('\t95%% de las muestras tendran un error menor a %0.2f\t->%0.2f%%\n', ...
  tmp,tmp/848*100);
% 1200
mmmm = mean(diag_err_1200);
ssss = std(diag_err_1200,1);
fprintf('1200:\n\tMean:\t%0.2f\tstd:\t%0.2f\n',mmmm,ssss)
tmp = mmmm+2*ssss;
fprintf('\t95%% de las muestras tendran un error menor a %0.2f\t->%0.2f%%\n', ...
  tmp,tmp/1200*100);
% 1341
mmmm = mean(diag_err_1341);
ssss = std(diag_err_1341,1);
fprintf('1341:\n\tMean:\t%0.2f\tstd:\t%0.2f\n',mmmm,ssss)
tmp = mmmm+2*ssss;
fprintf('\t95%% de las muestras tendran un error menor a %0.2f\t->%0.2f%%\n', ...
  tmp,tmp/1341*100);
