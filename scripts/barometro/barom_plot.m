function barom_plot(alt, avg_size, indexes, temp)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
%function barom_plot(alt, avg_size, indexes)
%
% Plot altitud, raw and averaged.
% Multiple segments can be marked by providing indexing in INDEXES.
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
if(nargin < 3)
  indexes = -1;
end

figure;
% plot data
plot(alt(avg_size:end));
hold on; 
alt_moving_avg = moving_avg(alt,avg_size);
% plot avg
plot(alt_moving_avg(avg_size:end),'g--');
legend('Datos crudos',sprintf('Promedio en %d muestras',avg_size))

if(exist('temp','var'))
    plot(temp,'r')
    legend('Datos crudos', ...
      sprintf('Promedio. en %d muestras',avg_size), ...
      'Perfil de temperatura')
end

ylabel('Altura respecto al nivel del mar (m)')
xlabel('# de muestra')

if(indexes ~= -1)
  % plot segment borders
  max_val = max(alt);
  for i=1:length(indexes)
      line([indexes(i) indexes(i)],[min(alt) max(alt)],'color','black')
  end
%stem(indexes)
end

axis([1 length(alt) min(alt) max(alt)])
grid on
hold off
