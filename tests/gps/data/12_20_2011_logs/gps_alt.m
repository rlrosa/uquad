%% Load times corresponding to 0m, 1m, 2m, 3m from log file
% each row of t corresponds to the times for one log file
t  =  {'2011-12-20T20:37:59.844Z' '2011-12-20T20:38:42.844Z' ...
       '2011-12-20T20:39:39.844Z' '2011-12-20T20:42:09.844Z' };

%% Load log files
log_count = 7;
for i=1:log_count
  switch i
    case 1
      log_files = '1.log';
    case 2
      log_files = '2.log';
    case 3
      log_files = '3.log';
    case 4
      log_files = '4.log';
    case 5
      log_files = '5.log';
    case 6
      log_files = '6.log';
    case 7
      log_files = '7.log';
    otherwise
      error('Invalid log count.')
  end      
  xs(i) = xmltools(log_files);
end

%% Extract & plot data
% set this to true to plot each alt on a separate plot
plot_individual_data = 0;

fig_all = figure;
subplot 211
grid on
title('Altitud estimation error')
plot_colors = 'bgkc';
xlabel('Time (s)')
ylabel('Relative Elevation (m)')
subplot 212
grid on
title('Satelites available')
xlabel('Time (s)')
ylabel('Satelite count')

for i = 1:4
  [easting, northing, elevation, utm_zone, sat] = ...
    gpxlogger_xml_handler(xs(3 + i),0);
  % Take data for point 1 as (x,y,z) = (0,0,0)
  easting = easting - easting(1);
  northing = northing - northing(1);
  elevation = elevation - elevation(1);
  
  % Plot data on separate plots for better resolution
  if(plot_individual_data)
    figure;
    plot(elevation,'.-')
    title(sprintf('Steady at %dm',i-1))
    xlabel('Time (s)')
    ylabel('Relative Elevation (m)')
    hold on
    hold off
    grid on
  end
  
  % Plot all data on one plot to estimate error Vs altitud
  figure(fig_all)
  subplot 211
  hold on
  plot(abs(elevation),sprintf('.-%c',plot_colors(i)))
  hold off
  % Plot sats available when data was taken
  subplot 212
  hold on
  plot(sat,sprintf('.-%c',plot_colors(i)))
  hold off
end

figure(fig_all)
subplot 211
legend('0m', '1m', '2m', '3m')
subplot 212
legend('0m', '1m', '2m', '3m')
