function gps_plot3(easting, northing, elevation, sat, ...
  f_handle, fig_count, smooth_win, rel_plot)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function gps_plot3(easting, northing, elevation, sat, ...
% f_handle, fig_count, do_smooth)
%
% Generates 3d plot from GPS data.
% 
% Inputs:
%   easting, northing: UTM coordinates (assumed in the same zone).
%   elevation: Elevation in m.
%   sat: Satelites used to take data.
%   filename: Name of log file that provided the data.
%   f_handle (optional): Existing plot handle, hold it and plot.
%   fig_count (optional): Existing plot count, to use different colors per
%     plot.
%   smooth_win (optional): Size of the window to use to calculate an
%     average (moving average). If set to 0, no average will be performed.
%   rel_plot: Plot relative data. Absolute data is cut by MatLab when
%     plotting. To be able to read values in the xaxis/yaxis, relative data
%     must be plotted.
%
% Example:
%
% % Plot 2 trajectories on the same figure.
% close all
% gps_plot3(x, y, z, sat);
% gps_plot3(x_1, y_1, z_1, sat_1, figure(1), 1);
% 
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
path_colors = 'bgkycr';
x = easting;
y = northing;
z = elevation;

if(nargin < 8)
  rel_plot = 1;
end
if(nargin < 7)
  smooth_win = 0;
end
if((nargin < 6) || (fig_count == 0))
  fig_count = 0;
else
  col_count = size(path_colors,2);
  if(fig_count > col_count)
    error('Only %d figs supported',col_count);
  end
end
if(nargin < 5)
  f_handle = figure;
  axes1 = axes('Parent',f_handle);
  view(axes1,[0 90]);
  hold(axes1,'all');
else
  figure(f_handle);
  hold on;
end

% use relative plot to improve axis resolution
if(rel_plot)
  x = x - mean(x);
  y = y - mean(y);
  z = z - mean(z);
  title_string = 'relativo';
else
  title_string = '';
end

% calculate smoothed trayectory
if(smooth_win)
  x_avg = moving_avg(x, smooth_win);
  y_avg = moving_avg(y, smooth_win);
  z_avg = moving_avg(z, smooth_win);
end

path_color = path_colors(fig_count + 1);
plot3(x, y, z,sprintf('.-%c',path_color));
xlabel(sprintf('Easting %s (m)', title_string))
ylabel(sprintf('Northing %s (m)', title_string))
zlabel(sprintf('Elevation %s (m)', title_string))
grid on;
title('3D Trajectory in UTM');

% Display initial position.
hold on;
display_initial_pos = 0;
if(display_initial_pos)
  plot3(x(1),y(1),z(1), 'ro','HandleVisibility','off')
  plot3(x(1),y(1),z(1), 'ro', 'Markersize', 20,'HandleVisibility','off')
  plot3(x(1),y(1),z(1), 'ro', 'Markersize', 10,'HandleVisibility','off')
  plot3(x(1),y(1),z(1), 'rx', 'Markersize', 20,'HandleVisibility','off')
end

% Plot smoothed trayectory
if(smooth_win)
  plot3(x_avg, y_avg, z_avg,sprintf('x-%c',path_color),'LineWidth',2);
end

hold off
legend('Trajectory')

% Plot individually, to improve resolution
plot_individual_data = 0;
if plot_individual_data
  if(nargin < 5)
    f_handle = figure;
  else
    figure(f_handle + 1)
  end
  subplot 131
  title(sprintf('Coordenadas individuales (%s)', title_string))
  hold on
  plot(x,sprintf('.-%c',path_color));
  if(smooth_win)plot(x_avg,sprintf('x-%c',path_color), 'LineWidth', 2);end
  xlabel('Tiempo (s)')
  ylabel(sprintf('Easting %s (m)', title_string))
  hold off
  grid on
  subplot 132
  hold on
  plot(y,sprintf('.-%c',path_color));
  if(smooth_win)plot(y_avg,sprintf('x-%c',path_color), 'LineWidth', 2);end
  xlabel('Tiempo (s)')
  ylabel(sprintf('Northing %s (m)', title_string))
  hold off
  grid on
  subplot 133
  hold on
  plot(z,sprintf('.-%c',path_color));
  if(smooth_win)plot(z_avg,sprintf('x-%c',path_color), 'LineWidth', 2);end
  xlabel('Tiempo (s)')
  if(rel_plot)
    ylabel('Altura relativa (m)')
  else
    ylabel('Altura (m)')
  end
  hold off
  grid on

  % Plot individually, to improve resolution
  if(nargin < 5)
    f_handle = figure;
  else
    % inc one figure
    figure(f_handle + 2)
  end
  hold on
  plot(sat,sprintf('.-%c',path_color));
  title('Satelites disponibles')
  xlabel('Tiempo (s)')
  ylabel('# de Satelites')
  hold off
  grid on
end
