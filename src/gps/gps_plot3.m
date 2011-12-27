function gps_plot3(easting, northing, elevation, sat, f_handle, fig_count)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function gps_plot3(easting, northing, elevation, sat, f_handle, fig_count)
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
%   plot.
%
% Example:
%
% % Plot 2 trajectories on the same figure.
% close all
% gps_plot3(x, y, z, sat);
% gps_plot3(x_1, y_1, z_1, sat_1, figure(1), 1);
% 
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
path_colors = 'bgkyc';
x = easting;
y = northing;
z = elevation;

if(nargin < 6)
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
x = x - mean(x);
y = y - mean(y);
z = z - mean(z);

path_color = path_colors(fig_count + 1);
plot3(x, y, z,sprintf('.-%c',path_color));
xlabel('Relative Easting (m)')
ylabel('Relative Northing (m)')
zlabel('Relative Elevation (m)')
grid on;
title('3D Trajectory in UTM');

% Display initial position.
hold on;
plot3(x(1),y(1),z(1), 'ro')
plot3(x(1),y(1),z(1), 'ro', 'Markersize', 20)
plot3(x(1),y(1),z(1), 'ro', 'Markersize', 10)
plot3(x(1),y(1),z(1), 'rx', 'Markersize', 20)
hold off
legend('Trajectory', 'Initial pos')

% Plot individually, to improve resolution
if(nargin < 5)
  f_handle = figure;
else
  figure(f_handle + 1)
end
subplot 131
title('Individual relative measurements')
hold on
plot(x,sprintf('.-%c',path_color));
xlabel('Time (s)')
ylabel('Relative Easting (m)')
hold off
grid on
subplot 132
hold on
plot(y,sprintf('.-%c',path_color));
xlabel('Time (s)')
ylabel('Relative Northing (m)')
hold off
grid on
subplot 133
hold on
plot(z,sprintf('.-%c',path_color));
xlabel('Time (s)')
ylabel('Relative Elevation (m)')
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
title('Satelites available')
xlabel('Time (s)')
ylabel('Satelite count')
hold off
grid on

