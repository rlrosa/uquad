function gps_plot3(easting, northing, elevation, f_handle, fig_count)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function gps_plot3(easting, northing, elevation, f_handle, fig_count)
%
% Generates 3d plot from GPS data.
% 
% Inputs:
%   easting, northing: UTM coordinates (assumed in the same zone).
%   elevation: Elevation in m.
%   filename: Name of log file that provided the data.
%   f_handle (optional): Existing plot handle, hold it and plot.
%   fig_count (optional): Existing plot count, to use different colors per
%   plot.
%
% Example:
%
% % Plot 2 trajectories on the same figure.
% gps_plot3(x, y, z);
% gps_plot3(x_1, y_1, z_1, gcf, 1);
% 
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
path_colors = 'bgkyc';
x = easting;
y = northing;
z = elevation;

if(nargin < 5)
  fig_count = 0;
else
  col_count = size(path_colors,2);
  if(fig_count > col_count)
    error('Only %d figs supported',col_count);
  end
end
if(nargin < 4)
  f_handle = figure;
  axes1 = axes('Parent',f_handle);
  view(axes1,[0 90]);
  hold(axes1,'all');
else
  figure(f_handle);
  hold on;
end

path_color = path_colors(fig_count + 1);
plot3(x, y, z,sprintf('.-%c',path_color));
xlabel('Easting (m)')
ylabel('Northing (m)')
zlabel('Elevation (m)')
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
