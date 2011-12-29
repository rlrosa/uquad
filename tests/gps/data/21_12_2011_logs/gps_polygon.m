%% General stuff
polygon_points = 6;

%% Load times corresponding to points 1,2,3,4,5,6 in each log
% each row of t corresponds to the times for one log file
t  =  {'2011-12-21T18:31:27.524Z' '2011-12-21T18:34:44.524Z'   ...
       '2011-12-21T18:36:23.524Z' '2011-12-21T18:38:26.524Z'   ...
       '2011-12-21T18:39:29.524Z' '2011-12-21T18:40:25.525Z' ; ...
       '2011-12-21T18:45:11.525Z' '2011-12-21T18:45:50.525Z'   ...
       '2011-12-21T18:46:32.525Z' '2011-12-21T18:48:55.525Z'   ...
       '2011-12-21T18:49:31.525Z' '2011-12-21T18:50:17.525Z' ; ...
       '2011-12-21T18:55:31.526Z' '2011-12-21T18:56:12.526Z'   ...
       '2011-12-21T18:57:50.526Z' '2011-12-21T18:58:39.526Z'   ...
       '2011-12-21T18:59:43.526Z' '2011-12-21T19:01:12.526Z' ; ...
       '2011-12-21T19:05:33.526Z' '2011-12-21T19:06:14.526Z'   ...
       '2011-12-21T19:07:10.526Z' '2011-12-21T19:07:57.526Z'   ...
       '2011-12-21T19:08:46.526Z' '2011-12-21T19:09:30.526Z' ; ...
       '2011-12-21T19:27:45.528Z' '2011-12-21T19:28:20.528Z'   ...
       '2011-12-21T19:30:03.528Z' '2011-12-21T19:30:46.528Z'   ...
       '2011-12-21T19:31:34.528Z' '2011-12-21T19:32:13.528Z' ; ...
       '2011-12-21T19:39:08.528Z' '2011-12-21T19:39:38.528Z'   ...
       '2011-12-21T19:39:59.528Z' '2011-12-21T19:40:47.528Z'   ...
       '2011-12-21T19:41:24.528Z' '2011-12-21T19:42:37.528Z'};

%% Load log files
log_files = { '01.log', '02.log' '03.log' '04.log' '11.log' '12.log' };
log_count = size(log_files,2);
for i=1:log_count
  switch i
    case 1
      log_files = '01.log';
    case 2
      log_files = '02.log';
    case 3
      log_files = '03.log';
    case 4
      log_files = '04.log';
    case 5
      log_files = '11.log';
    case 6
      log_files = '12.log';
  end      
  xs(i) = xmltools(log_files);
end

%% Extract UTM from xml
easting = zeros(polygon_points,log_count);
northing = zeros(polygon_points,log_count);
elevation = zeros(polygon_points,log_count);
utm_zone = zeros(polygon_points,log_count);
sat = zeros(polygon_points,log_count);

for i=1:log_count
  [e_tmp, n_tmp, alt_tmp, ~, sat_tmp] = ...
    gpxlogger_time_to_UTM(xs(i),t(i,:));
  easting(:,i) = e_tmp;
  northing(:,i) = n_tmp;
  elevation(:,i) = alt_tmp;
  %utm_zone(:,i) = utm_z_tmp; % string problems
  sat(:,i) = sat_tmp;
end

%% Use relative values
% Take data for point 1 as (x,y,z) = (0,0,0)
for i=1:log_count  
  easting(:,i) = easting(:,i) - easting(1,i);
  northing(:,i) = northing(:,i) - northing(1,i);
  elevation(:,i) = elevation(:,i) - elevation(1,i);
end

%% Plot data
if(log_count ~= 6)
  error('Expecting 6 log files...')
end

for i = 1:log_count/3
  figure; subplot 311
  for j = 1:3
    fprintf('Sat. count for plot %d:\n\t',3*(i-1)+j)
    for k = 1:log_count
      fprintf('%d\t', sat(k,3*(i-1)+j))
    end
    fprintf('\n')
    subplot(3,1,j)
    plot(easting(:,3*(i-1)+j), northing(:,3*(i-1)+j),'x')
    title(sprintf('Polygon - log #%d',3*(i-1)+j))
    xlabel('Relative Easting (m)')
    ylabel('Relative Northing (m)')
    hold on
    % display initial position
    plot(easting(1,3*(i-1)+j), northing(1,3*(i-1)+j),'ro')
    plot(easting(1,3*(i-1)+j), northing(1,3*(i-1)+j),'ro', 'Markersize', 10)
    hold off
    grid on
  end
end
