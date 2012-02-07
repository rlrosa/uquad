function [easting, northing, alt, utm_zone, sat, ind] = ...
  gpxlogger_time_to_UTM(xs, times)
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% function [easting, northing, alt, utm_zone, sat, ind] = ...
%  gpxlogger_time_to_UTM(xs, times)
%
% Looks for timestamps TIMES, returning the corresponding UTM coordinates.
% Also returns the # of satelites that were available when data was taken.
% The index where each value was found is returned in ind
%
% Example:
%   xs = xmltools('1.log');
%   times = {'2011-12-21T19:05:33.526Z' '2011-12-21T19:06:14.526Z'};
%   [e,n,a,z] = gpxlogger_time_to_UTM(xs, times);
%
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

packet_count = size(xs.child(2).child(2).child(2).child,2);

found_count = 0;
times_len = length(times);

lat = zeros(times_len,1);
lon = zeros(times_len,1);
alt = zeros(times_len,1);
sat = zeros(times_len,1);
ind = zeros(times_len,1);

for trkpt_i = 1:packet_count
  time_str = xs.child(2).child(2).child(2).child(trkpt_i).child(2).value;
%% Uncomment this to pull only hh:mm:ss
%  date_start_char_ind = 12;
%  T = 'T';
%  if(time_str(date_start_char_ind) ~= T)
%    for i = 1:size(time_str,2)
%      if(time_str(i) == T)
%        date_start_char_ind = i;
%        break;
%      end
%    end
%  end
%  time_str = time_str(date_start_char_ind + 1:eend-5);
  
  for i = 1 + found_count: times_len
    if(strcmp(time_str, times(i)))
      curr = xs.child(2).child(2).child(2).child(trkpt_i);
      lat(i) = str2double(curr.attribs(1).value);
      lon(i) = str2double(curr.attribs(2).value);
      alt(i) = str2double(curr.child(1).value);
      ind(i) = trkpt_i;
      if (size(curr.child,2) >= 8)
        sat(i) = str2double(curr.child(5).value);
      else
        % sat count is not always logged
        sat(i) = -1;
      end
      found_count = found_count + 1;
      break;
    end
  end
  if(found_count == times_len)
    % we're done, break out of the main loop
    break;
  end
end

if(found_count ~= times_len)
  error('Did not find all timestamps!')
end

% Convert to UTM
[easting, northing, utm_zone] = deg2utm(lat, lon);


