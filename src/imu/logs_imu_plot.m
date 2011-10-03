function logs_imu_plot(frames,avg,avg_count)
%% trim down data
% Limitation is in size of avg, so trim down frames
frames_end_index= avg_count -1 + size(avg,1);
frames_trimmed = frames(avg_count:frames_end_index,:);

avg_time_sec = avg(:,1) + avg(:,2)/1e6;% sec + usec/1e6
avg_time_sec = avg_time_sec - avg_time_sec(1);
frames_time_sec = frames_trimmed(:,1) + frames_trimmed(:,2)/1e6;
frames_time_sec = frames_time_sec  - frames_time_sec(1);
if(sum(avg_time_sec ~= frames_time_sec ) ~= 0)
  display('ERROR: avg and frames not in sync!');
  return
end

%% plot
% assuming:
%   frame:  sec usec count x y z r p y
%   avg:    sec usec x y z r p y
sensor = 'xyzrpy';
for i=1:6
  subplot (230 + i)
  plot(frames_time_sec, frames_trimmed(:,3+i),'r');
  hold on
  plot(avg_time_sec, avg(:,2+i),'b')
  hold off
  title(sprintf('%c',sensor(i)));
  legend('raw','avg');
  ylabel('ADC value (10bits)');
  xlabel('time (sec)');
  axis tight
end