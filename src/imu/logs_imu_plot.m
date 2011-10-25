function logs_imu_plot(frames,avg,avg_count)
% assuming:
%   frame:  sec usec x y z r p y
%   avg:    sec usec x y z r p y
frames_with_counts = 0;
avg_with_counts = 0;

%% trim down data
% Limitation is in size of avg, so trim down frames
frames_end_index= avg_count -1 + size(avg,1);
frames_trimmed = frames(avg_count:frames_end_index,:);

avg_time_usec = (avg(:,1) - avg(1,1))*1e6 + avg(:,2);% sec*1e6 + usec
frames_time_usec = (frames_trimmed(:,1) - frames_trimmed(1,1))*1e6  + frames_trimmed(:,2);
unmatching_timestamps = avg_time_usec ~= frames_time_usec;
not_sync_count = sum(unmatching_timestamps);

if(not_sync_count ~= 0)
  fprintf('ERROR: avg and frames not in sync %d times!\n',not_sync_count);
  user_in = input('Display indexes where not in sync?(1==yes,0==no)\n');
  if(user_in == 1)
    unsync_index = find(unmatching_timestamps~=0);
    display(unsync_index);
    if(not_sync_count ==1)
      user_in = input('Split data?(1==yes,0==no)\n');
      if(user_in == 1)
        assignin('base','avg1',avg(1:unsync_index-1,:));
        assignin('base','avg2', avg(unsync_index+1:end,:));
        assignin('base','avg_time_usec1', avg_time_usec(1:unsync_index-1,:));
        assignin('base','avg_time_usec2', avg_time_usec(unsync_index+1:end,:));
        assignin('base','frames1', frames(1:unsync_index-1,:));
        assignin('base','frames2', frames(unsync_index+1:end,:));
        assignin('base','frames_time_usec1', frames_time_usec(1:unsync_index-1,:));
        assignin('base','frames_time_usec2', frames_time_usec(unsync_index+1:end,:));
      end
    end    
  end
  figure(243)
  stem(avg_time_usec ~= frames_time_usec);
  title('Timestamp mismatches')
  xlabel('sample #')
  ylabel('high == mismatch; low == ok')
  return
end

%% plot
sensor = 'xyzrpy';
for i=1:6
  subplot (230 + i)
  plot(frames_time_usec, frames_trimmed(:,2+i+frames_with_counts),'r');
  hold on
  plot(avg_time_usec, avg(:,2+i+avg_with_counts),'b')
  hold off
  title(sprintf('%c',sensor(i)));
  legend('raw','avg');
  ylabel('ADC value (10bits)');
  xlabel('time (sec)');
  axis tight
end