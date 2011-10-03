if(~exist('log_date','var'))
  warning(sprintf('\nSet string var log_date to select log.\nDo not include extension nor avg, just date.\n'));
  return
end

% Modify this to match IMU_FRAME_SAMPLE_AVG_COUNT in imu_comm.h
avg_count = 8;

logs_imu_import(sprintf('../build/test/imu_comm_test/logs/%s.log',log_date),'frames');
logs_imu_import(sprintf('../build/test/imu_comm_test/logs/%savg.log',log_date),'avg');

logs_imu_plot(frames,avg,avg_count)

