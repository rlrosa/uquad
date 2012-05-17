set(main_bin main)
add_executable (${main_bin} main)

# Set required libraries
target_link_libraries(${main_bin} uquad_math)
target_link_libraries(${main_bin} motor)
target_link_libraries(${main_bin} uquad_kernel_msgq)
target_link_libraries(${main_bin} imu)
target_link_libraries(${main_bin} uquad_io)
target_link_libraries(${main_bin} uquad_time)
target_link_libraries(${main_bin} uquad_logger)
target_link_libraries(${main_bin} kalman)
target_link_libraries(${main_bin} control)
target_link_libraries(${main_bin} uquad_check_net)
target_link_libraries(${main_bin} path_planner)

# To enable/disable GPS, set USE_GPS to 1/0 in src/CMakelists.txt
if(${USE_GPS})
  message("Main using GPS...")
  add_definitions(-DUSE_GPS=${USE_GPS})
  target_link_libraries(${main_bin} gps_comm)
endif(${USE_GPS})
