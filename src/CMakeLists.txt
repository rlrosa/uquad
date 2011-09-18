cmake_minimum_required (VERSION 2.8.3)
project (UQUAD)

include_directories(common imu)

# Generates a lib for imu_comm
add_subdirectory(imu)

# Check in test dir for deeper includes
add_subdirectory(test)
