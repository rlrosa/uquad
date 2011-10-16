cmake_minimum_required (VERSION 2.8.3)
project (UQUAD)

# List all dirs with headers
include_directories(common imu uquad_time uquad_io)
# ignoring gpsd, since we may use opkg
#include_directories(common imu uquad_time submodules/gpsd)

# List external libs
# ignoring gpsd, since we may use opkg
#link_directories(submodules/gpsd)

# Generates a lib for imu_comm
add_subdirectory(imu)
add_subdirectory(uquad_time)
add_subdirectory(uquad_io)

# Check in test dir for deeper includes
add_subdirectory(test)
