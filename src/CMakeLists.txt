cmake_minimum_required (VERSION 2.8.3)
project (UQUAD)

# List all dirs with headers
include_directories(common imu)
# ignoring gpsd, since we may use opkg
#include_directories(common imu uquad_time submodules/gpsd)

# List external libs
# ignoring gpsd, since we may use opkg
#link_directories(submodules/gpsd)

# Generate libs
add_subdirectory(imu)
add_subdirectory(common)

# Check in test dir for deeper includes
add_subdirectory(test)
