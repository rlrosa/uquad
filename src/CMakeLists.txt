cmake_minimum_required (VERSION 2.8.3)
project (UQUAD)

# List all dirs with headers
include_directories(common)
include_directories(motor)
include_directories(kernel_msgq)
include_directories(imu)
include_directories(math)
include_directories(check_net)
include_directories(gps)
include_directories(kalman)
include_directories(control)
include_directories(path_planner)
include_directories(logger)

# Add libm, for pow()
link_libraries(m)
# Add curses, for stdin input -> DISABLED, slows down beagleboard.
# link_libraries(curses)

# Add warning to compile flags:
set(${PROJECT_NAME}_C_FLAGS "-Wall" CACHE STRING
    "${PROJECT_NAME} C FLAGS")
# Enable the project's flags:
set(CMAKE_C_FLAGS "${${PROJECT_NAME}_C_FLAGS}")

set(USE_GPS 1)
if(${USE_GPS})
  message("Using GPS...")
  add_definitions(-DUSE_GPS=${USE_GPS})
endif(${USE_GPS})

# Generate libs
add_subdirectory(imu)
add_subdirectory(motor)
add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(kernel_msgq)
add_subdirectory(gps)
add_subdirectory(kalman)
add_subdirectory(main)
add_subdirectory(control)
add_subdirectory(path_planner)
add_subdirectory(logger)

# Check in test dir for deeper includes
add_subdirectory(test)

# Build check_net exec
add_subdirectory(check_net)
