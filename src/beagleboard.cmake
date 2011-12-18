# this one is important
SET(CMAKE_SYSTEM_NAME Linux)
#this one not so much
SET(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
SET(CMAKE_C_COMPILER   $ENV{OE_BEAGLE_TOOLCHAIN}/arm-angstrom-linux-gnueabi-gcc)
SET(CMAKE_CXX_COMPILER   $ENV{OE_BEAGLE_TOOLCHAIN}/arm-angstrom-linux-gnueabi-g++)

# where is the target environment 
SET(CMAKE_FIND_ROOT_PATH $ENV{OE_BEAGLE_TOOLCHAIN}/)

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)