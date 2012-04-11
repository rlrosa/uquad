#!/bin/bash
# Run main and whatever is required
# Args:
#   ./go.sh <serial_port> <do_pc_test>

trap ctrl_c INT
function ctrl_c() {
    echo "** Trapped CTRL-C"
    killall cmd
    killall host
    killall check_net.sh
    echo Please wait 5 sec for logger to finish...
    sleep 3
    echo 2 sec...
    sleep 2
    killall logger
    exit
}

if [ $1 ];
then
    serial_port=$1
else
    serial_port=/dev/ttyS1
fi
echo Will connect to IMU at ${serial_port}
if [ $2 ];
then
    pc_test="pc_test"
    echo Using fake motor driver
else
    pc_test=""
    echo Using REAL motor driver
fi

err_pipe=err.p

# configure serial port
echo Configuring ${serial_port} for IMU
stty -F ${serial_port} 115200 -echo raw
sleep 0.5

# build
(cd build/logger; make;)
(cd i2c_beagle; make ${pc_test};)
(cd build/main; make;)

# use correct calibration file
cp imu/imu_calib.txt build/main/

# prepare logger
cp build/logger/logger build/main

# prepare motor command
mv i2c_beagle/cmd${pc_test} build/main/cmd

# set up stderr logger
if ! [ -a build/main/${err_pipe} ];
then
    mkfifo build/main/${err_pipe}
fi

# run network check, kill cmd if network fails
x=`uname -a | grep "x86_64"`
if [ "$x" = "" ]; then
    echo Setting up check_net.sh
    (cd ../scripts; ./check_net.sh &)
    echo check_net.sh running...
else
    echo WARNING! check_net.sh will not be used, assuming this is not a beagleboard
fi

# run logger for errors
(cd build/main; ./logger ${err_pipe} &)

# run main
(cd build/main; ./main ${serial_port} 2> ${err_pipe};echo "Main finished!";)

# kill everything. Muaha, ha.
ctrl_c
