#!/bin/bash -E
# Run main and whatever is required
# -E: Exit immediately if a command exits with a non-zero status.
#     ERR trap is inherited by shell functions.
# Args:
#   ./go.sh <serial_port> <do_pc_test>

trap ctrl_c INT
function ctrl_c() {
    echo "** Trapped CTRL-C"
    killall cmd
    killall check_net.sh
    echo "Please wait for logger to finish (25 sec)..."
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
cd i2c_beagle; make ${pc_test};
cd ../
cd build/main; make;
cd ../../

# use correct calibration file
if [ ! -e `pwd`/build/main/imu_calib.txt ];
then
    ln -s `pwd`/imu/imu_calib.txt `pwd`/build/main/imu_calib.txt
fi

# use correct control gain
for file in control/K*.txt
do
    file=${file##*/}
    if [ ! -e `pwd`/build/main/${file} ];
    then
	ln -s `pwd`/control/${file} `pwd`/build/main/${file}
	echo "Created link for ${file} in build/main"
    fi
done

# prepare motor command
mv i2c_beagle/cmd${pc_test} build/main/cmd

# launch gpsd
(cd ../scripts; ./start_gpsd.sh)
sleep 1

# run network check, kill cmd if network fails
echo Setting up check_net.sh
(cd ../scripts; ./check_net.sh &)
# wait for check net to start, or not
timer=4
while [ $timer -gt 0 ];
do
    echo "Wait ${timer} for check_net.sh..."
    sleep 1
    timer=$(( $timer - 1 ))
done
sleep 1

# run main
echo ""
echo Running main...
echo ""
(cd build/main; ./main ${serial_port};echo "";echo "-- -- -- --";echo "Main finished!";echo "-- -- -- --";)

# kill everything. Muaha, ha.
ctrl_c
