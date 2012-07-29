#!/bin/bash -e
# Run main and whatever is required
# -e: Exit immediately if a command exits with a non-zero status.
#     ERR trap is inherited by shell functions.
# Args:
#   ./go.sh <serial_port> <do_pc_test> <log_path> <gps_path>

trap ctrl_c INT
function ctrl_c() {
    echo "** Trapped CTRL-C"
    killall cmd
    exit
}

# use correct calibration file
if [ ! -e `pwd`/build/main/imu_calib.txt ];
then
    ln -s `pwd`/imu/imu_calib.txt `pwd`/build/main/imu_calib.txt
fi

# use correct control gain
for file in control/*.txt
do
    file=${file##*/}
    if [ ! -e `pwd`/build/main/${file} ];
    then
	ln -s `pwd`/control/${file} `pwd`/build/main/${file}
	echo "Created link for ${file} in build/main"
    fi
done

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
if [ $3 ];
then
    log_path=$3
else
    log_path=/media/sda1/
fi
echo Will save logs to ${log_path}
if [ $3 ];
then
    gps_path=$3
else
    gps_path=/dev/ttyUSB0
fi
echo Will connect to GPS at ${gps_path}
if [ $4 ];
then
    pp_path=$4
    echo Loaded pp_path from ${pp_path}
else
    pp_path=
    echo No pp_path supplied.
fi

err_pipe=err.p

# build
cd i2c_beagle; make ${pc_test};
cd ../
cd build/main; make;
cd ../../

# prepare motor command
mv i2c_beagle/cmd${pc_test} build/main/cmd

# launch gpsd
#(cd ../scripts; ./start_gpsd.sh)
#sleep 1

# run main
echo ""
echo Running main...
echo ""
(cd build/main; ./main ${serial_port} ${log_path} ${gps_path} ${pp_path};echo "";echo "-- -- -- --";echo "Main finished!";echo "-- -- -- --";)

# kill everything. Muaha, ha.
ctrl_c
