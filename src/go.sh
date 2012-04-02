trap ctrl_c INT
function ctrl_c() {
        echo "** Trapped CTRL-C"
	killall logger
}

if [ $1 ];
then
    serial_port=$1
else
    serial_port=/dev/ttyS1
fi
err_pipe=err.p

# configure serial port
stty -F ${serial_port} 115200 -echo raw
sleep 0.5

# build
(cd build/logger; make;)
(cd i2c_beagle; make;)
(cd build/main; make;)

# use correct calibration file
cp imu/imu_calib.txt build/main/

# prepare logger
cp build/logger/logger build/main

# prepare motor command
cp i2c_beagle/cmd build/main/
# prepare pc_test motor command
# (cd i2c_beagle; make pc_test; cp cmd_pc_test_debug ../build/main/cmd;)

# set up stderr logger
if ! [ -a build/main/${err_pipe} ];
then
    mkfifo build/main/${err_pipe}
fi

# run main, route stderr to pipe
(cd build/main; ./logger ${err_pipe}; ./main ${serial_port} 2> ${err_pipe})