#!/bin/bash

if [ "$1" == "-a" ]; then
    CURR_DIR=`pwd`
    echo "Scanning $CURR_DIR recursively..."
    etags -a `find $CURR_DIR -name "*.cpp" -o -name "*.c" -o -name "*.h" -o -name "*.hpp"`
    exit
fi

SCAN_DIRS=(imu
           motor
	   test
	   i2c_beagle
           common
           gps
           kernel_msgq
	   math
           submodules/gpsd)

echo "Delete old database.."
rm -f TAGS

for i in ${SCAN_DIRS[*]}; do
    echo "Scanning:" $i;
    etags -a `find ./$i -name "*.cpp" -o -name "*.c" -o -name "*.h" -o -name "*.hpp"`                                                
done

echo "Done"
cd -
