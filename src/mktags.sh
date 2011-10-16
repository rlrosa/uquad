#!/bin/bash

if [ "$1" == "-a" ]; then
    CURR_DIR=`pwd`
    echo "Scanning $CURR_DIR recursively..."
    etags -a `find $CURR_DIR -name "*.cpp" -o -name "*.c" -o -name "*.h" -o -name "*.hpp"`
    exit
fi


ETAGS_DIR=/home/rrosa/work/uquad/src/


SCAN_DIRS=(imu
           common
           control
           gps
           submodules/gpsd
	   uquad_io
           imu_fw)

cd $ETAGS_DIR

echo "Delete old database.."
rm -f TAGS

for i in ${SCAN_DIRS[*]}; do
    echo "Scanning:" $i;
    etags -a `find $ETAGS_DIR/$i -name "*.cpp" -o -name "*.c" -o -name "*.h" -o -name "*.hpp"`                                                
done

echo "Done"
cd -
