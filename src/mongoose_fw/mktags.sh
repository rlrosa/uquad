#!/bin/bash

# After running this script tell emacs about the tags:
#   M-x visit-tags-table RET $(ETAGS_DIR)/TAGS

if [ "$1" == "-a" ]; then
    CURR_DIR=`pwd`
    echo "Scanning $CURR_DIR recursively..."
    etags -a `find $CURR_DIR -name "*.cpp" -o -name "*.c" -o -name "*.h" -o -name "*.hpp"`
    exit
fi


ETAGS_DIR=/home/rrosa/sketchbook/ckdevices_Mongoose9DOF_Base_AHRS_1_1/

SCAN_DIRS=(.
           /usr/share/arduino/libraries)

cd $ETAGS_DIR

echo "Delete old database.."
rm -f TAGS

for i in ${SCAN_DIRS[*]}; do
    echo "Scanning:" $i;
    etags -a `find $i -name "*.cpp" -o -name "*.c" -o -name "*.h" -o -name "*.hpp" -o -name "*.pde"`                                                
done

echo "Done"
cd -
