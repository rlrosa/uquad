#!/bin/bash
if [ -z "$1" ]
then
    echo "Setting up cmake WITHOUT debug symbols"
    (cd build ; cmake ..)
else
    if [ "$1" == debug ]
    then
	echo "Setting up cmake with debug symbols"
	(cd build ; cmake .. -DCMAKE_BUILD_TYPE=Debug)
    else
	echo "Invalid argument"
    fi
fi
     