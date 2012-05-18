#!/bin/bash -e

cd build
dest_platform=
debug=
for i in $*
do
    if [ "$i" == debug ]
    then
	echo "Setting up cmake with debug symbols..."
	debug="-DCMAKE_BUILD_TYPE=Debug"
    else
	if [ "$i" == arm ]
	then
	    echo "Cross compiling for arm"
	    dest_platform="-DCMAKE_TOOLCHAIN_FILE=../beagleboard.cmake"
	    cd ../arm_build
	    if [ ! "${OE_BUILD_TMPDIR}" ]
	    then
		echo Error! Add this line to "~/.bashrc": source ~/.oe/environment-oecore
		cd ..
		exit
	    else
		if [ ! "${OE_BEAGLE_TOOLCHAIN}" ]
		then
		    export OE_BEAGLE_TOOLCHAIN=$OE_BUILD_TMPDIR/sysroots/x86_64-linux/usr/bin/armv7a-angstrom-linux-gnueabi
		    echo Toolchain env var not found, setting it to $OE_BEAGLE_TOOLCHAIN
		fi
	    fi
	else
	    if [ "$i" == help ]
	    then
		echo "Usage: ./run_cmake.sh [debug] [arm] [help]"
		cd ..
		exit
	    fi
	fi
    fi
done

cmake $debug $dest_platform ..

cd ..
echo Done!