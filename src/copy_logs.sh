#!/bin/bash

if [ -f build ];
then
    echo build dir not found!
    exit
fi

if [ -f build/main ];
then
    echo main dir not found!
    exit
fi

cd ../tests/main/logs
if [ -e $1 ];
then
    echo Would overwrite logs! Aborting...!
else
    mkdir $1
    cd $1
    mv ../../../../src/build/main/i2c.dev .
    mv ../../../../src/build/main/imu_data.log .
    mv ../../../../src/build/main/imu_raw.log .
    mv ../../../../src/build/main/imu_avg.log .
    mv ../../../../src/build/main/kq_s_ack.log .
    mv ../../../../src/build/main/kq_s_data.log .
    mv ../../../../src/build/main/w.log .
    mv ../../../../src/build/main/cmd_rx.log .
    mv ../../../../src/build/main/cmd_tx.log .
    mv ../../../../src/build/main/cmd_output.log .
    mv ../../../../src/build/main/x_hat.log .
    mv ../../../../src/build/main/kalman_in.log .
    mv ../../../../src/build/main/tv.log .
    cd ../
    echo success!
fi
cd ../../../src
