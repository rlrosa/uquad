#!/bin/bash -e
(cd ../src/build/check_net/;make;./server &; sleep 2)
python attitude.py imu_data.log & > /dev/null
script -f imu_data.log
ssh root@10.42.43.2 -t "./tito_kalman.sh"

