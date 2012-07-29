#!/bin/bash -e
python attitude.py imu_data.log & > /dev/null
script -c "ssh root@10.42.43.2 -t \"./tito_imu.sh\"" -f imu_data.log
