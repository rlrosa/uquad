#!/bin/bash -e
# start server
if [ ! `pidof -s server` = "" ]; then
    echo "Server already running..."
else
    cd ../src/build/check_net
    make
    ./server &
    cd -
    # wait for server
    echo "Wait for server to start..."
    sleep 2
    echo "Server running"
fi
python attitude.py x_hat.log & > /dev/null
script -c 'ssh root@10.42.43.2 -t "./tito_kalman.sh"' -f x_hat.log
