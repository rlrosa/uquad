#!/bin/bash -E
# Check if comm with server is running, if not, kill cmd.

# wait for cmd startup
sleep 4

# check connections
while(true);
do
    x=`ping -c3 10.42.43.1 2>&1 | grep "100% packet loss"`
    if [ ! "$x" = "" ]; then
	echo "check_net.sh: Connection lost!"
	echo "Will kill cmd!"
	kill -9 `pidof -s cmd`
	exit
    fi
    sleep 1
done