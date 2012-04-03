#!/bin/bash
# Check if comm with server is running, if not, kill cmd.

# wait for cmd startup
sleep 4

# check connections
while(true);
do
    x=`ping -c1 10.42.43.1 2>&1 | grep "100% packet loss"`
    if [ ! "$x" = "" ]; then
	echo "Connection lost!"
	kill -9 `pidof -s cmd`
	exit
    fi
    sleep 1
done