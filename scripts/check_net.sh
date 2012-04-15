#!/bin/bash -E
# Check if comm with server is running, if not, kill cmd.

# Check if we have working wireless connection
# If not, then don't bother with this (may be using eth)
x=`ping -c1 10.42.43.1 2>&1 | grep "100% packet loss"`
if [ ! "$x" = "" ]; then
    echo "No connection, detected, will not control cmd..."
    exit
fi
echo "check_net.sh running..."
sleep 1

# check connections
while(true);
do
    x=`ping -c1 10.42.43.1 2>&1 | grep "100% packet loss"`
    if [ ! "$x" = "" ]; then
	echo "check_net.sh: Connection lost!"
	echo "Will kill cmd!"
	kill -9 `pidof -s cmd`
	exit
    fi
    sleep 1
done