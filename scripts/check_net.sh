#!/bin/bash
# Check if comm with server is running, if not, kill motors.
while(true);
do
    x=`ping -c1 10.42.43.1 2>&1 | grep unknown`
    if [ ! "$x" = "" ]; then
	kill -9 `pidof -s cmd`
    else
	echo ok!
    fi
    sleep 1
done