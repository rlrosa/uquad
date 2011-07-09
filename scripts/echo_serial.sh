#!/bin/bash
# Averiguar el nombre del conector usb en /dev/ttyUSB# (donde '#' es el numero que se le asignó) y correr el siguiente script con /dev/ttyUSB# como argumento.
if [ -e $1 ]
then
    echo opened $1
else
    echo $1 not found.
    exit
fi
DEVICE=$1
SHOW=0
cat $DEVICE &
while IFS="";read myline
do
{
    echo $myline > $DEVICE
}
done