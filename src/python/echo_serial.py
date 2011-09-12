#!/usr/bin/env python
# Averiguar el nombre del conector usb en /dev/ttyUSB# (donde '#' es el numero que se le asigno) y correr el siguiente script con /dev/ttyUSB# como argumento.
import sys
import serial
import threading

if len(sys.argv) > 1:
    if sys.argv[1] == 'help':
        print 'Usage: ./serial_comm.py <device> <baudrate> <stopbits> <timeout>'
        quit()
    else:
        device = sys.argv[1]
else:
    device = '/dev/tty/USB0'
if len(sys.argv) > 2:
    baudrate_ = sys.argv[2]
else:
    baudrate_ = 115200
stopbits_ = serial.STOPBITS_ONE
if len(sys.argv) > 3:
    if(sys.argv[3]==2):
        stopbits_ = serial.STOPBITS_TWO
    else:
        if not (sys.argv[3]==1):
            print 'Ignoring stopbits value'
        
if len(sys.argv) > 4:
    timeout_ = sys.argv[2]
else:
    timeout_ = 0

print 'Opening %s ...' % (device)
try:
    ser = serial.Serial(port=device,baudrate=baudrate_, stopbits=stopbits_,timeout=timeout_)
    ser.open()
except IOerror:
    print 'Could not open device %s' % (device)
    quit()
print 'Opened %s:\nBaudrate: %s\nStopbits: %s\nTimeout: %s ' % (device,baudrate_,stopbits_,timeout_)

# Start

# Reading thread
def read_ser():
    line = ''
    while 1:
        line = ser.readline()
        sys.stdout.write(line)
        sys.stdout.flush()
        line = ''
        
errors = 0
max_errors = 20

read_thread = threading.Thread(target=read_ser)
read_thread.setDaemon(True)
read_thread.start()

# Main loop
escape_sequence = 'Ctrl+]'
print 'To terminate press %s followed by RET' % escape_sequence
while 1:
    try:
        cmd = raw_input()
        if(cmd == escape_sequence):
            print '\nEscape sequence read, terminating program...\n'
            quit()
        if(cmd == ''):
            ser.write('\n')
            continue
        if(cmd == '\x1d'):
            quit()
        ser.write(cmd+'\n')
        ser.flush()
    except KeyboardInterrupt: 
       ser.write('%c' % 0x03)

quit()
