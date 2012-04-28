# Reads and displays gyro data from state estimation log
# Arguments:
#   - Device name (optional).
#   Example:
#       python attitude.py /dev/ttyUSB0
# Based on script:
#		Test for Razor 9DOF IMU
#		Jose Julio @2009
from visual import * # For display
import serial # To read data from IMU
import string # For parsing
import math # For math XD
from math import atan2,cos,sin,asin,pi,copysign,sqrt # for vago
import random # for noise
import sys # For script arguments
import os # To check if path exists
from numpy import matrix,linalg

from time import sleep, time

# Constants
sep_symbol = '\t'
words_per_lin = 12
is_file = False
DEBUG = False

## Convert from degrees to rad
grad2rad = 3.141592/180.0
rad2grad = 180.0/3.141592

LOG_XHAT = 1
LOG_IMU_AVG = 2
LOG_IMU_DATA = 3

# Check your COM port and baud rate
if len(sys.argv) > 1:
    if sys.argv[1] == 'help':
        print 'Usage: ./attitude.py [<file> || <device>[<baudrate> <stopbits> <timeout>]]'
        quit()
    else:
        device = sys.argv[1]
        if(device[0:4] != '/dev'):
            is_file = True
            l = len(device);
            if(device[l-9:l-4] == 'x_hat'):
                log_type = LOG_XHAT
            elif (device[l-11:l-4] == 'imu_avg'):
                log_type = LOG_IMU_AVG
            elif (device[l-12:l-4] == 'imu_data'):
                log_type = LOG_IMU_DATA
            else:
                print 'Invalid log name!'
                exit
else:
    device = '/dev/ttyUSB0'
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
    timeout_ = 1

if not is_file:
    print 'Opening %s ...' % (device)
    try:
        imu_data = serial.Serial(port=device,baudrate=baudrate_, stopbits=stopbits_,timeout=timeout_)
        imu_data.open()
    except:
        print 'Could not open device %s' % (device)
        quit()
else:
    try:
        imu_data = open('%s' % device, 'r')
    except:
        print 'Could not open input file %s!' % device
        quit()

print 'Opened %s !' % (device)

print 'Loading graphics...'
# Main scene
try:
    scene=display(title="win-main",background=(1,1,1))
    scene.range=(1.2,1.2,1.2)
    scene.forward = (.25,0.25,-0.25) # 'Into' screen
    scene.up=(0,0,1)
    
    # Second scene (Roll, Pitch, Yaw)
    scene2 = display(title='win-rpy',x=0, y=0, width=500, height=200,center=(0,0,0), background=(0,0,0))
    scene2.range=(1,1,1)
    scene.width=500
    scene.y=200

    scene2.select()
    #Roll, Pitch, Yaw
    cil_roll = cylinder(pos=(-0.7,0,0),axis=(0.2,0,0),radius=0.01,color=color.red)
    cil_roll2 = cylinder(pos=(-0.7,0,0),axis=(-0.2,0,0),radius=0.01,color=color.red)
    cil_pitch = cylinder(pos=(0.0,0,0),axis=(0.2,0,0),radius=0.01,color=color.green)
    cil_pitch2 = cylinder(pos=(0.0,0,0),axis=(-0.2,0,0),radius=0.01,color=color.green)
    #cil_course = cylinder(pos=(0.6,0,0),axis=(0.2,0,0),radius=0.01,color=color.blue)
    #cil_course2 = cylinder(pos=(0.6,0,0),axis=(-0.2,0,0),radius=0.01,color=color.blue)
    arrow_course = arrow(pos=(0.6,0,0),color=color.cyan,axis=(-0.2,0,0), shaftwidth=0.02, fixedwidth=1)
    
    #Roll,Pitch,Yaw labels
    label(pos=(-0.7,0.3,0),text="Roll",box=0,opacity=0)
    label(pos=(0.0,0.3,0),text="Pitch",box=0,opacity=0)
    label(pos=(0.45,0.3,0),text="Yaw",box=0,opacity=0)
    label(pos=(0.6,0.22,0),text="N",box=0,opacity=0,color=color.yellow)
    label(pos=(0.6,-0.22,0),text="S",box=0,opacity=0,color=color.yellow)
    label(pos=(0.38,0,0),text="W",box=0,opacity=0,color=color.yellow)
    label(pos=(0.82,0,0),text="E",box=0,opacity=0,color=color.yellow)
    label(pos=(0.75,0.15,0),height=7,text="NE",box=0,color=color.yellow)
    label(pos=(0.45,0.15,0),height=7,text="NW",box=0,color=color.yellow)
    label(pos=(0.75,-0.15,0),height=7,text="SE",box=0,color=color.yellow)
    label(pos=(0.45,-0.15,0),height=7,text="SW",box=0,color=color.yellow)
    
    L1 = label(pos=(-0.7,0.22,0),text="-",box=0,opacity=0)
    L2 = label(pos=(0.0,0.22,0),text="-",box=0,opacity=0)
    L3 = label(pos=(0.7,0.3,0),text="-",box=0,opacity=0)
    
    # Main scene objects
    scene.select()
    # Reference axis (x,y,z)
    arrow(color=color.green,axis=(1,0,0), shaftwidth=0.04, fixedwidth=1)
    arrow(color=color.green,axis=(0,1,0), shaftwidth=0.02 , fixedwidth=1)
    arrow(color=color.green,axis=(0,0,1), shaftwidth=0.02, fixedwidth=1)
    # labels
    label(pos=(0,0.4,0.8),text="imu test",box=0,opacity=0,color=color.black)
    label(pos=(.5,0,0),text="X",box=0,opacity=0,color=color.black)
    label(pos=(0,.5,0),text="Y",box=0,opacity=0,color=color.black)
    label(pos=(0,0,.5),text="Z",box=0,opacity=0,color=color.black)
    L_calib = label(pos=(0,0,0),text='',box=0,opacity=0,height=30,color=color.black)
    # IMU object
    platform = box(length=1, height=0.05, width=1, color=color.red)
    p_line = box(length=1,height=0.08,width=0.1,color=color.yellow)
    plat_arrow = arrow(color=color.green,axis=(1,0,0), shaftwidth=0.06, fixedwidth=1)
except:
    print 'Failed to load graphics...'
print 'Graphics loaded !'
    
log_file_name = "Serial"+str(time())+".log"
log_path = 'logs'

print 'Opening log file named %s ...' % (log_file_name)
try:
    if not os.path.exists(log_path):
        os.makedirs(log_path)
    f = open('%s/%s' % (log_path,log_file_name), 'w')
except:
    print 'Failed to open log file %s ...' % (log_file_name)
print '%s opened!' % (log_file_name)

def gyro_read(data_str,zero):
    ## Convert data read from gyro to rad/s
    #
    # Gyro outputs ~1.65v for 0deg/sec, then this goes through a
    # 10bit ADC on the Atmega which compares 0-3.3v.
    # The data received is the result of the ADC.
    #
    # Note: Should be /300, but /450 seems to work better. This may
    # be wrong, but a possible explanation is that fullscale is not
    # at 0-3.3v.
    # This may need adjustment per sensor
    # 
    # @param data_str : String read from ADC conversion of gyro output
    # @param zero : Float of value read from the gyro when staying still.
    # 
    # @return Rate of turn (rad/sec)
    global grad2rad
    global gyro_adjust
    return gyro_adjust*(float(data_str)-zero)*grad2rad

## Uses acc readings to calculate pitch and roll
# Inputs in m/s**2
# Returns [pitch, roll]
def get_angle_acc(ax,ay,az):
    global gravity
    norm = sqrt(ax**2 + ay**2 + az**2)
    [ax,ay,az] = [ax/norm,ay/norm,az/norm]
    try:
        r = atan2(-ax,az)
        p = - atan2(ay,az)
    except:
        print 'Math error!'
    if(DEBUG):
        print 'acc:xyz: %.2f ||  %.2f ||  %.2f' % (ax,ay,az)
        print 'acc:pr:  %.2f ||  %.2f' % (p,r)
    return [p,r]

def m_T_q(roll,pitch,yaw):
    a = yaw
    b = pitch
    c = roll
    r1 = ([cos(a)*cos(b), cos(a)*sin(b)*sin(c) - cos(c)*sin(a), sin(a)*sin(c) + cos(a)*cos(c)*sin(b)])
    r2 = ([ cos(b)*sin(a), cos(a)*cos(c) + sin(a)*sin(b)*sin(c), cos(c)*sin(a)*sin(b) - cos(a)*sin(c)])
    r3 = ([       -sin(b),                        cos(b)*sin(c),                        cos(b)*cos(c)])
    R = array([r1,r2,r3])
    return R

contador = 0

# Data input loop
while 1:
    # IMU settings
    global calibrate
    global calibration_sample_size
    global T
    global mode
    global th
    # Kalman
    cov_pitch = 0.1
    cov_roll = 0.1
    cov_yaw = 0.1
    # Data storage
    roll=0
    pitch=0
    yaw=0
    roll_sensor=0
    pitch_sensor=0
    yaw_sensor=0
    roll_str = '0'
    pitch_str = '0'
    yaw_str = '0'
    roll_zero=-1
    pitch_zero=-1
    yaw_zero=-1
    dummy = 0.0
    request_printed = False
    loops = 0
    T = 10.0/10000 # in s

    while 1:
        contador += 1
        try:
            if loops == 0 or contador == 31500:
                loops = input('Time (s) between frames:(-1 for 10e-3)')
                if(loops > 0):
                    T = loops
                    loops = -1
                    continue
            if loops <= 0:
                sleep(T)

            line = imu_data.readline()
            if not len(line)>1:
                # No new data
                continue
            words = string.split(line,sep_symbol)    # Fields split
            words_len = len(words)
            if (words_len < words_per_lin):
                # Expects 12 entries per line:
                # x
                # y
                # z
                # psi
                # phi
                # theta
                # vqx
                # vqy
                # vqz
                # wqx
                # wqy
                # wqz
                if (not request_printed):
                    print 'Invalid data, please provide a correct log..'
                    request_printed = True
                    continue
                else:
                    request_printed = False
            print line
            f.write(line)# Write to the output log file
            # Input looks correct, one last check
            if (words_len < words_per_lin + 1):
                print 'Read too much data.\nGarbage?'
                continue
            try:
                if log_type == LOG_XHAT:
                    x      = float(words[0])
                    y      = float(words[1])
                    z      = float(words[2])
                    psi    = float(words[3])
                    phi    = float(words[4])
                    theta  = float(words[5])
                    vqx    = float(words[6])
                    vqy    = float(words[7])
                    vqz    = float(words[8])
                    wqx    = float(words[9])
                    wqy    = float(words[10])
                    wqz    = float(words[11])
                elif log_type == LOG_IMU_AVG or log_type == LOG_IMU_DATA:
                    ax     = float(words[3])
                    ay     = float(words[4])
                    az     = float(words[5])
                    gx     = float(words[6])
                    gy     = float(words[7])
                    gz     = float(words[8])
                    phi    = float(words[9])
                    psi    = float(words[10])
                    theta  = float(words[11])
                    temp   = float(words[12])
                    z      = float(words[13])                    
                
                roll  = psi
                pitch = phi
                yaw   = theta
                yaw = 0 # ignore
                
            except:
                print 'Invalid line: %s' % line

            # ZYX
            # Order: roll,pitch then yaw (must be in this order)
            # Rotate (1,0,0):
            R = m_T_q(roll,pitch,yaw)
            axis = (R[0,0],R[1,0],R[2,0])
            up   = (R[0,2],R[1,2],R[2,2])
            # pitch_display = -pitch
            # axis=(cos(yaw)*cos(pitch_display),
            #       sin(yaw)*cos(pitch_display),
            #       -sin(pitch_display))
            # up=(cos(yaw)*sin(pitch_display)*cos(roll) + sin(yaw)*sin(pitch_display),
            #     sin(yaw)*sin(pitch_display)*cos(roll) - cos(yaw)*sin(roll),
            #     cos(pitch_display)*cos(roll))

            if (DEBUG):
                print 'axis: %f\t%f\t%f' % (axis[0],axis[1],axis[2])
                print 'up: %f\t%f\t%f' % (up[0],up[1],up[2])
            platform.axis=axis
            platform.up=up
            platform.length=1.0
            platform.width=0.65
            plat_arrow.axis=axis
            plat_arrow.up=up
            plat_arrow.length=0.8
            p_line.axis=axis
            p_line.up=up
            cil_roll.axis=(0.2*cos(roll),0.2*sin(roll),0)
            cil_roll2.axis=(-0.2*cos(roll),-0.2*sin(roll),0)
            cil_pitch.axis=(0.2*cos(pitch),0.2*sin(pitch),0)
            cil_pitch2.axis=(-0.2*cos(pitch),-0.2*sin(pitch),0)
            arrow_course.axis=(0.2*sin(-yaw),0.2*cos(yaw),0)
            L1.text = str(roll*rad2grad % 360)[0:6]
            L2.text = str(pitch*rad2grad % 360)[0:6]
            L3.text = str(yaw*rad2grad % 360)[0:6]
        except KeyboardInterrupt:
            print 'Caught keyboard interrupt!'
            print 'Close display window to exit...'
            exit
        
imu_data.close()
f.close
