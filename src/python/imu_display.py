# Reads and displays gyro data from the Atomic IMU (sparkfun)
# Requieres a serial (UART) connection to the IMU.
# Arguments:
#   - Device name (optional).
#   Example:
#       python imu_display.py /dev/ttyUSB0
# Usage:
#   Press Ctrl+C to reset zero level.

# Based on script:
#		Test for Razor 9DOF IMU
#		Jose Julio @2009

from visual import * # For display
import serial # To read data from IMU
import string # For parsing
import math # For math XD
from math import atan2,cos,sin,asin,pi # for vago
import random # for noise
import sys # For script arguments
import os # To check if path exists
import threading # To read from serial as background task

from time import time

## Constants
gravity = 9.81
calibration_sample_size = 300 # enough
MODE_GYRO = 0
MODE_ACC = 1
MODE_KALMAN = 2
mode = MODE_KALMAN
# IMU setting
len_frec_line = 35 # Length of '5) Set output frequency, currently '
len_sens_line = 44 # Length of '4) Set accelerometer sensitivity, currently '
# IMU set to ASCII outputs data starting with 'A', separated by '\t' and
# ending with "Z\n"
frec = 80
T = 1.0/frec
start_symbol = '\rA'
end_symbol = 'Z\n'
sep_symbol = "\t"

## Convert from degrees to rad
grad2rad = 3.141592/180.0
rad2grad = 180.0/3.141592
## Adjust data read from imu
adc_bits = 10
vref = 3.3
volt_zero_rate_level = 1.65 # 5@uquad/doc/gyro/datasheet_LISY300AL.pdf
volt_max = 2**adc_bits-1
zero = volt_zero_rate_level * volt_max / vref
# Better off setting zero according to whatever bias caused by setup.

## Gyros
# Convert from voltage to degrees/sec
# Full scale is 300deg/sec, and in a 10 bit ADC that corresponds to 20**10-1=1023
# The adjustment is input_v*300/(1023-512) approx input_v*0.58708414872798431
gyro_adjust = 475.0/(1023.0-512.0)

## Acc
# Convert according to sens
# It is x mv per g
# sens - mV/g
# 1.5  - 800
# 2    - 600
# 4    - 400
# 6    - 300
# Go from V to mV and then use sens
sens = 1.5
#acc_adjust = 3.3/1023.0
acc_v_per_counts = 3.3/1023.0
acc_counts_per_v = 1023.0/3.3

# Check your COM port and baud rate
if len(sys.argv) > 1:
    if sys.argv[1] == 'help':
        print 'Usage: ./serial_comm.py <device> <baudrate> <stopbits> <timeout>'
        quit()
    else:
        device = sys.argv[1]
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

print 'Opening %s ...' % (device)
try:
    imu_data = serial.Serial(port=device,baudrate=baudrate_, stopbits=stopbits_,timeout=timeout_)
    imu_data.open()
except:
    print 'Could not open device %s' % (device)
    quit()
print 'Opened %s !' % (device)


print 'Loading graphics...'
# Main scene
try:
    scene=display(title="win-main",background=(1,1,1))
    scene.range=(1.2,1.2,1.2)
    #scene.forward = (0,-1,-0.25)
    scene.forward = (-1,-.01,-0.25)
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
    arrow(color=color.green,axis=(-1,0,0), shaftwidth=0.04, fixedwidth=1)
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
calibrate = False

print 'Opening log file named %s ...' % (log_file_name)
try:
    if not os.path.exists(log_path):
        os.makedirs(log_path)
    f = open('%s/%s' % (log_path,log_file_name), 'w')
except:
    print 'Failed to open log file %s ...' % (log_file_name)
print '%s opened!' % (log_file_name)

def rand_noise():
    return random.uniform(-1,1)*.0001

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
    g = gravity;
    try:
#        b = atan2(ay,ax)
#        a = asin(az/g)
#        gxz = g*cos(b)*cos(a)
#        gyz = g*sin(b)*cos(a)
#        p = atan2(az/gxz,ax/gxz)
#        r = atan2(az/gyz,ay/gyz)
        r = atan2(az,ax) - pi/2
        p = atan2(az,ay) - pi/2
    except:
        print 'Math error!'
    return [p,r]

# Return Volts pero g
def acc_v_per_g(sens):
    return 1.2/sens

# Returns m/s**2
def acc_volt_to_g(volts,sens):
    global gravity
    return volts/acc_v_per_g(sens)*gravity

# Returns in m/s**2
def acc_read(counts,zero,sens):
    global acc_v_per_counts
    global gravity
    volts = (counts-zero)*acc_v_per_counts
    return acc_volt_to_g(volts,sens)

def kalman(st,cov,gyro,acc_angle,T):
    # Tweak?
    proc_noise = 0.0001
    measure_noise = 0.001
    # Prediction
    st_ = st + gyro*T
    cov_ = cov + proc_noise
    # Update
    y = acc_angle - st_
    s = cov_ + measure_noise
    k = cov_/s
    st = st_ + k*y
    cov = (1-k)*cov_
    return [st,cov]
    
# Data input loop
def read_loop():
    # IMU settings
    global calibrate
    global calibration_sample_size
    global frec
    global T
    global sens
    global mode
    # Initial calibration
    calibrate = calibration_sample_size
    # Kalman
    cov_pitch = 0.1
    cov_roll = 0.1
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
    acc_x_zero = 470.0
    acc_y_zero = 500.0
    acc_z_zero = 560.0
    request_printed = False

    print 'Ctrl+C to reset zeros position'
    while 1:
        # If all sensors are enabled then:
        # ['A', count, acc_x, acc_y, acc_z, pitch, roll, yaw, 'Z\n']
        # If for example pitch is disabled, then the data read would be:
        # ['A', count, acc_x, acc_y, acc_z, roll, yaw, 'Z\n']
        # This script requires all sensors enabled (to simplify parsing.)
        
        line = imu_data.readline()
        if not len(line)>1:
            # No new data
            continue
        words = string.split(line,sep_symbol)    # Fields split
        words_len = len(words)
        if not ((words[0] == start_symbol) & (words[words_len-1] == end_symbol)):
            # Print out menu stuff
            print line
            try:
                if(len(line) > len_frec_line):
                    if(line[0:len_frec_line] == '5) Set output frequency, currently '):
                        words = string.split(line,' ')                    
                        frec = int(words[len(words)-1])
                        T = 1.0/frec
                        print 'Frequency detected: %d' % frec
                if(len(line) > len_sens_line):
                    if(line[0:len_sens_line] == '4) Set accelerometer sensitivity, currently '):
                        words = string.split(line,' ')
                        last_word = words[len(words)-1]
                        if (len(last_word) > 3):
                            sens = 1.5
                        else:
                            sens = float(last_word[0])
                        print 'Sensibility detected: %s' % str(sens)
            except:
                print 'Failed to adjust frec!'
            continue
        if (words_len < 9):
            # Parsing limited to all sensors enabled case
            if (not request_printed):
                print 'Please enable all sensors\nPress the spacebar followed by the Enter key to enter IMU menu.'
                request_printed = True
            continue
        else:
            request_printed = False
            print line
            f.write(line)# Write to the output log file
        # Input looks correct, one last check
        if (words_len > 9 ):
            print 'Read too much data.\nGarbage?'
            continue
        try:
            acc_x_str = words[2]
            acc_y_str = words[3]
            acc_z_str = words[4]
            pitch_str = words[5]
            roll_str = words[6]
            yaw_str = words[7]            
            if (mode==MODE_GYRO):
                # Gyro only
                pitch_sensor = gyro_read(pitch_str,pitch_zero)
                pitch = pitch_sensor*T + pitch + rand_noise()
                roll_sensor = gyro_read(roll_str,roll_zero)
                roll = roll_sensor*T + roll + rand_noise()
            elif (mode==MODE_ACC):
                # Acc only
                acc_x_sensor = acc_read(float(acc_x_str),acc_x_zero,sens)
                acc_y_sensor = acc_read(float(acc_y_str),acc_y_zero,sens)
                acc_z_sensor = acc_read(float(acc_z_str),acc_z_zero,sens)
                [pitch,roll] = get_angle_acc(acc_x_sensor,acc_y_sensor,acc_z_sensor)
            elif (mode == MODE_KALMAN):
                # Use Kalman filter to integrate gyros and acc for pitch and roll
                acc_x_sensor = acc_read(float(acc_x_str),acc_x_zero,sens)
                acc_y_sensor = acc_read(float(acc_y_str),acc_y_zero,sens)
                acc_z_sensor = acc_read(float(acc_z_str),acc_z_zero,sens)
                pitch_sensor = gyro_read(pitch_str,pitch_zero)
                roll_sensor = gyro_read(roll_str,roll_zero)
                [pitch_acc,roll_acc] = get_angle_acc(acc_x_sensor,acc_y_sensor,acc_z_sensor)
                [pitch,cov_pitch] = kalman(pitch,cov_pitch,pitch_sensor,pitch_acc,T)
                [roll,cov_roll] = kalman(roll,cov_roll,roll_sensor,roll_acc,T)
            else:
                print 'invalid mode'
                quit()
            # No cure for yaw drift
            yaw_sensor = gyro_read(yaw_str,yaw_zero)
            yaw = yaw_sensor*T + yaw + rand_noise()


        except:
            print 'Invalid line: %s' % line
        axis=(-cos(pitch)*cos(yaw),-cos(pitch)*sin(yaw),sin(pitch)) 
        up=(sin(roll)*sin(yaw)-cos(roll)*sin(pitch)*cos(yaw),-sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw),-cos(roll)*cos(pitch))
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
        arrow_course.axis=(0.2*sin(yaw),0.2*cos(yaw),0)
        L1.text = str(roll*rad2grad % 360)[0:6]
        L2.text = str(pitch*rad2grad % 360)[0:6]
        L3.text = str(yaw*rad2grad % 360)[0:6]

        if( calibrate == calibration_sample_size):
            # Reset accums
            pitch_zero_sum = 0
            roll_zero_sum = 0
            yaw_zero_sum = 0
            acc_x_zero_sum = 0
            acc_y_zero_sum = 0
            acc_z_zero_sum = 0
            L_calib.text = 'CALIBRATING...'
        if ( calibrate > 0 ):
            # Accumulate readings
            pitch_zero_sum = pitch_zero_sum + float(pitch_str)
            roll_zero_sum = roll_zero_sum + float(roll_str)
            yaw_zero_sum = yaw_zero_sum + float(yaw_str)
            acc_x_zero_sum = acc_x_zero_sum + float(acc_x_str)
            acc_y_zero_sum = acc_y_zero_sum + float(acc_y_str)
            acc_z_zero_sum = acc_z_zero_sum + float(acc_z_str)
            calibrate = calibrate - 1
            # Show current status
            if ((calibrate % 20)==0):
                # Flash text on imu display at a visible rate
                if(len(L_calib.text)>0):
                    L_calib.text = ''
                else:
                    L_calib.text = 'CALIBRATING...'
            print 'calibrating...'
        elif (calibrate == 0):
            # Set the mean as a zero ref
            pitch_zero = pitch_zero_sum/calibration_sample_size
            roll_zero = roll_zero_sum/calibration_sample_size
            yaw_zero = yaw_zero_sum/calibration_sample_size
            pitch = 0
            roll = 0
            yaw = 0
            print 'Gyros calibrated!'
            acc_x_zero = acc_x_zero_sum/calibration_sample_size
            acc_y_zero = acc_y_zero_sum/calibration_sample_size
            acc_z_zero = acc_z_zero_sum/calibration_sample_size
            # Fix z, since g=1g instead of zero (device flat)
            counts_per_g = acc_counts_per_v*acc_v_per_g(sens)
            acc_z_zero = acc_z_zero - counts_per_g
            L_calib.text = ''
            print 'Accs calibrated!'
            calibrate = calibrate - 1

# Run read loop
read_thread = threading.Thread(target=read_loop)
read_thread.setDaemon(True)
read_thread.start()# Command exec loop

while 1:
    try:
        cmd = raw_input()
        imu_data.write(cmd)
    except KeyboardInterrupt:
        print 'Calibrating gyros...'
        calibrate = calibration_sample_size
    except:
        print 'error!'
        if errors > max_errors:
            print 'Too many errors, terminating...'
            break
        else:
            errors = errors + 1
            
imu_data.close()
f.close
quit()
