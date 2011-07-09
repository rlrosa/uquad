# Based on script:
#		Test for Razor 9DOF IMU
#		Jose Julio @2009

from visual import * # For display
import string # For parsing
import math # For math XD
import sys # For script arguments
import os # To check if path exists

from time import time

grad2rad = 3.141592/180.0

# Check your COM port and baud rate
#ser = serial.Serial(port='/dev/ttyUSB0',baudrate=115200, timeout=1)
if len(sys.argv) > 2:
    device = sys.argv[1]
else:
    device = '/dev/ttyUSB0'
print 'Opening %s ...' % (device)
try:
    imu_data = open(device,'r')
except:
    print 'Could not open device %s' % (device)
print 'Opened %s !' % (device)


print 'Loading graphics...'
# Main scene
try:
    scene=display(title="win-main",background=(1,1,1))
    scene.range=(1.2,1.2,1.2)
#scene.forward = (0,-1,-0.25)
    scene.forward = (1,0,-0.25)
    scene.up=(0,0,1)
    
# Second scene (Roll, Pitch, Yaw)
    scene2 = display(title='win-rpy',x=0, y=0, width=500, height=200,center=(0,0,0), background=(0,0,0))
    scene2.range=(1,1,1)
    scene.width=500
    scene.y=200

    scene2.select()
#Roll, Pitch, Yaw
    cil_roll = cylinder(pos=(-0.4,0,0),axis=(0.2,0,0),radius=0.01,color=color.red)
    cil_roll2 = cylinder(pos=(-0.4,0,0),axis=(-0.2,0,0),radius=0.01,color=color.red)
    cil_pitch = cylinder(pos=(0.1,0,0),axis=(0.2,0,0),radius=0.01,color=color.green)
    cil_pitch2 = cylinder(pos=(0.1,0,0),axis=(-0.2,0,0),radius=0.01,color=color.green)
#cil_course = cylinder(pos=(0.6,0,0),axis=(0.2,0,0),radius=0.01,color=color.blue)
#cil_course2 = cylinder(pos=(0.6,0,0),axis=(-0.2,0,0),radius=0.01,color=color.blue)
    arrow_course = arrow(pos=(0.6,0,0),color=color.cyan,axis=(-0.2,0,0), shaftwidth=0.02, fixedwidth=1)
    
#Roll,Pitch,Yaw labels
    label(pos=(-0.4,0.3,0),text="Roll",box=0,opacity=0)
    label(pos=(0.1,0.3,0),text="Pitch",box=0,opacity=0)
    label(pos=(0.55,0.3,0),text="Yaw",box=0,opacity=0)
    label(pos=(0.6,0.22,0),text="N",box=0,opacity=0,color=color.yellow)
    label(pos=(0.6,-0.22,0),text="S",box=0,opacity=0,color=color.yellow)
    label(pos=(0.38,0,0),text="W",box=0,opacity=0,color=color.yellow)
    label(pos=(0.82,0,0),text="E",box=0,opacity=0,color=color.yellow)
    label(pos=(0.75,0.15,0),height=7,text="NE",box=0,color=color.yellow)
    label(pos=(0.45,0.15,0),height=7,text="NW",box=0,color=color.yellow)
    label(pos=(0.75,-0.15,0),height=7,text="SE",box=0,color=color.yellow)
    label(pos=(0.45,-0.15,0),height=7,text="SW",box=0,color=color.yellow)
    
    L1 = label(pos=(-0.4,0.22,0),text="-",box=0,opacity=0)
    L2 = label(pos=(0.1,0.22,0),text="-",box=0,opacity=0)
    L3 = label(pos=(0.7,0.3,0),text="-",box=0,opacity=0)
    
# Main scene objects
    scene.select()
# Reference axis (x,y,z)
    arrow(color=color.green,axis=(1,0,0), shaftwidth=0.02, fixedwidth=1)
    arrow(color=color.green,axis=(0,-1,0), shaftwidth=0.02 , fixedwidth=1)
    arrow(color=color.green,axis=(0,0,-1), shaftwidth=0.02, fixedwidth=1)
# labels
    label(pos=(0,0,0.8),text="imu test",box=0,opacity=0)
    label(pos=(1,0,0),text="X",box=0,opacity=0)
    label(pos=(0,-1,0),text="Y",box=0,opacity=0)
    label(pos=(0,0,-1),text="Z",box=0,opacity=0)
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

roll=0
pitch=0
yaw=0
run = 1
while run==1:
    try:
        # IMU set to ASCII outputs data starting with 'A', separated by '\t' and
        # ending with "Z\n"
        # If all sensors are enabled then:
        # ['A', count, acc_x, acc_y, acc_z, pitch, roll, yaw, 'Z\n']
        # If for example pitch is disabled, then the data read would be:
        # ['A', count, acc_x, acc_y, acc_z, roll, yaw, 'Z\n']

        line = imu_data.readline()
        print line
        f.write(line)                     # Write to the output log file
        words = string.split(line,"\t")    # Fields split
        words_len = len(words)
        if words_len > 5:        
            try:
                roll_str = words[2]
                pitch_str = words[3]
                yaw_str = words[4]
                # !??!
                yaw = float(yaw_str)*grad2rad
                roll = (float(roll_str))*grad2rad
                pitch = float(pitch_str)*grad2rad
            except:
                print "Invalid line"
                
            axis=(cos(pitch)*cos(yaw),-cos(pitch)*sin(yaw),sin(pitch)) 
            up=(sin(roll)*sin(yaw)+cos(roll)*sin(pitch)*cos(yaw),sin(roll)*cos(yaw)-cos(roll)*sin(pitch)*sin(yaw),-cos(roll)*cos(pitch))
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
            L1.text = roll_str
            L2.text = pitch_str
            L3.text = yaw_str
        else:
            # If len<5 then not enough sensors are on.
            print "I need pitch, roll and yaw."
    except KeyboardInterrupt:
        run = 0
        print 'User interrupt! \n Please close windows to terminate...'
imu_data.close
f.close
