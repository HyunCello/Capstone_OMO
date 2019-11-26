#!/usr/bin/env python

import serial  # for connect between computer&omorobot
import rospy   # to use ROS on python
from time import sleep  # to use sleep(it's like 'delay' from arudino but second)
from geometry_msgs.msg import Twist  # to use cmd_vel topic

# Variables
cycle_counter = 0
flag_motor_on = True
cmdV = 0
cmdW = 0

# Fetch /global parameters
param_port = '/dev/ttyUSB0'
param_baud = 115200

# Open Serial port with parameter settings
ser = serial.Serial(param_port, param_baud)

def callbackCmdVel(cmd):
    global cmdV, cmdW

    cmdV = cmd.linear.x
    cmdW = cmd.angular.z
    cmdV = cmdV * 100
    cmdW = cmdW * 50
    print("V : %f | W : %f" %(cmdV, cmdW))

def Robot():
    # Set global variables
    global cycle_counter
    global flag_motor_on
    global cmdV, cmdW

    # Initialize 'omoros' node
    rospy.init_node('omoros', anonymous=True)

    # Set Subscribers
    rospy.Subscriber("cmd_vel", Twist, callbackCmdVel)

    # Check serial is opened
    if ser.isOpen():
        print("Serial is opened")
        sleep(1)  # wait for 1 second

    # Set control cycle
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        if ser.isOpen():
            if flag_motor_on == True:
                msg = '$CVW,{:.0f},{:.0f}'.format(cmdV, cmdW)
                ser.write(msg+"\r"+"\n")
            else:
                ser.write("$CVW,"+"0,"+"0"+"\r"+"\n")

        #print("test %d" %cycle_counter)
        cycle_counter = cycle_counter + 1
        
        # Keep rate 30hz
        rate.sleep()

if __name__ == '__main__':
   try:
      Robot()
   except rospy.ROSInterruptException:
      pass
