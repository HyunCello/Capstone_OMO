#!/usr/bin/env python

import serial  # for connect between computer&omorobot
import rospy   # to use ROS on python
from time import sleep  # to use sleep(it's like 'delay' from arudino but second)

# Variables
cycle_counter = 0
flag_motor_on = False

def Robot():
    # Set global variables
    global cycle_counter
    global flag_motor_on

    # Initialize 'omoros' node
    rospy.init_node('omoros', anonymous=True)

    # Fetch /global parameters
    param_port = '/dev/ttyUSB0'
    param_baud = 115200
   
    # Open Serial port with parameter settings
    ser = serial.Serial(param_port, param_baud)

    # Check serial is opened
    if ser.isOpen():
        print("Serial is opened")
        sleep(1)  # wait for 1 second

    # Set control cycle
    #rate = rospy.Rate(rospy.get_param('~hz', 30)) # 30hz
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        print(flag_motor_on)
        if cycle_counter % 300 == 0:
            flag_motor_on ^= 1

        if ser.isOpen():
            if flag_motor_on == True:
                ser.write("$CVW,"+"100,"+"0"+"\r"+"\n")
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
