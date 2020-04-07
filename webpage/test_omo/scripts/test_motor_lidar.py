#!/usr/bin/env python

import serial  # for connect between computer&omorobot
import rospy   # to use ROS on python
from time import sleep  # to use sleep(it's like 'delay' from arudino but second)
from geometry_msgs.msg import Twist  # to use cmd_vel topic
from std_msgs.msg import String

# Variables
cycle_counter = 0
flag_motor_on = True
cmdV = 0
cmdW = 0
obstacle_state = "none"
flag_obstacle = False

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

def callbackLidar(msg):
    global obstacle_state
    global flag_obstacle

    obstacle_state = msg.data
    print("%s" % obstacle_state)
    flag_obstacle = True

def Robot():
    # Set global variables
    global cycle_counter
    global flag_motor_on
    global cmdV, cmdW
    global obstacle_state, flag_obstacle

    # Initialize 'omoros' node
    rospy.init_node('omoros', anonymous=True)

    # Set Subscribers
    rospy.Subscriber("cmd_vel", Twist, callbackCmdVel)
    rospy.Subscriber("lidar_obstacle", String, callbackLidar)

    # Check serial is opened
    if ser.isOpen():
        print("Serial is opened")
        sleep(1)  # wait for 1 second

    # Set control cycle
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

        if flag_obstacle == True:
#            if obstacle_state == "Forward":
#                cmdV, cmdW = 200, 0
#            elif obstacle_state == "Backward":
#                cmdV, cmdW = -100, 0
#            else: # Stop
#                cmdV, cmdW = 0, 0

            # Lidar avoidance # KYP
#            if obstacle_state == "Go":
#                cmdV, cmdW = 150, 0
#            elif obstacle_state == "Right":
#                cmdV, cmdW = 100, -150
#            elif obstacle_state == "Left":
#                cmdV, cmdW = 100, 150
#            else:
#                cmdV, cmdW = 0, 0

            # People tracking
            if obstacle_state == "Left":
                cmdV, cmdW = 0, 100
            elif obstacle_state == "Right":
                cmdV, cmdW = 0, -100
            elif obstacle_state == "Keep going":
                cmdV, cmdW = 70, 0
            else: # Stop
                cmdV, cmdW = 0, 0

            flag_obstacle = False
            print("%s, V:%d, W:%d" %(obstacle_state, cmdV, cmdW))

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
