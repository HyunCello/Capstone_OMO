#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import time
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


lidar_msg = ''

def is_far(Sensor_value, Threshold, reverse):
    far = (Sensor_value >= Threshold) or (Sensor_value < 0.1)
    if reverse == 1:
        return not(far)
    else:
        return far

def callback(msg):
    global lidar_msg
    #rospy.loginfo('min, max angle (deg) : %f, %f', np.degrees(msg.angle_min), np.degrees(msg.angle_max))
    #rospy.loginfo('min, max range (m) : %f, %f', msg.range_min, msg.range_max)
    #rospy.loginfo('range num : %d', len(msg.ranges))
    #rospy.loginfo('angle_increment (deg) : %f', np.degrees(msg.angle_increment))
    range_front = np.mean(msg.ranges[115:125])
    range_left = np.mean(msg.ranges[230:])
    range_right = np.mean(msg.ranges[:10])
    rospy.loginfo('range - front : %f, left : %f, right : %f', range_front, range_left, range_right)

    if is_far(range_front, 0.6, 0):
        if (is_far(range_left, 0.6, 0) and is_far(range_right, 0.6, 0)) or (is_far(range_left, 0.6, 1) and is_far(range_right, 0.6, 1)):
            #rospy.loginfo('Go')
            lidar_msg = 'Go'
        elif (is_far(range_left, 0.6, 0) and is_far(range_right, 0.6, 1)):
            #rospy.loginfo('Left')
            lidar_msg = 'Left'
        elif (is_far(range_left, 0.6, 1) and is_far(range_right, 0.6, 0)):
            #rospy.loginfo('Right')
            lidar_msg = 'Right'
    else:
        if (is_far(range_left, 0.6, 0) and is_far(range_right, 0.6, 0)) or (is_far(range_left, 0.6, 1) and is_far(range_right, 0.6, 1)):
            #rospy.loginfo('Stop')
            lidar_msg = 'Stop'
        elif (is_far(range_left, 0.6, 0) and is_far(range_right, 0.6, 1)):
            #rospy.loginfo('Left')
            lidar_msg = 'Left'
        elif (is_far(range_left, 0.6, 1) and is_far(range_right, 0.6, 0)):
            #rospy.loginfo('Right')
            lidar_msg = 'Right'
        

rospy.init_node('lidar_analyser')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('lidar_obstacle', String, queue_size=10)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish(lidar_msg)
    rospy.loginfo('published : %s', lidar_msg)
    rate.sleep()