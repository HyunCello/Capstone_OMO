#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

obstacle_state = 'Stop'

def callback(data):
#    rospy.loginfo("%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f"\
#                    %(data.ranges[0], data.ranges[1], data.ranges[2], \
#                      data.ranges[3], data.ranges[4], data.ranges[5], \
#                      data.ranges[6], data.ranges[7], data.ranges[8], data.ranges[9]))
    global obstacle_state
    
    distance = 0
    for i in range(115, 129): # Center 5 degrees
        distance = distance + data.ranges[i]
    distance = distance / 10.0
    if distance < 0.3:
        obstacle_state = 'Backward'
    elif distance > 0.45:
        obstacle_state = 'Forward'
    else:
        obstacle_state = 'Stop'

    rospy.loginfo("%s"%obstacle_state)

def listener():
    global obstacle_state

    rospy.init_node('ydlidar_listener', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback)
    pub = rospy.Publisher('lidar_obstacle', String, queue_size=10)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        pub.publish(obstacle_state)
        rate.sleep()
#    rospy.spin()

if __name__ == '__main__':
    listener()
