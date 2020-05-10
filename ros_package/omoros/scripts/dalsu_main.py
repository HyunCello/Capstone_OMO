#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped


def goal_publisher():
    rospy.init_node("goal_publisher", anonymous=True)
    pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
    rate = rospy.Rate(10) # 10hz
    count = 0

    while not rospy.is_shutdown():
        goal = PoseStamped()

        if (count < 5):
            goal.header.seq = 1
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"

            goal.pose.position.x = 2.5
            goal.pose.position.y = 1.1
            goal.pose.position.z = 0.0

            goal.pose.orientation.x = 0.0
            goal.pose.orientation.y = 0.0
            goal.pose.orientation.z = -0.7
            goal.pose.orientation.w = 0.7

            pub.publish(goal)
            rospy.loginfo('Goal published!')
            count += 1

        rate.sleep()
        #rospy.spin()


if __name__ == '__main__':
    try:
        goal_publisher()
    except rospy.ROSInterruptException:
        pass